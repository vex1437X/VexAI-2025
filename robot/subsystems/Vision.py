import pyrealsense2 as rs
import numpy as np
import cv2
import sys
from scipy.spatial import distance as dist
from robot.util.Subsystem import Subsystem


class CentroidTracker:
    """
    Tracks object centroids across frames, allowing temporary disappearances.
    """

    def __init__(self, max_disappeared=5):
        self.next_object_id = 0
        self.objects = dict()  # object_id -> centroid
        self.disappeared = dict()  # object_id -> consecutive missing count
        self.max_disappeared = max_disappeared

    def register(self, centroid):
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1

    def deregister(self, object_id):
        del self.objects[object_id]
        del self.disappeared[object_id]

    def update(self, input_centroids):
        # no detections → mark all existing as disappeared
        if len(input_centroids) == 0:
            for oid in list(self.disappeared):
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)
            return self.objects

        # first frame or no existing tracks → register all
        if len(self.objects) == 0:
            for c in input_centroids:
                self.register(c)
            return self.objects

        # otherwise match input centroids to existing ones
        object_ids = list(self.objects.keys())
        object_centroids = list(self.objects.values())
        D = dist.cdist(np.array(object_centroids), np.array(input_centroids))

        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]

        used_rows, used_cols = set(), set()
        for r, c in zip(rows, cols):
            if r in used_rows or c in used_cols:
                continue
            # skip if jump is too large
            if D[r, c] > 50:
                continue
            oid = object_ids[r]
            self.objects[oid] = input_centroids[c]
            self.disappeared[oid] = 0
            used_rows.add(r)
            used_cols.add(c)

        # any unmatched existing → disappeared
        for r in set(range(len(object_centroids))) - used_rows:
            oid = object_ids[r]
            self.disappeared[oid] += 1
            if self.disappeared[oid] > self.max_disappeared:
                self.deregister(oid)

        # any unmatched new centroids → register
        for c in set(range(len(input_centroids))) - used_cols:
            self.register(input_centroids[c])

        return self.objects


class Vision(Subsystem):
    """
    Vision subsystem for detecting (and tracking) red rings using an Intel RealSense depth camera.

    A bounding‑box width/height ratio filter has been added so only blobs whose
    width is approximately `desired_ratio * height` are considered valid. This
    helps reject tall/skinny or square noise blobs while keeping ring‑shaped
    detections.
    """

    def __init__(
        self,
        serialHelper=None,
        desired_ratio: float = 2.0,
        ratio_tolerance: float = 1,
    ):
        """
        :param desired_ratio: Ideal width/height ratio (w / h). 2.0 means width ≈ 2× height.
        :param ratio_tolerance: Acceptable fractional deviation from desired_ratio.
                                e.g. 0.25 → ±25 % margin, so 2.0 ± 0.5.
        """
        super().__init__(serialHelper)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)

        self.frame_done = False
        self.results = {}

        # ratio‑filter settings
        self.desired_ratio = desired_ratio
        self.ratio_tolerance = ratio_tolerance

        # configure & start camera
        self._configure_pipeline()

        # multi‑ring tracker (allows up to 5 dropped frames)
        self.tracker = CentroidTracker(max_disappeared=5)

    # ---------------------------------------------------------------------
    # Camera configuration
    # ---------------------------------------------------------------------
    def _configure_pipeline(self):
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        try:
            self.pipeline.start(self.config)
            print("RealSense pipeline started.")
        except RuntimeError as e:
            print(f"⚠️  Could not start RealSense pipeline: {e}")
            raise

    # ---------------------------------------------------------------------
    # Main per‑frame processing
    # ---------------------------------------------------------------------
    def process_frame(self, min_blob_size: int = 5000):
        """
        Process a frame and return a dict of tracked rings:
           { track_id: (x_offset, z_offset), … }

        Filtering criteria:
          • Blob area >= ``min_blob_size``
          • Bounding‑box ``width / height`` within ``desired_ratio ± ratio_tolerance * desired_ratio``
        """

        if self.frame_done:
            return self.results
        try:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                return {}

            color_image = np.asanyarray(color_frame.get_data())
            mask = self._generate_mask(color_image)

            # -----------------------------------------------------------------
            # Connected‑components analysis to locate ring candidates
            # -----------------------------------------------------------------
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                mask
            )
            input_cs = []
            for i in range(1, num_labels):
                area = stats[i, cv2.CC_STAT_AREA]
                if area < min_blob_size:
                    continue  # ignore small noise

                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]
                if h == 0:
                    continue  # avoid division by zero
                ratio = w / float(h)

                # apply ratio filter
                if (
                    abs(ratio - self.desired_ratio)
                    > self.ratio_tolerance * self.desired_ratio
                ):
                    continue

                (cx, cy) = centroids[i]
                input_cs.append((int(cx), int(cy)))

            # update centroid tracker with filtered detections
            objects = self.tracker.update(input_cs)

            # -----------------------------------------------------------------
            # Deproject each track to (x, z) world offsets
            # -----------------------------------------------------------------
            self.results = {}
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            for oid, (cx, cy) in objects.items():
                depth = depth_frame.get_distance(cx, cy)
                if depth == 0:
                    continue
                x, _, z = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth)
                self.results[oid] = (x, z)

            # -----------------------------------------------------------------
            # --- Diagnostic visualisation ------------------------------------
            # -----------------------------------------------------------------
            cv2.imshow("Color Image", color_image)
            cv2.imshow("Red Ring Mask", mask)

            output_image = color_image.copy()
            for i in range(1, num_labels):
                area = stats[i, cv2.CC_STAT_AREA]
                if area < min_blob_size:
                    continue

                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]
                if h == 0:
                    continue
                ratio = w / float(h)
                if (
                    abs(ratio - self.desired_ratio)
                    > self.ratio_tolerance * self.desired_ratio
                ):
                    continue

                x, y = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
                cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    output_image,
                    f"{area}",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

            cv2.imshow("Detections with Bounding Boxes", output_image)
            cv2.waitKey(1)  # small delay to allow GUI update

            self.frame_done = True
            return self.results

        except Exception as e:
            print(f"Error in Vision.process_frame: {e}")
            return {}

    # ---------------------------------------------------------------------
    # Helper functions
    # ---------------------------------------------------------------------
    def _generate_mask(self, color_image: np.ndarray) -> np.ndarray:
        """Return a binary mask for red regions in *color_image* (HSV threshold)."""
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 100, 90), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 100, 90), (179, 255, 255))
        return cv2.bitwise_or(mask1, mask2)

    def stop(self):
        """Stop camera pipeline and close OpenCV windows."""
        self.pipeline.stop()
        cv2.destroyAllWindows()
        print("RealSense pipeline stopped.")
