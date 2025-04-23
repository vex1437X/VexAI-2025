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
    """

    def __init__(self, serialHelper=None):
        super().__init__(serialHelper)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)

        # configure & start camera
        self._configure_pipeline()

        # multi‑ring tracker (allows up to 5 dropped frames)
        self.tracker = CentroidTracker(max_disappeared=5)

    def _configure_pipeline(self):
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        try:
            self.pipeline.start(self.config)
            print("RealSense pipeline started.")
        except RuntimeError as e:
            print(f"⚠️  Could not start RealSense pipeline: {e}")
            raise

    def process_frame(self):
        """
        Process a frame and return a dict of tracked rings:
           { track_id: (x_offset, z_offset), … }
        """
        try:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                return {}

            color_image = np.asanyarray(color_frame.get_data())
            mask = self._generate_mask(color_image)

            # find all ring‐candidate centroids
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                mask
            )
            input_cs = []
            for i in range(1, num_labels):
                area = stats[i, cv2.CC_STAT_AREA]
                if area < 500:  # ignore small noise
                    continue
                (cx, cy) = centroids[i]
                input_cs.append((int(cx), int(cy)))

            # update track
            objects = self.tracker.update(input_cs)

            # deproject each track to (x, z)
            results = {}
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            for oid, (cx, cy) in objects.items():
                depth = depth_frame.get_distance(cx, cy)
                if depth == 0:
                    continue
                x, _, z = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth)
                results[oid] = (x, z)

            # print number of rings detected
            # num_rings = len(results)
            # if num_rings > 0:
            #     print(f"Detected {num_rings} ring(s): {results}")
            # else:
            #     print("No rings detected.")
            return results

        except Exception as e:
            print(f"Error in Vision.process_frame: {e}")
            return {}

    def _generate_mask(self, color_image):
        # Lab a* for red
        lab = cv2.cvtColor(color_image, cv2.COLOR_BGR2Lab)
        a_chan = lab[:, :, 1]
        _, red_mask = cv2.threshold(a_chan, 150, 255, cv2.THRESH_BINARY)

        # HSV saturation to reject whites
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        s_chan = hsv[:, :, 1]
        _, sat_mask = cv2.threshold(s_chan, 50, 255, cv2.THRESH_BINARY)

        # combine + clean up
        mask = cv2.bitwise_and(red_mask, sat_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def stop(self):
        self.pipeline.stop()
        print("RealSense pipeline stopped.")
