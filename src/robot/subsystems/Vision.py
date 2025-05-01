from src.core.subsystems.Subsystem import Subsystem
from src.core.hardware.RealSenseInterface import RealSenseInterface
from src.robot.hardware.RealSenseCamera import RealSenseCamera
import numpy as np
import cv2
from scipy.spatial import distance as dist
import pyrealsense2 as rs # type: ignore


class CentroidTracker:
    """
    Tracks object centroids across frames, allowing temporary disappearances.
    """

    def __init__(self, max_disappeared=5):
        self.next_object_id = 0
        self.objects = {}         # object_id -> centroid
        self.disappeared = {}     # object_id -> consecutive missing count
        self.max_disappeared = max_disappeared

    def register(self, centroid):
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1

    def deregister(self, object_id):
        del self.objects[object_id]
        del self.disappeared[object_id]

    def update(self, input_centroids):
        if len(input_centroids) == 0:
            for oid in list(self.disappeared):
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)
            return self.objects

        if len(self.objects) == 0:
            for c in input_centroids:
                self.register(c)
            return self.objects

        object_ids = list(self.objects.keys())
        object_centroids = list(self.objects.values())
        D = dist.cdist(np.array(object_centroids), np.array(input_centroids))

        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]

        used_rows, used_cols = set(), set()
        for r, c in zip(rows, cols):
            if r in used_rows or c in used_cols:
                continue
            if D[r, c] > 50:
                continue
            oid = object_ids[r]
            self.objects[oid] = input_centroids[c]
            self.disappeared[oid] = 0
            used_rows.add(r)
            used_cols.add(c)

        for r in set(range(len(object_centroids))) - used_rows:
            oid = object_ids[r]
            self.disappeared[oid] += 1
            if self.disappeared[oid] > self.max_disappeared:
                self.deregister(oid)

        for c in set(range(len(input_centroids))) - used_cols:
            self.register(input_centroids[c])

        return self.objects


class Vision(Subsystem):
    """
    Vision subsystem for detecting (and tracking) red rings using an Intel RealSense depth camera.
    """

    def __init__(self, camera: RealSenseInterface = None, min_blob_size: int = 1500):
        self.camera = camera or RealSenseCamera.get_instance()
        self.min_blob_size = min_blob_size
        self.tracker = CentroidTracker(max_disappeared=5)

    def on_enable(self) -> None:
        # Configure camera when subsystem is enabled
        # Expect camera_settings provided via RobotContainer.configure
        pass

    def on_disable(self) -> None:
        # Stop camera and close windows when disabled
        self.camera.close()
        cv2.destroyAllWindows()

    def process_frame(self) -> dict:
        """
        Capture a frame, generate red-ring mask, track centroids, and deproject to real-world coords.
        Returns:
            Dict[int, Tuple[float, float]]: mapping of track_id to (x, z) offsets.
        """
        try:
            # Acquire aligned color and depth frames from camera
            color_image, depth_frame = self.camera.get_frame()
            if color_image is None or depth_frame is None:
                return {}

            # Generate mask for red rings
            mask = self._generate_mask(color_image)

            # Display for debugging
            cv2.imshow("Color", color_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

            # Detect blobs and compute centroids
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
            input_centroids = [tuple(map(int, centroids[i]))
                               for i in range(1, num_labels)
                               if stats[i, cv2.CC_STAT_AREA] >= self.min_blob_size]

            # Update tracked objects
            tracked = self.tracker.update(input_centroids)

            # Deproject each tracked centroid to 3D space
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            results = {}
            for oid, (cx, cy) in tracked.items():
                depth = depth_frame.get_distance(cx, cy)
                if depth <= 0:
                    continue
                x, _, z = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth)
                results[oid] = (x, z)

            return results

        except Exception as e:
            print(f"Vision.process_frame error: {e}")
            return {}

    def _generate_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower1, upper1 = (0, 100, 90), (10, 255, 255)
        lower2, upper2 = (170, 100, 90), (179, 255, 255)
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        return cv2.bitwise_or(mask1, mask2)