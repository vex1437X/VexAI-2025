import pyrealsense2 as rs
import numpy as np
import cv2
import sys
from robot.util.Subsystem import Subsystem


class Vision(Subsystem):
    """
    Vision subsystem for processing frames and detecting rings using RealSense.
    """

    def __init__(self, serialHelper=None):
        super().__init__(serialHelper)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)

        self._configure_pipeline()

        self.latest_offsets = None

        print("RealSense camera initialized successfully.")

    def _configure_pipeline(self):
        """Configure the RealSense pipeline."""
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        try:
            self.pipeline.start(self.config)
        except RuntimeError as e:
            sys.exit(1)

    def process_frame(self):
        """
        Process a frame to detect rings.

        Returns:
            tuple: (x_offset, z_offset) or (None, None) if no ring is detected.
        """
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        mask = self._mask_generator(color_frame)
        return self._detect_object(mask, depth_frame)

    def _mask_generator(self, color_frame):
        """Generate a binary mask for detecting rings."""
        color_img = np.asanyarray(color_frame.get_data())
        lab = cv2.cvtColor(color_img, cv2.COLOR_BGR2Lab)
        _, red_mask = cv2.threshold(lab[:, :, 1], 150, 255, cv2.THRESH_BINARY)

        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        _, sat_mask = cv2.threshold(hsv[:, :, 1], 50, 255, cv2.THRESH_BINARY)

        mask = cv2.bitwise_and(red_mask, sat_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def _detect_object(self, mask, depth_frame):
        """Detect the largest object in the mask and calculate its offsets."""
        n_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        if n_labels <= 1:
            return None, None

        areas = stats[1:, cv2.CC_STAT_AREA]
        best_lab = 1 + np.argmax(areas)
        cx, cy = map(int, centroids[best_lab])

        dist = depth_frame.get_distance(cx, cy)
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        x_off, _, z_off = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)

        return x_off, z_off

    def get_latest_offsets(self):
        """
        Get the latest offsets by processing a frame.

        Returns:
            tuple: (x_offset, z_offset) or (None, None) if no ring is detected.
        """
        self.latest_offsets = self.process_frame()
        return self.latest_offsets

    def stop(self):
        """Stop the RealSense pipeline."""
        self.pipeline.stop()
