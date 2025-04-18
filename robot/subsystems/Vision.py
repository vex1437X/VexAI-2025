import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import threading
from robot.util.Subsystem import Subsystem
from robot.util.Logger import Logger


class Vision(Subsystem):
    """
    Vision subsystem for processing frames and detecting objects using RealSense.
    """

    def __init__(self, serialHelper=None):
        super().__init__(serialHelper)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)

        self._configure_pipeline()
        Logger.info("Vision subsystem initialized.")

        self.latest_offsets = None
        self.running = True
        self.thread = threading.Thread(target=self._process_frames, daemon=True)
        self.thread.start()

    def _configure_pipeline(self):
        """Configure the RealSense pipeline."""
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        try:
            self.pipeline.start(self.config)
            Logger.info("RealSense pipeline started.")
        except RuntimeError as e:
            Logger.error(f"Could not start RealSense pipeline: {e}")
            sys.exit(1)

    def _process_frames(self):
        while self.running:
            self.latest_offsets = self.process_frame(lambda frame: None)

    def process_frame(self, mask_generator):
        """
        Process a frame to detect objects based on a mask generator.

        Args:
            mask_generator (callable): A function that generates a binary mask for object detection.

        Returns:
            tuple: (x_offset, z_offset) or (None, None) if no object is detected.
        """
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        mask = mask_generator(color_frame)
        return self._detect_object(mask, depth_frame)

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
        return self.latest_offsets

    def stop(self):
        """Stop the RealSense pipeline."""
        self.running = False
        self.thread.join()
        self.pipeline.stop()
        Logger.info("RealSense pipeline stopped.")
