from robot.commands.VisionCommand import VisionCommand
import cv2
import numpy as np


class DetectRing(VisionCommand):
    """
    Command to detect rings using the Vision subsystem.
    """

    def __init__(self, vision=None):
        self.vision = vision
        self.x_off = None
        self.z_off = None

    def mask_generator(self, color_frame):
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

    def execute(self):
        pass

    def start(self):
        pass

    def end(self, interrupted=False):
        pass

    def is_finished(self):
        return False
