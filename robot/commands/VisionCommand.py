from robot.util.Command import Command


class VisionCommand(Command):
    """
    Base class for vision-based commands.
    """

    def __init__(self, vision):
        self.vision = vision
        self.x_off = None
        self.z_off = None

    def mask_generator(self, color_frame):
        """
        Generate a binary mask for object detection.

        This method should be overridden by subclasses to define specific detection criteria.
        """
        raise NotImplementedError("Subclasses must implement mask_generator")

    def execute(self):
        self.x_off, self.z_off = self.vision.process_frame(self.mask_generator)
        if self.x_off is None or self.z_off is None:
            print("No object detected")
        else:
            print(f"Offset (m): x={self.x_off:.3f}, z={self.z_off:.3f}")

    def start(self):
        pass

    def end(self, interrupted=False):
        pass

    def is_finished(self):
        return False
