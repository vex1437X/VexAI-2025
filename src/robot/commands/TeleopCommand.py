from src.core.commands.Command import Command
from src.core.logging.logger_config import get_logger

import pygame

logger = get_logger(__name__)


class TeleopCommand(Command):
    def __init__(self, drivetrain=None, drive_mode="tank", max_speed=50, joystick=None):
        super().__init__()
        self.drivetrain = drivetrain
        self.drive_mode = drive_mode
        self.max_speed = max_speed
        self.joystick = joystick

    def execute(self):
        pygame.event.pump()
        if self.drive_mode == "tank":
            left_y = -self.apply_deadzone(self.joystick.get_axis(1))
            right_y = -self.apply_deadzone(self.joystick.get_axis(3))
            left_speed = left_y * self.max_speed
            right_speed = right_y * self.max_speed
            self.drivetrain.set_speed(left_speed, right_speed)
            # logger.info(f"Driving go brrr {left_y} + f{right_y}")

    def initialize(self):
        pass

    def end(self, interrupted=False):
        self.drivetrain.stop()

    def is_finished(self):
        return False

    def apply_deadzone(self, value, threshold=0.1):
        return value if abs(value) >= threshold else 0.0
