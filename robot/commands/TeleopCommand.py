from robot.util.Command import Command
from robot.util.MotorController import MotorController
from robot.util.Logger import Logger
import pygame


class Teleop(Command):
    def __init__(self, joystick, max_speed=50, drive_mode="tank", serialHelper=None):
        self.motor_controller = MotorController(serialHelper)
        self.joystick = joystick
        self.drive_mode = drive_mode
        self.max_speed = max_speed

        Logger.info(f"Teleop initialized with joystick: {joystick}, max_speed: {max_speed}, drive_mode: {drive_mode}")

    def execute(self):
        pygame.event.pump()
        if self.drive_mode == "tank":
            left_y = -self.apply_deadzone(self.joystick.get_axis(1))
            right_y = -self.apply_deadzone(self.joystick.get_axis(3))
            self.motor_controller.set_motor_speeds(
                left_y * self.max_speed, right_y * self.max_speed
            )
        elif self.drive_mode == "arcade":
            x = self.apply_deadzone(self.joystick.get_axis(0))
            y = self.apply_deadzone(self.joystick.get_axis(1))
            self.motor_controller.set_motor_speeds(
                y * self.max_speed, x * self.max_speed
            )

    def start(self):
        Logger.info("Teleop started.")

    def end(self, interrupted=False):
        self.motor_controller.set_motor_speeds(0, 0)
        Logger.info(f"Teleop ended. Interrupted: {interrupted}")

    def is_finished(self):
        return False

    def apply_deadzone(self, value, threshold=0.1):
        return value if abs(value) >= threshold else 0.0