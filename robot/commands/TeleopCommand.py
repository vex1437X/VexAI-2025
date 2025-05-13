from robot.util.Command import Command
from robot.util.Constants import Instruction
from robot.util.MotorController import MotorController

from math import cos, sin
import pygame


class Teleop(Command):
    def __init__(
        self,
        joystick,
        max_speed=100,
        drive_mode="tank",
        motor_controller: MotorController = None,
    ):
        self.motor_controller = motor_controller
        self.joystick = joystick
        self.drive_mode = drive_mode
        self.max_speed = max_speed

    def execute(self):
        pygame.event.pump()
        if self.drive_mode == "tank":
            left_y = self.apply_deadzone(self.joystick.get_axis(1))
            right_y = self.apply_deadzone(self.joystick.get_axis(4))
            left_speed = left_y * self.max_speed
            right_speed = right_y * self.max_speed
            self.motor_controller.set_speed(
                Instruction.DRIVE_SET,
                [left_speed, right_speed, left_speed, right_speed],
            )
        elif self.drive_mode == "arcade":
            x = self.apply_deadzone(self.joystick.get_axis(1))
            y = self.apply_deadzone(self.joystick.get_axis(0))
            left_speed = y * self.max_speed
            right_speed = x * self.max_speed
            self.motor_controller.set_speed(
                Instruction.DRIVE_SET,
                [left_speed, right_speed, left_speed, right_speed],
            )
        elif self.drive_mode == "holonomic":
            # Raw joystick values
            x = self.apply_deadzone(self.joystick.get_axis(0))
            y = self.apply_deadzone(self.joystick.get_axis(1))
            rotation = self.apply_deadzone(self.joystick.get_axis(3))

            # Convert joystick x and y into global x and y based on heading
            heading = (
                self.motor_controller.get_heading()
            )  # Assuming heading is in radians
            global_x = x * cos(heading) - y * sin(heading)
            global_y = x * sin(heading) + y * cos(heading)

            # Update x and y with global values
            x = global_x
            y = global_y

            # Calculate motor speeds for holonomic drive
            fl = (y + x + rotation) * self.max_speed  # Front Left
            fr = (y - x - rotation) * self.max_speed  # Front Right
            bl = (y - x + rotation) * self.max_speed  # Back Left
            br = (y + x - rotation) * self.max_speed  # Back Right

            # Normalize speeds to ensure they are within the max_speed range
            max_magnitude = max(abs(fl), abs(fr), abs(bl), abs(br), self.max_speed)
            if max_magnitude > self.max_speed:
                fl = fl / max_magnitude * self.max_speed
                fr = fr / max_magnitude * self.max_speed
                bl = bl / max_magnitude * self.max_speed
                br = br / max_magnitude * self.max_speed

            # Set motor speeds
            self.motor_controller.set_speed(
                Instruction.DRIVE_SET,
                [fl, fr, bl, br],
            )
            print(str(fl) + ", " + str(fr))

    def start(self):
        pass

    def end(self, interrupted=False):
        self.motor_controller.set_speed(Instruction.DRIVE_SET, [0, 0, 0, 0])

    def is_finished(self):
        return False

    def apply_deadzone(self, value, threshold=0.1):
        return value if abs(value) >= threshold else 0.0
