from robot.util.Command import Command
from robot.util.Constants import Instruction

from math import cos, sin
import pygame


class Teleop(Command):
    def __init__(self, joystick, max_speed=50, drive_mode="tank", serialHelper=None):
        self.serialHelper = serialHelper
        self.joystick = joystick
        self.drive_mode = drive_mode
        self.max_speed = max_speed

    def execute(self):
        pygame.event.pump()
        if self.drive_mode == "tank":
            left_y = -self.apply_deadzone(self.joystick.get_axis(1))
            right_y = -self.apply_deadzone(self.joystick.get_axis(3))
            left_speed = left_y * self.max_speed
            right_speed = right_y * self.max_speed
            command = self.serialHelper.encode_instruction(
                Instruction.DRIVE_SET,
                [left_speed, right_speed, left_speed, right_speed],
            )
            self.serialHelper.send_command(command)
        elif self.drive_mode == "arcade":
            x = self.apply_deadzone(self.joystick.get_axis(0))
            y = self.apply_deadzone(self.joystick.get_axis(1))
            left_speed = y * self.max_speed
            right_speed = x * self.max_speed
            command = self.serialHelper.encode_instruction(
                Instruction.DRIVE_SET,
                [left_speed, right_speed, left_speed, right_speed],
            )
            self.serialHelper.send_command(command)
        elif self.drive_mode == "holonomic":
            # Raw joystick values
            x = self.apply_deadzone(self.joystick.get_axis(0))
            y = self.apply_deadzone(self.joystick.get_axis(1))
            rotation = self.apply_deadzone(self.joystick.get_axis(2))

            # Conver from global to robot-centric coordinates
            heading = self.serialHelper.value(Instruction.GYRO)
            xl = x * cos(heading) - y * sin(heading)
            yl = x * sin(heading) + y * cos(heading)

            normalize = max(abs(xl), abs(yl), abs(rotation))
            if normalize > 1:
                xl /= normalize
                yl /= normalize
                rotation /= normalize
            left_speed = (yl + xl + rotation) * self.max_speed
            right_speed = (yl - xl - rotation) * self.max_speed
            command = self.serialHelper.encode_instruction(
                Instruction.DRIVE_SET,
                [left_speed, right_speed, left_speed, right_speed],
            )
            self.serialHelper.send_command(command)
        print(f"Driving go brrr {left_y} + f{right_y}")

    def start(self):
        pass

    def end(self, interrupted=False):
        command = self.serialHelper.encode_instruction(
            Instruction.DRIVE_SET, [0, 0, 0, 0]
        )
        self.serialHelper.send_command(command)

    def is_finished(self):
        return False

    def apply_deadzone(self, value, threshold=0.1):
        return value if abs(value) >= threshold else 0.0
