from robot.util.Constants import Instruction

class MotorController:
    def __init__(self, serialHelper):
        self.serialHelper = serialHelper

    def set_motor_speeds(self, left_speed, right_speed):
        command = self.serialHelper.encode_instruction(
            Instruction.DRIVE_SET, [left_speed, right_speed, left_speed, right_speed]
        )
        self.serialHelper.send_command(command)