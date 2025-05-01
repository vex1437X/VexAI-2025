from robot.util.Command import Command
from robot.util.Constants import Instruction


class Search(Command):
    def __init__(self, serialHelper=None, vision=None, turn_speed=20):
        self.serialHelper = serialHelper
        self.turn_speed = turn_speed
        self.vision = vision

    def start(self):
        print("Search command started.")

    def execute(self):
        # turn left until an object is detected, then end the command
        command = self.serialHelper.encode_instruction(
            Instruction.DRIVE_SET,
            [-self.turn_speed, self.turn_speed, -self.turn_speed, self.turn_speed],
        )
        self.serialHelper.send_command(command)

    def is_finished(self):
        # check if an object is detected
        raw = self.vision.process_frame()
        print(f"Search: {raw}")
        if len(raw) == 0:
            return False
        return True

    def end(self, interrupted=False):
        # stop the motors
        command = self.serialHelper.encode_instruction(
            Instruction.DRIVE_SET, [0, 0, 0, 0]
        )
        self.serialHelper.send_command(command)
        # print("Search command ended.")
