from robot.util.Command import Command
from robot.util.Constants import Instruction
from robot.util.MotorController import MotorController


class Search(Command):
    def __init__(
        self, motor_controller: MotorController = None, vision=None, turn_speed=35
    ):
        self.motor_controller = motor_controller
        self.turn_speed = turn_speed
        self.vision = vision

    def start(self):
        print("Search command started.")

    def execute(self):
        # turn left until an object is detected, then end the command
        self.motor_controller.set_speed(
            Instruction.DRIVE_SET,
            [-self.turn_speed, self.turn_speed, -self.turn_speed, self.turn_speed],
        )

    def is_finished(self):
        # check if an object is detected
        detections = self.vision.get_detections()
        if detections is None:
            return False
        return len(detections) > 0

    def end(self, interrupted=False):
        # stop the motors
        self.motor_controller.set_speed(Instruction.DRIVE_SET, [0, 0, 0, 0])
        print("Search command ended.")
