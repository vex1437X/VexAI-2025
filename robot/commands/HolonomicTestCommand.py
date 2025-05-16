from robot.util.Command import Command
from robot.util.MotorController import MotorController
import time
from robot.util.Constants import Instruction


class HolonomicTestCommand(Command):
    """
    A command to test the holonomic drive system of a robot.
    """

    def __init__(self, motor_controller: MotorController):
        self.mc = motor_controller

    def execute(self):
        """
        Execute the command to test the holonomic drive system.
        """
        # Example logic for testing the holonomic drive
        self.mc.drive_holomonic(1, 0, 0, 50.0)
        time.sleep(1)
        self.mc.set_speed(Instruction.DRIVE_SET, [0, 0, 0, 0])
        time.sleep(2)

        self.mc.drive_holomonic(0, 1, 0, 50.0)
        time.sleep(1)
        self.mc.set_speed(Instruction.DRIVE_SET, [0, 0, 0, 0])
        time.sleep(2)

        self.mc.drive_holomonic(0, 0, 1, 50.0)
        time.sleep(1)
        self.mc.set_speed(Instruction.DRIVE_SET, [0, 0, 0, 0])
        time.sleep(2)
