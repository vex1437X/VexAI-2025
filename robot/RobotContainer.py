import pygame
from robot.subsystems.Drivetrain import Drivetrain
from robot.commands.TeleopCommand import Teleop
from robot.util.MotorController import MotorController
from robot.util.SerialHelper import SerialHelper


class RobotContainer:
    SERIAL_PORT = "/dev/ttyACM1"
    BAUD_RATE = 9600
    MAX_SPEED = 100

    def __init__(self):
        # Initialize serial communication and motor controller
        self.serialHelper = SerialHelper(self.SERIAL_PORT, self.BAUD_RATE)
        self.motor_controller = MotorController(self.serialHelper)

        # Initialize joystick
        self.joystick = self._init_joystick()

        # Initialize drivetrain and set the default teleop command
        self.drivetrain = Drivetrain(
            self.joystick, self.MAX_SPEED, "holonomic", self.motor_controller
        )
        self.teleop_command = Teleop(
            self.joystick, self.MAX_SPEED, "holonomic", self.motor_controller
        )
        self.drivetrain.set_command(self.teleop_command)

    def _init_joystick(self):
        # Initialize the joystick hardware
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick found.")
            return None

        # Initialize the first joystick and return it
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"Joystick initialized: {js.get_name()}")
        return js
