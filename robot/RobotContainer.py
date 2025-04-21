import pygame

from robot.subsystems.Drivetrain import Drivetrain
from robot.util.SerialHelper import SerialHelper
from robot.subsystems.Vision import Vision
from robot.commands.TeleopCommand import Teleop
from robot.commands.TurnDrive import TurnDrive
import logging


class RobotContainer:
    """
    The RobotContainer class is responsible for defining the robot's subsystems,
    commands, and button mappings.
    """

    def __init__(self):
        # logging.basicConfig(level=logging.INFO)
        # self.logger = logging.getLogger(__name__)

        # self.logger.info("Initializing RobotContainer...")

        # Initialize pygame
        # self.logger.info("Initializing pygame...")
        pygame.init()
        pygame.joystick.init()

        # Initialize joystick
        print("Initializing joystick...")
        self.joystick = self._initialize_joystick()

        # Hardcoded configuration values
        # self.logger.info("Setting configuration values...")
        self.serial_port = "/dev/tty.usbmodem143303"
        self.baud_rate = 9600
        self.max_speed = 50

        # Initialize SerialHelper
        # self.logger.info(
        #     f"Initializing SerialHelper with port {self.serial_port} and baud rate {self.baud_rate}..."
        # )
        self.serialHelper = SerialHelper(
            serial_port=self.serial_port, baud_rate=self.baud_rate
        )

        # Initialize subsystems
        # self.logger.info("Initializing Drivetrain subsystem...")
        self.drivetrain = Drivetrain(
            joystick=self.joystick,
            max_speed=self.max_speed,
            drive_mode="tank",
            serialHelper=self.serialHelper,
        )
        print("Drivetrain initialized.")

        # self.logger.info("Initializing Vision subsystem...")
        self.vision = Vision(serialHelper=self.serialHelper)
        # self.logger.info("Vision subsystem initialized.")

        # Track button states
        # self.logger.info("Initializing button states...")
        self.previous_button_states = [False] * self.joystick.get_numbuttons()

        # Configure button bindings
        # self.logger.info("Configuring button bindings...")
        self.configure_button_bindings()

        # self.logger.info("RobotContainer initialization complete.")

    def _initialize_joystick(self):
        """Initialize the joystick, if available."""
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Joystick initialized: {joystick.get_name()}")
            return joystick
        print("No joystick found.")
        return None

    def configure_button_bindings(self):
        """
        Define the button-to-command mappings here.
        """
        pass

    def get_autonomous_command(self):
        """
        Returns the command to run in autonomous mode.
        """
        return None

    def periodic(self):
        # self.logger.info("Periodic loop started.")
        self.drivetrain.tick()
        # self.logger.info("Drivetrain tick completed.")
        self.vision.tick()
        # self.logger.info("Vision tick completed.")

        # Handle joystick button presses
        if self.joystick:
            # self.logger.info("Processing joystick input.")
            current_button_states = [
                self.joystick.get_button(i)
                for i in range(self.joystick.get_numbuttons())
            ]

            # Check for X button (button 0) press
            if current_button_states[0] and not self.previous_button_states[0]:
                # self.logger.info("X button pressed.")
                self.drivetrain.set_command(
                    TurnDrive(
                        serialHelper=self.serialHelper, vision=self.vision, max_speed=50
                    )
                )

            # Check for O button (button 1) press
            elif current_button_states[1] and not self.previous_button_states[1]:
                # self.logger.info("O button pressed.")
                self.drivetrain.set_command(
                    Teleop(
                        joystick=self.joystick,
                        max_speed=self.max_speed,
                        drive_mode="tank",
                        serialHelper=self.serialHelper,
                    )
                )

            # Update previous button states
            self.previous_button_states = current_button_states

        pygame.time.delay(20)
        # self.logger.info("Periodic loop ended.")
