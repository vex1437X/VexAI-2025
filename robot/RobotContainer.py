import pygame
from robot.subsystems.Drivetrain import Drivetrain
from robot.util.SerialHelper import SerialHelper
from robot.subsystems.Vision import Vision
from robot.commands.TeleopCommand import Teleop
from robot.commands.TurnDrive import TurnDrive
from robot.util.Logger import Logger

import json

class RobotContainer:
    """
    The RobotContainer class is responsible for defining the robot's subsystems,
    commands, and button mappings.
    """

    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.joystick = self._initialize_joystick()

        # Load configuration from JSON file
        config = self.load_config()
        self.serial_port = config["serial_port"]
        self.baud_rate = config["baud_rate"]
        self.max_speed = config["max_speed"]
        
        self.serialHelper = SerialHelper(serial_port=self.serial_port, baud_rate=self.baud_rate)

        # Initialize subsystems
        self.drivetrain = Drivetrain(
            joystick=self.joystick, max_speed=self.max_speed, drive_mode="tank", serialHelper=self.serialHelper
        )
        self.vision = Vision(serialHelper=self.serialHelper)

        # Track button states
        self.previous_button_states = [False] * self.joystick.get_numbuttons()

        # Configure button bindings
        self.configure_button_bindings()

        Logger.info("RobotContainer initialized.")

    def load_config():
        with open("config.json", "r") as f:
            return json.load(f)

    def _initialize_joystick(self):
        """Initialize the joystick, if available."""
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            Logger.info("Joystick initialized.")
            return joystick
        Logger.error("No joystick found.")
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
        """
        Periodically update the robot's subsystems.
        """
        self.drivetrain.tick()
        self.vision.tick()

        # Handle joystick button presses
        if self.joystick:
            current_button_states = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

            # Check for X button (button 0) press
            if current_button_states[0] and not self.previous_button_states[0]:
                # X button was just pressed
                self.drivetrain.set_command(
                    TurnDrive(serialHelper=self.serialHelper, vision=self.vision, max_speed=50)
                )

            # Check for O button (button 1) press
            elif current_button_states[1] and not self.previous_button_states[1]:
                # O button was just pressed
                self.drivetrain.set_command(
                    Teleop(joystick=self.joystick, max_speed=self.max_speed, drive_mode="tank", serialHelper=self.serialHelper)
                )

            # Update previous button states
            self.previous_button_states = current_button_states

        pygame.time.delay(20)