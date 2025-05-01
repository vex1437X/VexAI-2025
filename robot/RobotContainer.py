import pygame

from robot.subsystems.Drivetrain import Drivetrain
from robot.util.SerialHelper import SerialHelper
from robot.subsystems.Vision import Vision
from robot.commands.TeleopCommand import Teleop
from robot.commands.TurnDrive import TurnDrive
from robot.commands.Search import Search


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
        self.serial_port = "/dev/tty.usbmodem143103"
        self.baud_rate = 9600
        self.max_speed = 50

        # Initialize SerialHelper
        # self.logger.info(
        #     f"Initializing SerialHelper with port {self.serial_port} and baud rate {self.baud_rate}..."
        # )
        self.serialHelper = SerialHelper(
            serial_port=self.serial_port, baud_rate=self.baud_rate
        )

        # self.logger.info("Initializing Vision subsystem...")
        self.vision = Vision(serialHelper=self.serialHelper)
        # self.logger.info("Vision subsystem initialized.")

        # Initialize subsystems
        # self.logger.info("Initializing Drivetrain subsystem...")
        self.drivetrain = Drivetrain(
            joystick=self.joystick,
            max_speed=self.max_speed,
            drive_mode="tank",
            serialHelper=self.serialHelper,
        )
        self.drivetrain.set_command(
            TurnDrive(vision=self.vision, serialHelper=self.serialHelper)
        )
        print("Drivetrain initialized.")

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
        pygame.time.delay(20)
        # ── PROCESS PYGAME EVENTS ──
        pygame.event.pump()

        self.drivetrain.tick()
        self.vision.tick()

        if not self.joystick:
            return

            # # make sure your previous_button_states is the right length
            # num_buttons = self.joystick.get_numbuttons()
            # if len(self.previous_button_states) != num_buttons:
            #     self.previous_button_states = [False] * num_buttons

            # # check every button for a “rising edge”
            # for i in range(num_buttons):
            #     current = self.joystick.get_button(i)
            #     if current and not self.previous_button_states[i]:
            #         # button i was just pressed
            #         if i == 2:  # Square
            #             print("Square button pressed.")
            #             cmd1 = TurnDrive(
            #                 max_speed=self.max_speed,
            #                 serialHelper=self.serialHelper,
            #                 vision=self.vision,
            #             )
            #             cmd2 = Search(
            #                 serialHelper=self.serialHelper,
            #                 vision=self.vision,
            #                 turn_speed=20,
            #             )

            #             cmd1.linkedCommand = cmd2
            #             cmd2.linkedCommand = cmd1
            #             print("Auto execute")
            #             self.drivetrain.set_command(cmd2)
            #         elif i == 3:  # Triangle
            #             print("Triangle button pressed.")
            #             cmd = Teleop(
            #                 joystick=self.joystick,
            #                 max_speed=self.max_speed,
            #                 drive_mode="tank",
            #                 serialHelper=self.serialHelper,
            #             )
            #             print("  → initializing Teleop")
            #             self.drivetrain.set_command(cmd)

            # update for next frame
            self.previous_button_states[i] = current
