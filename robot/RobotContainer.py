import pygame
import numpy as np

from robot.subsystems.Drivetrain import Drivetrain
from robot.util.SerialHelper import SerialHelper
from robot.subsystems.Vision import Vision
from robot.commands.TeleopCommand import Teleop
from robot.commands.TurnDrive import TurnDrive
from robot.commands.Search import Search
from robot.util.Constants import DataTag
from robot.PoseEstimator import PoseEstimator


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
        self.serial_port = "/dev/ttyACM1"
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
            TurnDrive(
                max_speed=self.max_speed,
                serialHelper=self.serialHelper,
                vision=self.vision,
            )
        )
        print("Drivetrain initialized.")

        # Track button states
        # self.logger.info("Initializing button states...")
        self.previous_button_states = [False] * self.joystick.get_numbuttons()

        # Configure button bindings
        # self.logger.info("Configuring button bindings...")
        self.configure_button_bindings()

        # self.logger.info("RobotContainer initialization complete.")

        self.pose_estimator = PoseEstimator(
            wheel_radius_m=(2.75 * 0.0254) / 2,  # wheel radius in meters
            base_half_length_m=0.2565 / 2,  # half‑chassis length  # <<< TODO
            base_half_width_m=0.2565 / 2,  # half‑chassis width   # <<< TODO
            gps_offsets_m=[(-0.019, 0.068), (-0.019, -0.068)],  # GPS0, GPS1  # <<< TODO
            # using +x and +y as forward and left respectively
        )

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

        self.serialHelper.periodic()

        # get the latest sensor data

        wheel_v = np.array(
            [
                self.serialHelper.value(DataTag.FL),
                self.serialHelper.value(DataTag.FR),
                self.serialHelper.value(DataTag.BL),
                self.serialHelper.value(DataTag.BR),
            ]
        )

        if (
            self.serialHelper.was_updated(DataTag.GPS0_X)
            and self.serialHelper.was_updated(DataTag.GPS0_Y)
            and self.serialHelper.was_updated(DataTag.GPS0_H)
        ):
            gps0_data = np.array(
                [
                    self.serialHelper.value(DataTag.GPS0_X),
                    self.serialHelper.value(DataTag.GPS0_Y),
                    self.serialHelper.value(DataTag.GPS0_H),
                ]
            )
        else:
            gps0_data = None

        if (
            self.serialHelper.was_updated(DataTag.GPS1_X)
            and self.serialHelper.was_updated(DataTag.GPS1_Y)
            and self.serialHelper.was_updated(DataTag.GPS1_H)
        ):
            gps1_data = np.array(
                [
                    self.serialHelper.value(DataTag.GPS1_X),
                    self.serialHelper.value(DataTag.GPS1_Y),
                    self.serialHelper.value(DataTag.GPS1_H),
                ]
            )
        else:
            gps1_data = None

        self.pose_estimator.update_pose(
            wheel_v=wheel_v,
            gyro=self.serialHelper.value(DataTag.GYRO),
            gps1_reading=gps0_data,
            gps2_reading=gps1_data["GPS1"],
        )

        self.drivetrain.tick()
        self.vision.tick()

        if not self.joystick:
            return

        # make sure your previous_button_states is the right length
        num_buttons = self.joystick.get_numbuttons()
        if len(self.previous_button_states) != num_buttons:
            self.previous_button_states = [False] * num_buttons

        turnDriveCMD = TurnDrive(
            max_speed=self.max_speed,
            serialHelper=self.serialHelper,
            vision=self.vision,
        )
        searchCMD = Search(
            serialHelper=self.serialHelper,
            vision=self.vision,
            turn_speed=20,
        )

        # turnDriveCMD.linkedCommand = searchCMD
        # searchCMD.linkedCommand = turnDriveCMD

        teleCMD = Teleop(
            joystick=self.joystick,
            max_speed=self.max_speed,
            drive_mode="tank",
            serialHelper=self.serialHelper,
        )

        # check every button for a “rising edge”
        # for i in range(num_buttons):
        #     current = self.joystick.get_button(i)
        #     if current and not self.previous_button_states[i]:
        #         # button i was just pressed
        #         if i == 2:  # Square
        #             print("Square button pressed.")
        #             print("Auto execute")
        #             self.drivetrain.set_command(searchCMD)
        #         elif i == 3:  # Triangle
        #             print("Triangle button pressed.")
        #             print("  → initializing Teleop")
        #             self.drivetrain.set_command(teleCMD)

        # update for next frame
        # self.previous_button_states[i] = current
        # Track the time since the last detection
        if not hasattr(self, "_last_detection_time"):
            self._last_detection_time = pygame.time.get_ticks()

        if len(self.vision.process_frame()) > 0:
            # Reset the timer if there are detections
            self._last_detection_time = pygame.time.get_ticks()
        else:
            # Check if 0.5 seconds have passed without detections
            current_time = pygame.time.get_ticks()
            if current_time - self._last_detection_time >= 1000:
                if not isinstance(self.drivetrain.command, Search):
                    self.drivetrain.set_command(searchCMD)
                    print("Search command set. vision")

        self.vision.frame_done = False
