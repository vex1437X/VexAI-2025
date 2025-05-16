import pygame
import numpy as np
import time
import threading

from robot.subsystems.Drivetrain import Drivetrain
from robot.util.SerialHelper import SerialHelper
from robot.subsystems.Vision import Vision
from robot.commands.TeleopCommand import Teleop
from robot.commands.TurnDrive import TurnDrive
from robot.commands.Search import Search
from robot.util.Constants import DataTag
from robot.PoseEstimator import PoseEstimator
from robot.util.Controller import Controller
from robot.util.MotorController import MotorController
from robot.commands.HolonomicTestCommand import HolonomicTestCommand


class RobotContainer:
    # Configuration constants for the robot
    SERIAL_PORT = "/dev/ttyACM1"
    BAUD_RATE = 115200
    MAX_SPEED = 100
    DETECTION_TIMEOUT = 1  # Timeout for vision detection in milliseconds

    def __init__(self):
        # Initialize serial communication and motor controller
        self.serialHelper = SerialHelper(self.SERIAL_PORT, self.BAUD_RATE)

        self._serial_thread_stop = (
            False  # flag to signal thread (if needed to stop gracefully)
        )
        serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        serial_thread.start()

        self.motor_controller = MotorController(self.serialHelper)

        # Initialize joystick and button bindings
        self.joystick = self._init_joystick()
        if self.joystick:
            self._init_button_bindings()

        # Initialize subsystems and set the default drivetrain command
        self._init_subsystems()
        self.drivetrain.set_command(self.search_command)

        # Initialize button states and detection timing
        btn_count = self.joystick.get_numbuttons() if self.joystick else 0
        self.previous_button_states = [False] * btn_count
        self._last_detection_time = pygame.time.get_ticks()

        # Initialize pose estimator for robot localization
        self.pose_estimator = PoseEstimator(
            wheel_radius_m=(2.75 * 0.0254) / 2,  # Convert wheel radius to meters
            base_half_length_m=0.2565 / 2,  # Half the robot's base length
            base_half_width_m=0.2565 / 2,  # Half the robot's base width
            gps_offsets_m=[(-0.019, 0.068), (-0.019, -0.068)],  # GPS sensor offsets
        )

    def _init_subsystems(self):
        # Initialize vision and drivetrain subsystems
        self.vision = Vision()

        self._vision_thread_stop = False
        vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
        vision_thread.start()

        self.drivetrain = Drivetrain(
            self.joystick,
            self.MAX_SPEED,
            "holonomic",
            self.motor_controller,
            self.vision,
        )

        # Initialize commands for different robot behaviors
        self.turn_command = TurnDrive(
            max_speed=self.MAX_SPEED,
            motor_controller=self.motor_controller,
            vision=self.vision,
        )
        self.search_command = Search(self.motor_controller, self.vision, turn_speed=35)
        self.teleop_command = Teleop(
            self.joystick, self.MAX_SPEED, "holonomic", self.motor_controller
        )
        self.testCMD = HolonomicTestCommand(self.motor_controller)

        # Set the default command for the drivetrain
        self.drivetrain.set_command(self.search_command)

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

    def _init_button_bindings(self):
        # Bind joystick buttons to specific commands
        self.controller = Controller(self.joystick)
        self.controller.on_button_down(
            0, self._schedule_teleop
        )  # Button 0: Teleop mode
        self.controller.on_button_down(
            1, self._schedule_search
        )  # Button 1: Search mode
        self.controller.on_button_down(2, self._schedule_turn)  # Button 2: Turn mode

    def _schedule_teleop(self):
        # Switch to teleop command
        self.drivetrain.set_command(self.teleop_command)

    def _schedule_search(self):
        # Switch to search command
        self.drivetrain.set_command(self.search_command)

    def _schedule_turn(self):
        # Switch to turn command
        self.drivetrain.set_command(self.turn_command)

    def _get_gps_data(self, tag_x, tag_y, tag_h):
        # Retrieve GPS data if all required tags have been updated
        if all(self.serialHelper.was_updated(t) for t in (tag_x, tag_y, tag_h)):
            return np.array(
                [
                    self.serialHelper.value(tag_x),
                    self.serialHelper.value(tag_y),
                    self.serialHelper.value(tag_h),
                ]
            )
        return None

    def _serial_loop(self):
        while not self._serial_thread_stop:
            self.serialHelper.periodic()  # read and update state (non-blocking ~10ms max)
            # small sleep to avoid busy-waiting, roughly match loop timing
            time.sleep(0.005)  # 5ms sleep for ~200Hz polling, adjust as needed

    def _vision_loop(self):
        while not self._vision_thread_stop:
            # Get the latest frame and detection results
            tracked_coords = (
                self.vision.process_frame()
            )  # returns dict of tracked object coords:contentReference[oaicite:15]{index=15}
            # Optionally, you could store tracked_coords or set a flag here.
            # In this design, vision.process_frame already updates internal state (current_frame, current_detections, tracker.objects).
            # Small sleep or yield could be added to limit frame rate if needed:
            # time.sleep(0.001)

    def periodic(self):
        # Perform periodic updates for the robot's operation

        # Delay for a fixed time and process joystick events
        pygame.time.delay(20)
        pygame.event.pump()
        # for evt in pygame.event.get():
        #    self.controller.handle_event(evt)

        # Retrieve wheel velocities from sensors
        wheel_v = np.array(
            [
                self.serialHelper.value(DataTag.FL),
                self.serialHelper.value(DataTag.FR),
                self.serialHelper.value(DataTag.BL),
                self.serialHelper.value(DataTag.BR),
            ]
        )

        # Retrieve GPS data from both sensors
        gps0 = self._get_gps_data(DataTag.GPS0_X, DataTag.GPS0_Y, DataTag.GPS0_H)
        gps1 = self._get_gps_data(DataTag.GPS1_X, DataTag.GPS1_Y, DataTag.GPS1_H)

        # Update the robot's pose estimation
        self.pose_estimator.update_pose(
            wheel_v=wheel_v,
            gyro=self.serialHelper.value(DataTag.GYRO),
            gps1_reading=gps0,
            gps2_reading=gps1,
        )

        # Update subsystems
        self.drivetrain.tick()
        self.vision.tick()

        now = time.time()
        if len(self.vision.tracker.objects) > 0:
            # At least one object is currently being tracked (detection ongoing)
            self._last_detection_time = now
        elif now - self._last_detection_time > self.DETECTION_TIMEOUT:
            # No detections for a timeout period – switch to search mode if not already searching
            if not isinstance(self.drivetrain.command, Search):
                self.drivetrain.set_command(self.search_command)
                print("Switching to Search command (vision timeout).")

        # goofy stuff
        if self.drivetrain.command == None:
            self._schedule_search()
