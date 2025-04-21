from robot.util.Command import Command
import math
from simple_pid import PID
from robot.util.Constants import Instruction


class TurnDrive(Command):
    """
    Command to turn and drive the robot toward a target detected by the Vision subsystem.
    """

    def __init__(self, max_speed=50, serialHelper=None, vision=None):
        """
        Initialize the TurnDrive command.

        Args:
            max_speed (float): Maximum speed for turning and driving.
            serialHelper (SerialHelper): Serial helper for sending commands to the robot.
            vision (Vision): Vision subsystem for detecting the target.
        """
        self.serialHelper = serialHelper
        self.vision = vision
        self.max_speed = max_speed

        # PID controllers for turning and driving
        self.turn_pid = PID(1, 0, 0, setpoint=0)
        self.turn_pid.output_limits = (-self.max_speed, self.max_speed)

        self.drive_pid = PID(200, 0.1, 0.05, setpoint=0)
        self.drive_pid.output_limits = (-self.max_speed, self.max_speed)

        # Flags to track completion of turning and driving
        self.turn_finished = False
        self.drive_finished = False
        print("TurnDrive initialized.")

    def execute(self):
        """
        Execute the TurnDrive command. This method is called periodically.
        """
        print("Executing TurnDrive command...")
        offsets = self.vision.process_frame()
        if offsets is None or len(offsets) < 2:
            print("No valid offsets detected. Stopping motors.")
            self._stop_motors()
            self.turn_finished = True
            self.drive_finished = True
            return

        x_offset, z_offset = offsets
        print(f"Offsets detected: x_offset={x_offset}, z_offset={z_offset}")
        self._handle_turning(x_offset, z_offset)
        self._handle_driving(z_offset)

        if self.turn_finished and self.drive_finished:
            print("Turn and drive completed. Stopping motors.")
            self._stop_motors()
            self.end(False)

    def _handle_turning(self, x_offset, z_offset):
        if self.turn_finished:
            print("Turning already finished.")
            return

        delta_angle = math.degrees(math.atan2(x_offset, z_offset))
        print(f"Delta angle for turning: {delta_angle}")

        if abs(delta_angle) < 1:
            print("Delta angle within threshold. Turning finished.")
            self.turn_finished = True
        else:
            turn_speed = self.turn_pid(delta_angle)
            print(f"Calculated turn speed: {turn_speed}")
            self._set_motor_speeds(-turn_speed, turn_speed)

    def _handle_driving(self, z_offset):
        if self.drive_finished:
            print("Driving already finished.")
            return
        if not self.turn_finished:
            print("Turning not finished. Skipping driving.")
            return

        drive_speed = self.drive_pid(z_offset)
        print(f"Calculated drive speed: {drive_speed}")

        if abs(z_offset) < 0.01:
            print("Z offset within threshold. Driving finished.")
            self.drive_finished = True
        else:
            self._set_motor_speeds(-drive_speed, -drive_speed)

    def _stop_motors(self):
        self._set_motor_speeds(0, 0)

    def _set_motor_speeds(self, left_speed, right_speed):
        command = self.serialHelper.encode_instruction(
            Instruction.DRIVE_SET, [left_speed, right_speed, left_speed, right_speed]
        )
        self.serialHelper.send_command(command)

    def start(self):
        self.turn_finished = False
        self.drive_finished = False

    def end(self, interrupted=False):
        self._stop_motors()

    def is_finished(self):
        return self.turn_finished and self.drive_finished
