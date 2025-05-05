from robot.util.Command import Command
import math
from simple_pid import PID
from robot.util.Constants import Instruction
import math
import time


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
        self.turn_pid = PID(0.5, 0, 0, setpoint=0)
        self.turn_pid.output_limits = (-self.max_speed, self.max_speed)

        self.drive_pid = PID(100, 0.1, 0.05, setpoint=0)
        self.drive_pid.output_limits = (-self.max_speed, self.max_speed)

        # Flags to track completion of turning and driving
        self.turn_finished = False
        self.drive_finished = False
        self.tof_finished = False
        # print("TurnDrive initialized.")

    def execute(self):
        """
        Execute the TurnDrive command. This method is called periodically.
        """

        offsets = None

        if hasattr(self, "_drive_start_time"):
            print(f"Drive start time: {time.time() - self._drive_start_time}")
            if time.time() - self._drive_start_time >= 0.5:
                self.drive_finished = True
        if self.tof_finished == False:
            try:
                raw = self.vision.process_frame()

                # normalize into a flat list of (x,z) tuples
                if raw is None:
                    candidates = []
                elif isinstance(raw, dict):
                    candidates = list(raw.values())
                else:
                    candidates = raw

                # find the nearest ring
                best_offset = None
                best_dist = float("inf")
                for offs in candidates:
                    # offs might be None or malformed
                    if not offs or len(offs) < 2:
                        continue
                    x_off, z_off = offs
                    d = math.hypot(x_off, z_off)
                    if d < best_dist:
                        best_dist = d
                        best_offset = (x_off, z_off)

                offsets = best_offset
                # print(f"Best offsets detected: {offsets}")

            except Exception as e:
                # if anything goes wrong, treat as “no detection”
                # print(f"Vision error in execute(): {e}")
                offsets = None

        # ensure we have a pair
        if not offsets:
            x_offset, z_offset = None, None
        else:
            x_offset, z_offset = offsets

        # only act if we actually saw something
        if x_offset is not None and z_offset is not None:
            self._handle_turning(x_offset, z_offset)
            self._handle_driving(z_offset)

        # check for completion
        if self.turn_finished and self.drive_finished:
            self._stop_motors()
            self.end(False)

    def _handle_turning(self, x_offset, z_offset):
        if self.turn_finished:
            # print("Turning already finished.")
            return

        delta_angle = math.degrees(math.atan2(x_offset, z_offset))
        # print(f"Delta angle for turning: {delta_angle}")

        if abs(delta_angle) < 1:
            # print("Delta angle within threshold. Turning finished.")
            self.turn_finished = True
        else:
            turn_speed = self.turn_pid(delta_angle)
            # print(f"Calculated turn speed: {turn_speed}")
            self._set_motor_speeds(-turn_speed, turn_speed)

    def _handle_driving(self, z_offset):
        if self.drive_finished:
            # print("Driving already finished.")
            return
        if not self.turn_finished:
            # print("Turning not finished. Skipping driving.")
            return

        drive_speed = self.drive_pid(z_offset)
        # print(f"Calculated drive speed: {drive_speed}")

        # drive more when vision lost (TODO: stop when color sensor senses ring)
        print(f"Z offset: {abs(z_offset)}")
        if abs(z_offset) < 0.5:
            # print("Z offset within threshold. Driving finished.")
            if not hasattr(self, "_drive_start_time"):
                self._drive_start_time = time.time()
                self.tof_finished = True
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
        self.tof_finished = False

    def end(self, interrupted=False):
        self._stop_motors()
        self.turn_pid.reset()
        self.drive_pid.reset()
        print("TurnDrive command ended.")
        if hasattr(self, "_drive_start_time"):
            del self._drive_start_time

    def is_finished(self):
        # print(f"Finished? {self.turn_finished} {self.drive_finished} ")
        return self.turn_finished and self.drive_finished
