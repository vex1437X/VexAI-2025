from robot.util.Command import Command
from robot.util.Logger import Logger
from robot.util.MotorController import MotorController
import math
from simple_pid import PID


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
        self.motor_controller = MotorController(serialHelper)
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

        Logger.info("TurnDrive initialized")

    def execute(self):
        """
        Execute the TurnDrive command. This method is called periodically.
        """
        offsets = self.vision.process_frame(lambda frame: None)
        if offsets is None or len(offsets) < 2:
            Logger.info("No target detected. Ending TurnDrive.")
            self._stop_motors()
            self.turn_finished = True
            self.drive_finished = True
            return

        x_offset, z_offset = offsets
        self._handle_turning(x_offset, z_offset)
        self._handle_driving(z_offset)

        if self.turn_finished and self.drive_finished:
            self._stop_motors()
            self.end(False)

    def _handle_turning(self, x_offset, z_offset):
        if self.turn_finished:
            return

        delta_angle = math.degrees(math.atan2(x_offset, z_offset))
        Logger.info(f"Delta Angle: {delta_angle:.2f} degrees")

        if abs(delta_angle) < 1:
            self.turn_finished = True
            Logger.info("Turn completed.")
        else:
            turn_speed = self.turn_pid(delta_angle)
            self.motor_controller.set_motor_speeds(-turn_speed, turn_speed)

    def _handle_driving(self, z_offset):
        if self.drive_finished or not self.turn_finished:
            return

        drive_speed = self.drive_pid(z_offset)
        Logger.info(f"Driving forward with speed: {drive_speed:.2f}")

        if abs(z_offset) < 0.01:
            self.drive_finished = True
            Logger.info("Drive completed.")
        else:
            self.motor_controller.set_motor_speeds(-drive_speed, -drive_speed)

    def _stop_motors(self):
        self.motor_controller.set_motor_speeds(0, 0)

    def start(self):
        self.turn_finished = False
        self.drive_finished = False
        Logger.info("TurnDrive started.")

    def end(self, interrupted=False):
        self._stop_motors()
        Logger.info(f"TurnDrive ended. Interrupted: {interrupted}")

    def is_finished(self):
        return self.turn_finished and self.drive_finished



