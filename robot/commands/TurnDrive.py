from robot.util.Command import Command
import math
import time
from simple_pid import PID
from robot.util.Constants import Instruction
from robot.util.MotorController import MotorController


class TurnDrive(Command):
    """Holonomic (mecanum / swerve) command that simultaneously translates and
    rotates the robot toward a vision‑tracked target.

    The vision system must provide *robot‑centric* offsets:
        x  – metres right (+) / left (‑)
        z  – metres forward (+) / backward (‑)

    The controller converts those errors into the desired chassis twist
    (vx, vy, ω) each cycle and feeds them straight into the drivetrain’s
    inverse kinematics.  No turn‑then‑drive sequencing is required.
    """

    def __init__(
        self,
        motor_controller: MotorController,
        vision,
        *,
        max_speed: float = 50,  # percent‑output cap (‑100 … 100)
        kx: float = 1.5,  # strafe gain  (m/s per m)
        kz: float = 1.0,  # forward gain (m/s per m)
        k_theta: float = 1.5,  # rotation gain (rad/s per rad)
        distance_full_heading: float = 1.0,  # m at which we still spin freely
        pos_tol: float = 0.35,  # m       – consider position settled
        theta_tol: float = math.radians(3),  # rad – consider heading settled
        settle_time: float = 0.20,  # s both tolerances must hold
        wheelbase: float = 0.30,  # m front–back distance between wheels
        track_width: float = 0.30,  # m left–right distance between wheels
    ) -> None:
        super().__init__()
        self.mc = motor_controller
        self.vision = vision
        self.max_speed = max_speed

        # Simple P‑only PID objects for optional integral/derivative later
        self.pid_x = PID(kx, 0, 0, setpoint=0, output_limits=(-max_speed, max_speed))
        self.pid_z = PID(kz, 0, 0, setpoint=0, output_limits=(-max_speed, max_speed))
        self.pid_theta = PID(
            k_theta, 0, 0, setpoint=0, output_limits=(-max_speed, max_speed)
        )

        self.d0 = distance_full_heading
        self.pos_tol = pos_tol
        self.theta_tol = theta_tol
        self.settle_time = settle_time
        self._within_tol_since = None

        # kinematics constants
        self.L = wheelbase
        self.W = track_width

        # watchdog for vision loss
        self._last_seen = time.time()
        self._vision_timeout = 0.5  # s

    # ---------------------------------------------------------------------
    # Framework lifecycle hooks
    # ---------------------------------------------------------------------
    def start(self):
        self._within_tol_since = None
        self.pid_x.reset()
        self.pid_z.reset()
        self.pid_theta.reset()
        self._stop()

    def execute(self):
        offsets = self._get_offsets()
        if offsets is None:
            # vision lost; stop if lost too long
            if time.time() - self._last_seen > self._vision_timeout:
                self._stop()
            return

        x_off, z_off = offsets
        self._last_seen = time.time()

        # Translation commands (robot frame):
        vx = self.pid_x(x_off)  # strafe: + right, – left
        vy = self.pid_z(z_off)  # forward: + forward, – backward

        # Heading error (face the target)
        delta_theta = math.atan2(x_off, z_off)
        weight = min(abs(z_off) / self.d0, 1.0)  # taper spin when close
        omega = self.pid_theta(delta_theta) * weight

        self.mc.drive_holomonic(vx, vy, omega)

        # Finish criteria
        if (
            abs(x_off) < self.pos_tol
            and abs(z_off) < self.pos_tol
            and abs(delta_theta) < self.theta_tol
        ):
            if self._within_tol_since is None:
                self._within_tol_since = time.time()
            elif time.time() - self._within_tol_since >= self.settle_time:
                self.finished = True
        else:
            self._within_tol_since = None

    def is_finished(self) -> bool:
        return getattr(self, "finished", False)

    def end(self, interrupted: bool = False):
        self._stop()
        # Clean up for GC or next run
        self.pid_x.reset()
        self.pid_z.reset()
        self.pid_theta.reset()

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------
    def _get_offsets(self):
        # Instead of grabbing a new frame, use the latest tracked coordinates
        try:
            # Get all current target coordinates (spatial x,z) from the vision tracker
            tracked = list(self.vision.tracker.objects.values())  # list of (x,z) tuples
            if not tracked:
                return None
            # Find the closest object by Euclidean distance
            best = None
            best_dist = float("inf")
            for x, z in tracked:
                d = math.hypot(x, z)
                if d < best_dist:
                    best_dist = d
                    best = (x, z)
            return best
        except Exception as exc:
            # If vision data is not available for some reason, treat as no target
            return None

    def _stop(self):
        self.mc.set_speed(Instruction.DRIVE_SET, [0, 0, 0, 0])
