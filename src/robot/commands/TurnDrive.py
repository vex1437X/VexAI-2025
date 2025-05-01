from time import time
import math
from simple_pid import PID
from src.core.commands.Command import Command
from src.core.logging.logger_config import get_logger

logger = get_logger(__name__)

class TurnDrive(Command):
    """
    Command to turn toward, then drive to, the nearest ring detected by Vision.
    """

    TURN_THRESHOLD_DEG = 1.0
    DRIVE_THRESHOLD_M = 0.5
    DRIVE_HOLD_SEC = 0.5

    def __init__(self, drivetrain, vision, max_speed: float = 50.0):
        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.max_speed = max_speed

        # placeholders; real PIDs created on initialize()
        self.turn_pid: PID
        self.drive_pid: PID

        # state flags
        self._turn_done = False
        self._drive_done = False
        self._hold_start: float | None = None

    def initialize(self) -> None:
        # PID for angle (degrees)
        self.turn_pid = PID(0.5, 0.0, 0.0, setpoint=0.0)
        self.turn_pid.output_limits = (-self.max_speed, self.max_speed)

        # PID for forward distance (meters or units from vision)
        self.drive_pid = PID(100.0, 0.1, 0.05, setpoint=0.0)
        self.drive_pid.output_limits = (-self.max_speed, self.max_speed)

        self._turn_done = False
        self._drive_done = False
        self._hold_start = None

        logger.info("TurnDrive initialized")

    def execute(self) -> None:
        if self._drive_done:
            return

        # get best (x_offset, z_offset) or None
        offsets = self._get_best_offset()
        if offsets is None:
            # nothing seen: stop motors and wait
            self.drivetrain.stop()
            return

        x_off, z_off = offsets

        if not self._turn_done:
            self._do_turn(x_off, z_off)
        else:
            self._do_drive(z_off)

    def is_finished(self) -> bool:
        return self._drive_done

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop()
        self.turn_pid.reset()
        self.drive_pid.reset()
        logger.info(f"TurnDrive {'interrupted' if interrupted else 'completed'}")

    # ──────────────────────────────────────────────────────────────────────────────

    def _get_best_offset(self) -> tuple[float, float] | None:
        try:
            raw = self.vision.process_frame()
        except Exception as e:
            logger.warning(f"Vision error: {e}")
            return None

        # unify to list of (x, z)
        if raw is None:
            return None
        candidates = raw.values() if isinstance(raw, dict) else raw

        best = min(
            ((x, z) for x, z in candidates if x is not None and z is not None),
            key=lambda p: math.hypot(p[0], p[1]),
            default=None
        )
        return best

    def _do_turn(self, x_off: float, z_off: float) -> None:
        angle = math.degrees(math.atan2(x_off, z_off))
        if abs(angle) <= self.TURN_THRESHOLD_DEG:
            self._turn_done = True
            self.drivetrain.stop()
            logger.debug("Turn complete")
        else:
            speed = self.turn_pid(angle)
            # left/right speeds for tank drive
            self.drivetrain.set_speed(-speed, speed)

    def _do_drive(self, z_off: float) -> None:
        # if within drive threshold, start hold timer
        if abs(z_off) <= self.DRIVE_THRESHOLD_M:
            if self._hold_start is None:
                self._hold_start = time()
                logger.debug("Drive hold started")
            elif time() - self._hold_start >= self.DRIVE_HOLD_SEC:
                self._drive_done = True
                self.drivetrain.stop()
                logger.info("Drive complete")
            return

        # driving toward target
        speed = self.drive_pid(z_off)
        self.drivetrain.set_speed(-speed, -speed)
        # reset hold timer if leaving threshold
        self._hold_start = None
