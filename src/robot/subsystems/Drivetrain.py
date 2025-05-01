from src.core.subsystems.Subsystem import Subsystem
from src.core.hardware.MotorInterface import MotorInterface
from src.robot.hardware.VexMotorController import VexMotorController


class Drivetrain(Subsystem):
    """
    Subsystem controlling a four-motor tank drivetrain.
    Allows injection of test motor instances for unit testing.
    """

    def __init__(
        self,
        fl_motor: MotorInterface | None = None,
        fr_motor: MotorInterface | None = None,
        bl_motor: MotorInterface | None = None,
        br_motor: MotorInterface | None = None,
        joystick=None,
        max_speed: float = 50.0,
        drive_mode: str = "tank",
        fl_motor_id: int = 1,
        fr_motor_id: int = 2,
        bl_motor_id: int = 3,
        br_motor_id: int = 4,
    ):
        # Assign joystick and drive parameters
        self.joystick = joystick
        self.max_speed = max_speed
        self.drive_mode = drive_mode

        # Injected or real motor controllers
        self.fl_motor = (
            fl_motor if fl_motor is not None else VexMotorController(fl_motor_id)
        )
        self.fr_motor = (
            fr_motor if fr_motor is not None else VexMotorController(fr_motor_id)
        )
        self.bl_motor = (
            bl_motor if bl_motor is not None else VexMotorController(bl_motor_id)
        )
        self.br_motor = (
            br_motor if br_motor is not None else VexMotorController(br_motor_id)
        )

    def on_enable(self) -> None:
        """Hook called when the drivetrain is enabled."""
        # No special startup behavior
        pass

    def on_disable(self) -> None:
        """Hook called when the drivetrain is disabled."""
        self.stop()

    def set_speed(self, left_speed: float, right_speed: float) -> None:
        """
        Drive the left and right sides at specified speeds.

        Args:
            left_speed (float): Speed for left motors [-max_speed, max_speed].
            right_speed (float): Speed for right motors [-max_speed, max_speed].
        """
        # Tank drive mapping
        self.fl_motor.set_speed(left_speed)
        self.bl_motor.set_speed(left_speed)
        self.fr_motor.set_speed(right_speed)
        self.br_motor.set_speed(right_speed)

    def stop(self) -> None:
        """Stop all drive motors."""
        self.fl_motor.stop()
        self.fr_motor.stop()
        self.bl_motor.stop()
        self.br_motor.stop()
