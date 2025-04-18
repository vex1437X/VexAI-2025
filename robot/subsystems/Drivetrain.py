from robot.util.Subsystem import Subsystem
from robot.commands.TeleopCommand import Teleop
from robot.util.Logger import Logger


class Drivetrain(Subsystem):
    def __init__(
        self, joystick=None, max_speed=50, drive_mode="tank", serialHelper=None
    ):
        super().__init__(serialHelper)
        self.joystick = joystick
        self.max_speed = max_speed
        self.drive_mode = drive_mode
        self.command = (
            Teleop(joystick, max_speed, drive_mode, serialHelper) if joystick else None
        )

        Logger.info("Drivetrain initialized.")
