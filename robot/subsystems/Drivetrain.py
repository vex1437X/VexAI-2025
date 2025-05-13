from robot.util.Subsystem import Subsystem
from robot.commands.TeleopCommand import Teleop


class Drivetrain(Subsystem):
    def __init__(
        self, joystick=None, max_speed=100, drive_mode="tank", motor_controller=None
    ):
        self.motor_controller = motor_controller
        self.joystick = joystick
        self.max_speed = max_speed
        self.drive_mode = drive_mode
        self.command = (
            Teleop(joystick, max_speed, drive_mode, motor_controller) if joystick else None
        )
        # if self.command:
        #     print("Command initialized.")
        # else:
        #     print("Command not initialized.")
