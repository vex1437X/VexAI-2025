import json
from pathlib import Path
import pygame

from src.core.config.ConfigModel import ConfigModel
from src.core.commands.CommandScheduler import CommandScheduler
from .hardware.PySerialInterface import PySerialInterface
from .hardware.RealSenseCamera import RealSenseCamera
from .hardware.VexMotorController import VexMotorController
from .subsystems.Drivetrain import Drivetrain
from .subsystems.Vision import Vision
from .commands.TeleopCommand import TeleopCommand

from tests.dummies import DummyMotor

def load_config() -> ConfigModel:
    # Load raw JSON and validate with Pydantic v2
    content = Path('config.json').read_text()
    return ConfigModel.model_validate_json(content)

class RobotContainer:
    def __init__(
        self,
        serial_interface=None,
        camera_interface=None,
        motor_interface=None,
        joystick=None,
    ):
        self.config = load_config()

        # Serial: inject dummy or configure singleton
        if serial_interface is not None:
            self.serial = serial_interface
            PySerialInterface._instance = serial_interface
        else:
            self.serial = PySerialInterface.get_instance(self.config)

        # Camera: inject dummy or real
        if camera_interface is not None:
            self.camera = camera_interface
        else:
            self.camera = RealSenseCamera.get_instance()
            self.camera.configure(self.config.camera_settings)

        # Joystick: inject dummy or real
        if joystick is None:
            pygame.init()
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            self.controller = joystick

        # Subsystems
        self.drivetrain = Drivetrain(joystick=self.controller, fl_motor = DummyMotor(), fr_motor = DummyMotor(), bl_motor = DummyMotor(), br_motor = DummyMotor())
        self.vision_system = Vision(
            camera=self.camera,
        )

        # Commands
        teleop = TeleopCommand(drivetrain=self.drivetrain, joystick=self.controller)
        CommandScheduler.get_instance().schedule(teleop)

    def periodic(self):
        # print("Running periodic tasks")
        CommandScheduler.get_instance().run()