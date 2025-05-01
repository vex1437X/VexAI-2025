from src.core.hardware.MotorInterface import MotorInterface
from .PySerialInterface import PySerialInterface
from src.core.config.util import Instruction
from src.core.logging.logger_config import get_logger

logger = get_logger(__name__)


class VexMotorController(MotorInterface):
    """
    VEX motor controller interface.
    """

    def __init__(self, motor_id: int = -1) -> None:
        """
        Initialize the VEX motor controller interface.
        """
        self._serial = PySerialInterface.get_instance()
        self._motor_id = motor_id
        self.speed = 0
        logger.info(f"VexMotorController initialized with motor ID: {motor_id}")

    def set_speed(self, speed: float) -> None:
        """
        Set the speed of the motor.
        """
        command = self._serial.encode_instruction(
            instr=Instruction.MOTOR_SET, operands=[int(self._motor_id), speed]
        )
        self._serial.write(command)
        self.speed = speed
        # logger.info(f"Motor {self._motor_id} speed set to {speed}")

    def stop(self) -> None:
        """
        Stop the motor.
        """
        command = self._serial.encode_instruction(
            instr=Instruction.MOTOR_SET, operands=[int(self._motor_id), 0]
        )
        self._serial.write(command)
        self.speed = 0

    def get_speed(self) -> float:
        """
        Get the current speed of the motor.
        """
        return self.speed
