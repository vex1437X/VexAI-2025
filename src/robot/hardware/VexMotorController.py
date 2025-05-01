from src.core.hardware.MotorInterface import MotorInterface
from .PySerialInterface import PySerialInterface
from src.core.config.util import Instruction

class VexMotorController(MotorInterface):
    """
    VEX motor controller interface.
    """

    def __init__(self, motor_id: int) -> None:
        """
        Initialize the VEX motor controller interface.
        """
        self._serial = PySerialInterface.get_instance()
        self._motor_id = None
        self.speed = 0

    def set_speed(self, speed: float) -> None:
        """
        Set the speed of the motor.
        """
        command = PySerialInterface.encode_instruction(instr=Instruction.MOTOR_SET, operands=[self._motor_id, speed])
        self._serial.send_command(command)
        self.speed = speed
        

    def stop(self) -> None:
        """
        Stop the motor.
        """
        command = PySerialInterface.encode_instruction(instr=Instruction.MOTOR_SET, operands=[self._motor_id, 0])
        self._serial.send_command(command)
        self.speed = 0
    
    def get_speed(self) -> float:
        """
        Get the current speed of the motor.
        """
        return self.speed
        