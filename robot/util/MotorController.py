from robot.util.SerialHelper import SerialHelper
from robot.util.Constants import Instruction

class MotorController:
    def __init__(self, serial_helper: SerialHelper):
        self._serial = serial_helper

    def set_speed(self, tag: Instruction, speed: list[float]):
        """
        Send a motor command for the given motor group (e.g. DRIVE_SET)
        """
        cmd = self._serial.encode_instruction(tag, speed)
        self._serial.send_command(cmd)

    def set_speed(self, tag: Instruction, speed: float):
        """
        Send a motor command for the given motor (e.g. FL or FR) -> TODO
        """
        cmd = self._serial.encode_instruction(tag, [speed])
        self._serial.send_command(cmd)

    def stop(self):
        """
        Stop all motors by sending a stop command.
        """
        # Assuming you have a specific instruction for stopping
        # stop_cmd = self._serial.encode_instruction(Instruction.STOP, []) TODO
        # self._serial.send_command(stop_cmd)