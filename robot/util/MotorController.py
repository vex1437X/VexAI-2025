from robot.util.SerialHelper import SerialHelper
from robot.util.Constants import Instruction, DataTag


class MotorController:
    def __init__(self, serial_helper: SerialHelper):
        self._serial = serial_helper

    def set_speed(self, tag: Instruction, speed: list[float]):
        """
        Send a motor command for the given motor group (e.g. DRIVE_SET)
        """
        cmd = self._serial.encode_instruction(tag, speed)
        self._serial.send_command(cmd)
        # print(speed)

    def drive_holomonic(
        self,
        vx: float,
        vy: float,
        omega: float,
        max_speed: float,
        normalize: bool = True,
    ):
        """
        Drive the robot in a holonomic manner using mecanum wheels.
        """
        vxf = -vx
        # Mecanum inverse kinematics (robot‑centric)
        # Reference: https://www.chiefdelphi.com/uploads/short-url/bsbgWq9Rgl9Q0IvqpuXzYjPtwic.pdf
        fl = vy + vxf + omega  # Front Left
        fr = vy - vxf - omega  # Front Right
        bl = vy - vxf + omega  # Back Left
        br = vy + vxf - omega  # Back Right
        wheel = [fl, fr, bl, br]  # FL, FR, RL, RR

        # Scale so that |wheel| ≤ max_speed
        max_mag = max(abs(w) for w in wheel)
        max_mag = max(max_mag, 1e-6)
        scale = max_speed / max_mag
        wheel = [w * scale for w in wheel]

        # Send to motor controller
        self.set_speed(Instruction.DRIVE_SET, wheel)
        print(f"{vx}, {vy}, {omega}")

    def stop(self):
        """
        Stop all motors by sending a stop command.
        """
        # Assuming you have a specific instruction for stopping
        # stop_cmd = self._serial.encode_instruction(Instruction.STOP, []) TODO
        # self._serial.send_command(stop_cmd)

    def get_gyro(self):
        return self._serial.value(DataTag.GYRO)
