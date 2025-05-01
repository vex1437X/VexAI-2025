# tests/dummies.py

from src.core.hardware.MotorInterface import MotorInterface
from src.core.hardware.SerialInterface import SerialInterface
from src.core.hardware.RealSenseInterface import RealSenseInterface
import numpy as np

class DummyMotor(MotorInterface):
    def __init__(self):
        self.commands = []  # record set_speed/stop calls

    def set_speed(self, speed: float) -> None:
        self.commands.append(("set_speed", speed))
        print

    def stop(self) -> None:
        self.commands.append(("stop", None))


class DummySerial(SerialInterface):
    def __init__(self, responses=None):
        # responses is a list of byte‐strings that read() will return
        self._responses = responses or []
        self.written = []

    def open(self) -> None:
        pass

    def read(self, size: int = 1) -> bytes:
        # pop next canned response (or return empty)
        return self._responses.pop(0) if self._responses else b""

    def write(self, data: bytes) -> None:
        self.written.append(data)

    def close(self) -> None:
        pass


class DummyCamera(RealSenseInterface):
    def __init__(self, frame_pairs):
        """
        frame_pairs: a list of (color_image, depth_frame) tuples that
        get_frame() will return in sequence
        """
        self._frames = list(frame_pairs)
        self.calls = 0

    def configure(self, settings: dict) -> None:
        pass

    def get_frame(self):
        # return the next tuple, or (None, None) when exhausted
        if self.calls < len(self._frames):
            pair = self._frames[self.calls]
            self.calls += 1
            return pair
        return None, None

    def close(self) -> None:
        pass

def DummyJoystick():
    """
    Create a dummy joystick object with attributes for testing.
    """
    class DummyJoystick:
        def __init__(self):
            self.axes = [0.0] * 6
            self.buttons = [0] * 12

        def get_axis(self, axis: int) -> float:
            return self.axes[axis]

        def get_button(self, button: int) -> int:
            return self.buttons[button]

    return DummyJoystick()