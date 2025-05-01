# tests/test_robotcontainer.py
import pytest
from src.robot.RobotContainer import RobotContainer
from tests.dummies import DummySerial, DummyCamera, DummyMotor, DummyJoystick

def test_container_with_dummies():
    dummy_serial = DummySerial(responses=[b""])
    dummy_camera = DummyCamera(frame_pairs=[(None, None)])
    dummy_motor  = DummyMotor()
    dummy_joystick = DummyJoystick()  # you’d implement this to match pygame.joystick.Joystick API

    rc = RobotContainer(
        serial_interface=dummy_serial,
        camera_interface=dummy_camera,
        motor_interface=dummy_motor,
        joystick=dummy_joystick,
    )
    
    while True:
        rc.periodic()

if __name__ == "__main__":
    test_container_with_dummies()
    # pytest.main([__file__])