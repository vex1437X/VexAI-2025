# VexAI Testing Project

## Overview
A modular testing framework for the VexAI competition. Use this project to prototype and debug robot functionality, including drivetrain control, vision-based object detection, and command-based architecture. Note: joystick teleoperation and debug utilities are for testing only and will not be part of the final competition build.

## Features
- **Command-Based Architecture**: Modular, reusable commands for robot actions.
- **Subsystems**: Modular subsystems for easy implementation of new mechanical systems
- **Joystick Debugging**: Teleoperation support via joystick for rapid testing.
- **Vision Integration**: Intel RealSense vision subsystem for ring detection (and extendable to other objects).
- **Serial Communication**: Easy communication with the robot over a serial interface.
- **Logging**: Centralized logging utility for real-time debugging and monitoring.

## Project Structure
```
VexAI Server/
├── main.py                   # Application entry point
├── README.md                 # Project documentation
├── requirements.txt          # Python dependencies
└── robot/                    # Robot code
    ├── config.json           # Robot settings (serial port, speeds, etc.)
    ├── RobotContainer.py     # Central hub for subsystems, commands, and bindings
    ├── commands/             # Command implementations
    │   ├── DetectRing.py     # Ring-detection command
    │   ├── TeleopCommand.py  # Joystick teleoperation command
    │   ├── TurnDrive.py      # Command to turn and drive toward detected objects
    │   └── VisionCommand.py  # Base class for vision commands
    ├── subsystems/           # Subsystem implementations
    │   ├── Drivetrain.py     # Drivetrain control
    │   └── Vision.py         # Vision-based object detection
    └── util/                 # Utilities and helpers
        ├── Command.py            # Base command class
        ├── CommandScheduler.py   # Command scheduler
        ├── Constants.py          # Project constants and enums
        ├── Logger.py             # Logging utility
        ├── MotorController.py    # Motor control helper
        ├── SerialHelper.py       # Serial communication helper
        └── Subsystem.py          # Base subsystem class
```

## Prerequisites
- **Python**: 3.8 or newer
- **pip**: Python package installer

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/actually-upload-this-fine
   cd VexAI\ Server
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Configuration
Edit `robot/config.json` to match your hardware setup:
```json
{
  "serial_port": "/dev/tty.usbmodem142303",
  "baud_rate": 9600,
  "joystick_deadzone": 0.1,
  "max_speed": 50
}
```

## Usage
1. Connect your robot via USB and ensure your joystick is plugged in.
2. Run the application:
   ```bash
   python main.py
   ```

## Vision Subsystem
The Vision subsystem uses an Intel RealSense camera to detect objects. By default, the `DetectRing` command identifies rings based on HSV thresholds.

### Adding a New Vision Command
1. Create a new command inheriting from `VisionCommand`.
2. Implement the `mask_generator(self, color_frame)` method to return a binary mask.

**Example:**
```python
from robot.commands.VisionCommand import VisionCommand
import cv2
import numpy as np

class DetectCube(VisionCommand):
    def mask_generator(self, color_frame):
        img = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        return cv2.inRange(hsv, lower_blue, upper_blue)
```

## Debugging
- **Joystick Issues**: Ensure the joystick is connected and recognized:
  ```python
  import pygame
  pygame.joystick.init()
  ```
- **Logging**: All debug output is handled by `Logger` and printed to the console with timestamps and log levels.

---

_For more details, reach out to 111foxman111 on discord_

