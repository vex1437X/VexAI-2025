from robot.RobotContainer import RobotContainer
from robot.server.vision_server import VisionServer
import time

def main() -> None:
    # Initialize robot and vision server
    robot_container = RobotContainer()
    vision_server = VisionServer()
    vision_server.start()
    
    # Main robot control loop
    while True:
        robot_container.periodic()
        # Update vision server with current frame
        if robot_container.vision:
            vision_server.update_frame(robot_container.vision.get_frame())
        time.sleep(0.02)

if __name__ == "__main__":
    main()
