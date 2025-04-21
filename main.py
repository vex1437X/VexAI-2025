from robot.RobotContainer import RobotContainer
import time


def main() -> None:
    robot_container = RobotContainer()  # builds subsystems, commands, bindings
    while True:
        robot_container.periodic()


if __name__ == "__main__":
    main()
    time.sleep(0.02)
