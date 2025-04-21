from robot.RobotContainer import RobotContainer


def main() -> None:
    robot_container = RobotContainer()  # builds subsystems, commands, bindings
    while True:
        robot_container.periodic()


if __name__ == "__main__":
    main()
