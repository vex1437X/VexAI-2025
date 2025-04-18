from robot.util.CommandScheduler import CommandScheduler

class Subsystem:
    """
    Base class for all robot subsystems.
    """

    def __init__(self, serialHelper=None):
        self.serialHelper = serialHelper
        self.command_scheduler = CommandScheduler()

    def tick(self):
        """
        Periodically execute the scheduled commands.
        """
        self.command_scheduler.run()

    def set_command(self, command):
        """
        Schedule a new command for the subsystem.

        Args:
            command (Command): The command to execute.
        """
        self.command_scheduler.schedule(command)

