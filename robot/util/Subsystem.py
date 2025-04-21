class Subsystem:
    """
    Base class for all robot subsystems.
    """

    def __init__(self, serialHelper=None):
        self.serialHelper = serialHelper
        self.command = None

    def tick(self):
        """
        Periodically execute the scheduled commands.
        """
        if self.command:
            self.command.execute()
            if self.command.is_finished():
                self.command.end(False)

    def set_command(self, command):
        """
        Schedule a new command for the subsystem.

        Args:
            command (Command): The command to execute.
        """
        if self.command and not self.command.is_finished():
            self.command.end(True)
        elif self.command:
            self.command.end(False)
        command.start()
        self.command = command
