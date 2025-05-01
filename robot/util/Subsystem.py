from robot.commands.Search import Search
from robot.commands.TurnDrive import TurnDrive


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
        print("Command class is" + str(type(self.command)))
        if self.command:
            self.command.execute()
            if self.command.is_finished():
                self.command.end(False)
                if isinstance(self.command, Search):
                    self.command = TurnDrive(
                        serialHelper=self.serialHelper,
                        vision=self.command.vision,
                    )
                elif isinstance(self.command, TurnDrive):
                    self.command = Search(
                        serialHelper=self.serialHelper,
                        vision=self.command.vision,
                    )
                else:
                    self.command = None
                self.command.start()

    def set_command(self, command):
        """
        Schedule a new command for the subsystem.

        Args:
            command (Command): The command to execute.
        """
        if self.command and not self.command.is_finished():
            # print(f"Command {self.command} is not finished. Ending it.")
            self.command.end(True)
        elif self.command:
            # print(f"Command {self.command} is finished. Ending it.")
            self.command.end(False)
        # print(f"Setting new command: {command}")
        command.start()
        self.command = command
