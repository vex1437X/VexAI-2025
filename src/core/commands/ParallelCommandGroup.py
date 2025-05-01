from typing import List
from .Command import Command

class ParallelCommandGroup(Command):
    """
    Runs all commands in parallel until all finish.
    """
    def __init__(self, commands: List[Command]):
        super().__init__()
        self._commands = commands

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        for cmd in self._commands:
            if not cmd._initialized:
                cmd.initialize()
                cmd._initialized = True
            cmd.execute()

    def is_finished(self) -> bool:
        return all(cmd.is_finished() for cmd in self._commands)

    def end(self, interrupted: bool) -> None:
        for cmd in self._commands:
            cmd.end(interrupted)