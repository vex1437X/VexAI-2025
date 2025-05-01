from typing import Set
from .Command import Command

class CommandScheduler:
    """
    Singleton scheduler for managing command lifecycles.
    """
    _instance = None

    def __init__(self):
        self._scheduled: Set[Command] = set()

    @staticmethod
    def get_instance():
        if CommandScheduler._instance is None:
            CommandScheduler._instance = CommandScheduler()
        return CommandScheduler._instance

    def schedule(self, command: Command) -> None:
        self._scheduled.add(command)

    def run(self) -> None:
        to_remove = set()
        for cmd in list(self._scheduled):
            finished = cmd._run()
            if finished:
                to_remove.add(cmd)
        self._scheduled -= to_remove