from abc import ABC, abstractmethod

class Command(ABC):
    """
    Base class for all commands in the framework.
    """
    def __init__(self):
        self._initialized = False

    @abstractmethod
    def initialize(self) -> None:
        """Called once when the command is scheduled."""
        pass

    @abstractmethod
    def execute(self) -> None:
        """Called every scheduler cycle until is_finished() returns True."""
        pass

    @abstractmethod
    def is_finished(self) -> bool:
        """Return True when the command has completed its task."""
        return True

    @abstractmethod
    def end(self, interrupted: bool) -> None:
        """Called once after is_finished() or if the command is cancelled."""
        pass

    def _run(self):
        if not self._initialized:
            self.initialize()
            self._initialized = True
        self.execute()
        if self.is_finished():
            self.end(False)
            return True
        return False