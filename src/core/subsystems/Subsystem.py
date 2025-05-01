from abc import ABC, abstractmethod

class Subsystem(ABC):
    """
    Base class for all subsystems.
    """
    @abstractmethod
    def on_enable(self) -> None:
        """Called when the subsystem is enabled at match start."""
        pass

    @abstractmethod
    def on_disable(self) -> None:
        """Called when the subsystem is disabled at match end."""
        pass