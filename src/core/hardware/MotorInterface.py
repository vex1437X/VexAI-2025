from abc import ABC, abstractmethod

class MotorInterface(ABC):
    """
    Abstract interface for motor controllers.
    """
    @abstractmethod
    def set_speed(self, speed: float) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass