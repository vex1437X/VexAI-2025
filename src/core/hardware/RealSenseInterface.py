from abc import ABC, abstractmethod

class RealSenseInterface(ABC):
    """
    Abstract interface for Intel RealSense camera.
    """
    @abstractmethod
    def configure(self, settings: dict) -> None:
        pass

    @abstractmethod
    def get_frame(self):
        pass

    @abstractmethod
    def close(self) -> None:
        pass