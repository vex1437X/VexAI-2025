from abc import ABC, abstractmethod

class SerialInterface(ABC):
    """
    Abstract interface for serial communication.
    """
    @abstractmethod
    def open(self) -> None:
        pass

    @abstractmethod
    def read(self) -> bytes:
        pass

    @abstractmethod
    def write(self, data: bytes) -> None:
        pass

    @abstractmethod
    def close(self) -> None:
        pass