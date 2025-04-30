class Command:
    def __init__(self, linkedCommand=None):
        self.linkedCommand = None

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        pass

    def is_finished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass
