class Command:
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        pass

    def is_finished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass