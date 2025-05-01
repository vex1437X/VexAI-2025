from src.core.commands.Command import Command
from src.core.logging.logger_config import get_logger

logger = get_logger(__name__)

class Search(Command):
    def __init__(self, vision=None, drivetrain=None, turn_speed=20):
        super().__init__()
        self.turn_speed = turn_speed
        self.vision = vision
        self.drivetrain = drivetrain

    def initialize(self):
        pass

    def execute(self):
        # turn left until an object is detected, then end the command
        self.drivetrain.set_speed(-self.turn_speed, self.turn_speed)
        

    def is_finished(self):
        # check if an object is detected
        raw = self.vision.process_frame()
        logger.info(f"Search: {raw}")
        if len(raw) == 0:
            return False
        return True

    def end(self, interrupted=False):
        # stop the motors
        self.drivetrain.stop()
