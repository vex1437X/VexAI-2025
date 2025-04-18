from enum import Enum


class Instruction(Enum):
    DRIVE_SET = 0x07


# Enum for data tags used in VEX data packets
class DataTag(Enum):
    GPS_X = 0x01
    GPS_Y = 0x02
    GPS_HEADING = 0x03
    GPS_YAW = 0x04
