from enum import Enum


class Instruction(Enum):
    DRIVE_SET = 0x07


# Enum for data tags used in VEX data packets
class DataTag(Enum):
    GPS0_X = 0x01
    GPS0_Y = 0x02
    GPS1_X = 0x03
    GPS1_Y = 0x04

    GYRO = 0x05

    FL = 0x0B
    FR = 0x0C
    RL = 0x0D
    RR = 0x0E
