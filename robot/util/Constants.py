from enum import Enum


class Instruction(Enum):
    LF_SET = 0x01
    RF_SET = 0x02
    LB_SET = 0x03
    RB_SET = 0x04
    MOTOR_SET = 0x05
    INTAKE_SET = 0x06
    DRIVE_SET = 0x07


# Enum for data tags used in VEX data packets
class DataTag(Enum):
    GPS0_X = 0x01
    GPS0_Y = 0x02
    GPS0_H = 0x03
    GPS1_X = 0x04
    GPS1_Y = 0x05
    GPS1_H = 0x06

    GYRO = 0x07

    FL = 0x0B
    FR = 0x0C
    BL = 0x0D
    BR = 0x0E
