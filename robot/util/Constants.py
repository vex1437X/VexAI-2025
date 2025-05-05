from enum import Enum


class Instruction(Enum):
    DRIVE_SET = 0x07


# Enum for data tags used in VEX data packets
class DataTag(Enum):
    GPS_X = 0x01
    GPS_Y = 0x02
    GPS_HEADING = 0x03
    GPS_YAW = 0x04

    IMU_PITCH = 0x05
    IMU_ROLL = 0x06
    IMU_YAW = 0x07
    IMU_ACCEL_X = 0x08
    IMU_ACCEL_Y = 0x09
    IMU_ACCEL_Z = 0x0A
