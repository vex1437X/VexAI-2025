from pydantic import BaseModel


class ConfigModel(BaseModel):
    serial_port: str
    baud_rate: int = 115200
    timeout: float = 0.01
    max_speed: float
    deadzone: float = 0.05
    camera_settings: dict
    fl_motor_id: int = 1
    fr_motor_id: int = 2
    bl_motor_id: int = 3
    br_motor_id: int = 4

    class Config:
        extra = "forbid"
