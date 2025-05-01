from pydantic import BaseModel

class ConfigModel(BaseModel):
    serial_port: str
    baud_rate: int = 115200
    timeout: float = 0.01
    max_speed: float
    deadzone: float = 0.05
    camera_settings: dict
    fl_motor_id: int
    fr_motor_id: int
    bl_motor_id: int
    br_motor_id: int

    class Config:
        extra = "forbid"