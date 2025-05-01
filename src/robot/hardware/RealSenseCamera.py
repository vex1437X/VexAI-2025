from src.core.hardware.RealSenseInterface import RealSenseInterface
import pyrealsense2 as rs # type: ignore
import numpy as np

class RealSenseCamera(RealSenseInterface):
    """
    Concrete implementation of RealsenseInterface using pyrealsense2.
    """
    _instance = None

    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None

    @classmethod
    def get_instance(cls) -> "RealSenseCamera":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def configure(self, settings: dict) -> None:
        self.config.enable_stream(rs.stream.color,
                                  settings.get('width',640),
                                  settings.get('height',480),
                                  rs.format.bgr8,
                                  settings.get('fps',30))
        self.profile = self.pipeline.start(self.config)

    def get_frame(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data())

    def close(self) -> None:
        self.pipeline.stop()