"""
test_pose_estimator.py
Offline demo for PoseEstimatorSubsystem + PoseVisualizer.

Simulates a robot driving in a 1 m radius CCW circle (ω = 0.5 rad s⁻¹)
and streams synthetic GPS + IMU packets into the estimator at ~20 Hz.
"""

import math
import time
from enum import Enum
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# 1)  Data‑tag fallback (only used if your own Enum isn’t importable)
# ---------------------------------------------------------------------------
try:
    from robot.util.Constants import DataTag  # ← use your real enum
except ModuleNotFoundError:

    class DataTag(str, Enum):  # minimal stub
        GPS_X = "gps_x"
        GPS_Y = "gps_y"
        GPS_HEADING = "gps_heading"
        IMU_YAW = "imu_yaw"
        IMU_ACCEL_X = "imu_ax_g"
        IMU_ACCEL_Y = "imu_ay_g"


# ---------------------------------------------------------------------------
# 2)  Fake serial helper – creates ONE fresh packet each call
# ---------------------------------------------------------------------------
class _FakeSerialHelper:
    """Mimics SerialHelper.read_vex_data().

    Every invocation returns a list with a single dict whose keys are DataTag
    values.  The packet corresponds to a point on a circle plus consistent
    yaw and accelerometer data.
    """

    G_TO_M_S2 = 9.80665

    def __init__(self, radius_m: float = 1.0, omega: float = 0.5):
        self.r = radius_m  # metres
        self.w = omega  # rad s⁻¹
        self.t0 = time.time()

    # Called by PoseEstimatorSubsystem.periodic()
    def read_vex_data(self):
        t = time.time() - self.t0
        x = self.r * math.cos(self.w * t)
        y = self.r * math.sin(self.w * t)
        heading_rad = self.w * t + math.pi / 2  # tangent
        heading_deg = math.degrees(heading_rad) % 360

        # True centre‑seeking acceleration components (a = −ω²r)
        ax_m_s2 = -(self.w**2) * self.r * math.cos(self.w * t)
        ay_m_s2 = -(self.w**2) * self.r * math.sin(self.w * t)

        pkt = {
            DataTag.GPS_X: x,
            DataTag.GPS_Y: y,
            DataTag.GPS_HEADING: heading_deg,
            DataTag.IMU_YAW: heading_deg,  # perfect agreement
            DataTag.IMU_ACCEL_X: ax_m_s2 / self.G_TO_M_S2,
            DataTag.IMU_ACCEL_Y: ay_m_s2 / self.G_TO_M_S2,
        }
        return [pkt]


# ---------------------------------------------------------------------------
# 3)  Import your real estimator & visualiser  (adjust paths as needed)
# ---------------------------------------------------------------------------
from robot.subsystems.PoseEstimator import PoseEstimatorSubsystem
from robot.subsystems.PoseEstimator import PoseVisualizer


# ---------------------------------------------------------------------------
# 4)  Tiny adapter so PoseVisualizer calls also advance the estimator
# ---------------------------------------------------------------------------
class _EstimatorAdapter:
    """Wraps PoseEstimatorSubsystem so that every Visualizer poll performs
    one periodic() step, keeping things single‑threaded.
    """

    def __init__(self, estimator):
        self.est = estimator

    def get_pose(self):
        self.est.periodic()  # run one control‑loop cycle
        return self.est.get_pose()

    def get_velocity(self):
        return self.est.vx, self.est.vy


# ---------------------------------------------------------------------------
# 5)  Go!
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    serial = _FakeSerialHelper()  # synthetic sensor stream
    estimator = PoseEstimatorSubsystem(serial)  # your fusion code
    viz = PoseVisualizer(_EstimatorAdapter(estimator))

    print("Close the plot window to exit.")
    plt.show()
