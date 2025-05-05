"""PoseEstimatorSubsystem
================================
Fuses VEX GPS and IMU data to estimate robot translation, heading, **and** velocity on the
3.6 m × 3.6 m field.

*   **Coordinate frame** – X right (east), Y forward (north), heading 0 rad along +X
    and increasing counter‑clockwise.
*   **Velocity** – computed every control‑loop tick as `(p_f – p_i)/dt`, so any drift
    in integrated accelerometer data is eliminated.

The class is designed to live inside your WPILib‑style robot code: call
:meth:`periodic` once per loop (~20 ms) and query :meth:`get_pose` /
:meth:`get_velocity` as needed by other subsystems.
"""

from __future__ import annotations

import math
import time
from robot.util.Constants import DataTag
from robot.util.Subsystem import Subsystem
from typing import Tuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FIELD_MIN = -1.8  # m
FIELD_MAX = 1.8


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------
class PoseEstimator(Subsystem):
    """Robot‑centric pose estimator with field‑centric outputs."""

    def __init__(self, serial_helper, gps_alpha: float = 0.25) -> None:
        """Args
        -----
        serial_helper
            Object exposing :py:meth:`read_vex_data`, returning an iterable of
            sensor packets (dict‑like access using :class:`robot.util.Constants.DataTag`).
        gps_alpha
            0–1 complementary‑filter gain; higher → trust GPS more (defaults to 0.25).
        """
        # External dependencies
        self._serial = serial_helper

        # Pose state (m / rad)
        self.x: float = 0.0
        self.y: float = 0.0
        self.heading: float = 0.0

        # Velocity state (m/s)
        self.vx: float = 0.0
        self.vy: float = 0.0
        self._prev_x: float | None = None
        self._prev_y: float | None = None

        # Heading‑fusion helpers
        self._heading_offset_deg: float = 0.0
        self._heading_initialised = False

        # Filtering
        self._gps_alpha = gps_alpha

        # Timing
        self._last_ts: float = time.time()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def get_pose(self) -> Tuple[float, float, float]:
        """Return ``(x, y, heading)`` (heading in **radians**)."""
        return self.x, self.y, self.heading

    def get_velocity(self) -> Tuple[float, float]:
        """Return ``(vx, vy)`` in m/s."""
        return self.vx, self.vy

    # ------------------------------------------------------------------
    # Framework hook
    # ------------------------------------------------------------------
    def periodic(self) -> None:
        """Call *once* per control loop (~20 ms)."""
        now = time.time()
        dt = now - self._last_ts if self._last_ts else 0.0
        self._last_ts = now

        # Consume all pending packets from the co‑processor
        for pkt in self._serial.read_vex_data():
            self._process_packet(pkt)

        # ---------------- Velocity (v = Δp/Δt) ----------------
        if self._prev_x is not None and dt > 0.0:
            self.vx = (self.x - self._prev_x) / dt
            self.vy = (self.y - self._prev_y) / dt
        else:
            self.vx = self.vy = 0.0

        self._prev_x, self._prev_y = self.x, self.y

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _process_packet(self, pkt: dict) -> None:
        """Decode **one** VEX sensor packet."""
        # --- Heading fusion -------------------------------------------------
        gps_head = pkt.get(DataTag.GPS_HEADING)
        imu_yaw = pkt.get(DataTag.IMU_YAW)

        if (
            gps_head is not None
            and imu_yaw is not None
            and not self._heading_initialised
        ):
            self._heading_offset_deg = gps_head - imu_yaw
            self._heading_initialised = True

        if imu_yaw is not None and self._heading_initialised:
            fused = (imu_yaw + self._heading_offset_deg) % 360.0
            self.heading = math.radians(fused)

        # --- Position fusion ----------------------------------------------
        gps_x = pkt.get(DataTag.GPS_X)
        gps_y = pkt.get(DataTag.GPS_Y)
        if gps_x is not None and gps_y is not None:
            self.x = self._gps_alpha * gps_x + (1 - self._gps_alpha) * self.x
            self.y = self._gps_alpha * gps_y + (1 - self._gps_alpha) * self.y

        # --- Clamp to field -----------------------------------------------
        self.x = max(FIELD_MIN, min(FIELD_MAX, self.x))
        self.y = max(FIELD_MIN, min(FIELD_MAX, self.y))

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def reset(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0) -> None:
        """Force‑reset pose and velocity (e.g. at match start)."""
        self.x, self.y = x, y
        self.heading = math.radians(heading_deg)
        self.vx = self.vy = 0.0
        self._prev_x = self._prev_y = None
        self._heading_initialised = False
