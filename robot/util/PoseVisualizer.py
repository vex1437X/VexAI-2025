"""PoseVisualizer
================
Realtime Matplotlib window that displays robot **position** (red dot) and
**velocity** (red arrow) on the 3.6 m × 3.6 m VEX field.

Typical usage::

    from pose_estimator import PoseEstimatorSubsystem
    from pose_visualizer import PoseVisualizer

    estimator = PoseEstimatorSubsystem(serial_helper)
    vis       = PoseVisualizer(estimator)

    while True:
        estimator.periodic()
        time.sleep(0.02)  # Your control‑loop delay
"""

from __future__ import annotations

import math
import time
from typing import Protocol, Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Type protocol – duck‑typed interface required by the visualiser
# ---------------------------------------------------------------------------
class _EstimatorLike(Protocol):
    def get_pose(self) -> Tuple[float, float, float]: ...
    def get_velocity(self) -> Tuple[float, float]: ...


# ---------------------------------------------------------------------------
# Visualiser implementation
# ---------------------------------------------------------------------------
class PoseVisualizer:
    """Live scatter/arrow plot for robot pose and velocity."""

    FIELD_MIN = -1.8  # m
    FIELD_MAX = 1.8  # m
    DEFAULT_ARROW_SCALE = 0.25  # metres of arrow per 1 m/s

    def __init__(
        self,
        estimator: _EstimatorLike,
        update_hz: float = 20.0,
        arrow_scale: float | None = None,
    ) -> None:
        self._estimator = estimator
        self._dt_ms = 1000.0 / update_hz
        self._arrow_scale = arrow_scale or self.DEFAULT_ARROW_SCALE

        # Fallback finite‑difference velocity if estimator lacks get_velocity
        self._prev_x: float | None = None
        self._prev_y: float | None = None
        self._prev_t: float | None = None

        self._setup_plot()
        self._ani = animation.FuncAnimation(
            self._fig,
            self._update,
            interval=self._dt_ms,
            blit=True,
        )
        plt.show(block=False)

    # ------------------------------------------------------------------
    # Matplotlib boilerplate
    # ------------------------------------------------------------------
    def _setup_plot(self):  # noqa: D401 – imperative mood preferred
        self._fig, self._ax = plt.subplots()
        self._ax.set_aspect("equal")
        self._ax.set_xlim(self.FIELD_MIN, self.FIELD_MAX)
        self._ax.set_ylim(self.FIELD_MIN, self.FIELD_MAX)
        self._ax.set_xlabel("X (m)")
        self._ax.set_ylabel("Y (m)")

        # Field border and quadrant axes
        square = [
            (self.FIELD_MIN, self.FIELD_MIN),
            (self.FIELD_MAX, self.FIELD_MIN),
            (self.FIELD_MAX, self.FIELD_MAX),
            (self.FIELD_MIN, self.FIELD_MAX),
            (self.FIELD_MIN, self.FIELD_MIN),
        ]
        self._ax.plot(*zip(*square), color="black")
        self._ax.axhline(0.0, linestyle="--", color="gray", linewidth=0.8)
        self._ax.axvline(0.0, linestyle="--", color="gray", linewidth=0.8)

        (self._dot,) = self._ax.plot([], [], marker="o", color="red")
        (self._vel_line,) = self._ax.plot([], [], color="red")

    # ------------------------------------------------------------------
    # Animation callback
    # ------------------------------------------------------------------
    def _update(self, _frame):
        x, y, _ = self._estimator.get_pose()

        # Velocity – estimator if present, fallback to Δp/Δt.
        if hasattr(self._estimator, "get_velocity"):
            vx, vy = self._estimator.get_velocity()
        else:
            now = time.time()
            if self._prev_x is None:
                vx = vy = 0.0
            else:
                dt = max(now - self._prev_t, 1e-6)
                vx = (x - self._prev_x) / dt
                vy = (y - self._prev_y) / dt
            self._prev_x, self._prev_y, self._prev_t = x, y, now

        speed = math.hypot(vx, vy)
        vx_draw = vx * self._arrow_scale
        vy_draw = vy * self._arrow_scale

        # Update artists
        self._dot.set_data([x], [y])
        self._vel_line.set_data([x, x + vx_draw], [y, y + vy_draw])
        self._ax.set_title(f"Robot Pose & Velocity — Speed: {speed:.2f} m/s")

        return self._dot, self._vel_line


# ---------------------------------------------------------------------------
# Stand‑alone quick‑test
# ---------------------------------------------------------------------------
if __name__ == "__main__":

    class _FakeEstimator:
        """Circular motion at 0.5 rad/s for development testing."""

        def __init__(self):
            self.t0 = time.time()
            self.r = 1.0  # m radius
            self._prev_t: float | None = None
            self._prev_x: float | None = None
            self._prev_y: float | None = None
            self.vx = self.vy = 0.0

        def get_pose(self):
            t = time.time() - self.t0
            x = self.r * math.cos(t * 0.5)
            y = self.r * math.sin(t * 0.5)
            heading = t * 0.5 + math.pi / 2
            return x, y, heading

        def get_velocity(self):
            x, y, _ = self.get_pose()
            now = time.time()
            if self._prev_x is None:
                self.vx = self.vy = 0.0
            else:
                dt = max(now - self._prev_t, 1e-6)
                self.vx = (x - self._prev_x) / dt
                self.vy = (y - self._prev_y) / dt
            self._prev_x, self._prev_y, self._prev_t = x, y, now
            return self.vx, self.vy

    print("Close the plot window to exit…")
    PoseVisualizer(_FakeEstimator())
    plt.show()
