# ekf_xdrive.py
import numpy as np
import time


def wrap_angle(a):
    """Wrap angle to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


class PoseEstimator:
    """
    State  x = [x_world,
                y_world,
                theta_world]   (3×1)

    Covariance P (3×3)

    Wheel order convention
        0: front‑left   (FL)
        1: front‑right  (FR)
        2: rear‑left    (RL)
        3: rear‑right   (RR)

    All angles in **radians**,
    all distances in **metres**.
    """

    def __init__(
        self,
        wheel_radius_m: float,
        base_half_length_m: float,
        base_half_width_m: float,
        gps_offsets_m: list[tuple[float, float]],
        Q_diag: tuple[float, float, float] = (1e-4, 1e-4, 1e-5),
        R_gps_diag: tuple[float, float, float] = (0.03, 0.03, 0.02),
        R_gyro: float = 1e-3,
        R_accel_diag: tuple[float, float] = (0.1, 0.1),
    ):
        # Geometry
        self.r = wheel_radius_m
        self.L = base_half_length_m  # forward half‑length  (robot centre → wheel)
        self.W = base_half_width_m  # half‑width           (robot centre → wheel)

        # Sensor mounting: list of (x_offset, y_offset) for each GPS
        self.gps_offsets = gps_offsets_m

        # Noise covariances
        self.Q = np.diag(Q_diag)  # process (per loop, tune)
        self.R_gps = np.diag(R_gps_diag)  # each GPS reading
        self.R_gyro = np.array([[R_gyro]])  # gyro heading
        self.R_accel = np.diag(R_accel_diag)  # accelerometer

        # Filter state
        self.x = np.zeros((3, 1))  # start at origin
        self.P = np.eye(3) * 0.01  # moderate initial uncertainty

        self.prev_time = time.time()
        self.prev_gyro = 0.0  # gyro delta from last loop

    # ------------------------------------------------------------------
    #  P R E D I C T
    # ------------------------------------------------------------------
    def predict(self, wheel_v: np.ndarray, gyro_delta_rad: float, dt: float):
        """
        wheel_delta_rad : 4‑element array of wheel angle *increments* (rad) over dt
        gyro_delta_rad  : change in heading measured by gyro over dt (rad)
        dt              : timestep (s)
        """
        # 1 – Body‑frame twist from wheel deltas
        Vb = self._wheel_deltas_to_body_twist(wheel_v, dt)  # [vx, vy, omega]
        dtheta = gyro_delta_rad  # use gyro for ω*dt
        Vb[2] = dtheta / dt if dt > 0 else 0.0  # overwrite ω

        # 2 – Integrate to world frame
        theta = float(self.x[2, 0])
        c, s = np.cos(theta), np.sin(theta)
        # body → world rotation
        dx_world = c * Vb[0] - s * Vb[1]
        dy_world = s * Vb[0] + c * Vb[1]

        # Euler‑step state prediction
        self.x[0, 0] += dx_world * dt
        self.x[1, 0] += dy_world * dt
        self.x[2, 0] = wrap_angle(theta + dtheta)

        # 3 – Jacobian F of f(x,u) with respect to state
        F = np.eye(3)
        F[0, 2] = (-s * Vb[0] - c * Vb[1]) * dt
        F[1, 2] = (c * Vb[0] - s * Vb[1]) * dt

        # 4 – Propagate covariance
        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------------------------
    #  U P D A T E S
    # ------------------------------------------------------------------
    def update_gps(self, gps_index: int, z: np.ndarray):
        """
        z : 3‑vector [x_world, y_world, theta_world] from GPS #gps_index
        """
        ox, oy = self.gps_offsets[gps_index]
        theta = float(self.x[2, 0])
        c, s = np.cos(theta), np.sin(theta)

        # Predicted reading h(x)
        hx = np.array(
            [
                [self.x[0, 0] + c * ox - s * oy],
                [self.x[1, 0] + s * ox + c * oy],
                [wrap_angle(self.x[2, 0])],
            ]
        )

        # Jacobian H (3×3)
        H = np.array([[1, 0, -s * ox - c * oy], [0, 1, c * ox - s * oy], [0, 0, 1]])

        self._generic_update(z.reshape(3, 1), hx, H, self.R_gps)

    def update_gyro(self, z_theta: float):
        """Absolute heading measurement (rad)."""
        z = np.array([[wrap_angle(z_theta)]])
        hx = np.array([[wrap_angle(self.x[2, 0])]])
        H = np.array([[0, 0, 1]])
        self._generic_update(z, hx, H, self.R_gyro)

    def update_accel(self, acc_world: np.ndarray):
        """
        acc_world : 2‑vector linear acceleration in **world** frame.
        Here we treat it as noisy direct measurement of (ẍ, ÿ) = 0,
        which constrains vx, vy drift in a 3‑state filter only indirectly.
        Feel free to skip or expand to a 5‑ or 6‑state model.
        """
        # In a [x,y,theta] filter we *could* skip accel or expand the state to velocities.
        pass  # <<< optional – extend as needed

    # ------------------------------------------------------------------
    #  I N T E R N A L S
    # ------------------------------------------------------------------
    def _generic_update(self, z, hx, H, R):
        """Shared EKF measurement update."""
        y = z - hx  # residual
        y[2:3] = wrap_angle(y[2:3])  # keep angles small if present
        S = H @ self.P @ H.T + R  # innovation cov
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        self.x += K @ y  # mean update
        self.x[2, 0] = wrap_angle(self.x[2, 0])
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ H) @ self.P  # Joseph form optional

    def _wheel_deltas_to_body_twist(self, v: np.ndarray, dt: float) -> np.ndarray:
        """
        Convert wheel *angle increments* to body‑frame twist (vx, vy, ω).
        Equation for 45° omni wheels (x‑drive / mecanum‑coefficient matrix).

        dphi : 4‑vector wheel angle increments (rad) over dt
        return: 3‑vector [vx_body, vy_body, omega] (units m/s, m/s, rad/s)
        """
        if dt <= 0:
            return np.zeros(3)

        # Kinematic matrix (r/4 already applied in v)
        #   [ +1  +1  +1  +1 ]      -- vx component
        #   [ -1  +1  +1  -1 ]      -- vy component
        #   [ -1  +1  -1  +1 ]/ (L+W) -- ω component  (signs depend on wheel orientation)
        vx = (v[0] + v[1] + v[2] + v[3]) / 4.0
        vy = (-v[0] + v[1] + v[2] - v[3]) / 4.0  # check sign convention!  # <<< TODO
        omega = (-v[0] + v[1] - v[2] + v[3]) / (4.0 * (self.L + self.W))

        return np.array([vx, vy, omega])

    # ------------------------------------------------------------------
    #  H E L P E R S
    # ------------------------------------------------------------------
    def set_state(self, x, P=None):
        """Force filter to a known state (e.g. at match start)."""
        self.x = np.asarray(x, dtype=float).reshape(3, 1)
        if P is not None:
            self.P = np.asarray(P, dtype=float)

    def get_state(self):
        return self.x.flatten(), self.P.copy()

    def update_pose(
        self,
        wheel_v: np.ndarray,
        gyro: float,
        gps1_reading: np.ndarray,
        gps2_reading: np.ndarray,
    ):
        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        gyro_delta_rad = wrap_angle(gyro - self.prev_gyro)
        self.prev_gyro = gyro

        self.predict(wheel_v, gyro_delta_rad, dt)
        if gps1_reading is not None:
            self.update_gps(0, gps1_reading)
        if gps2_reading is not None:
            self.update_gps(1, gps2_reading)

        #print(f"Pose: " f"{self.get_state()[0]}")
        return self.get_state()
