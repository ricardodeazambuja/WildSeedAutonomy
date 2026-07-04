"""Planar pose estimator — the ROS-free brain of `ego_localizer` (PLAN §5).

Wraps `fusion_core.EKF` with a concrete planar state and the IMU / wheel-odom
measurement models. Kept ROS-free so it unit-tests deterministically in
milliseconds (the node.py wrapper only does ROS plumbing). Later milestones add
lidar/visual relative updates (M3/M4) and GNSS absolute updates (M5) — each is
just another `*_update` here + a subscription in node.py, no change to this core.

State (odom frame):
    x = [px, py, yaw, vx, vy, wz]
i.e. positions [px, py, yaw] then velocities [vx, vy, wz], which is exactly the
constant-velocity block layout `fusion_core.models.cv_transition(dt, dim=3)`
produces (pos += vel·dt). Predict is therefore a linear CV model; measurement
yaw innovations are wrapped to (-pi, pi].
"""
from __future__ import annotations

import math

import numpy as np

from fusion_core.ekf import EKF
from fusion_core.models import cv_transition

YAW = 2


def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _process_noise(dt: float, sigma_a: float, sigma_alpha: float) -> np.ndarray:
    """White-noise-acceleration Q for the [px,py,yaw,vx,vy,wz] state.

    Per axis i the (pos_i, vel_i) block is sigma_i²·[[dt⁴/4, dt³/2],[dt³/2, dt²]];
    linear axes (x,y) use sigma_a, the yaw axis uses sigma_alpha.
    """
    s2 = np.array([sigma_a, sigma_a, sigma_alpha]) ** 2
    Q = np.zeros((6, 6))
    for i in range(3):
        Q[i, i] = s2[i] * dt ** 4 / 4.0
        Q[i, i + 3] = Q[i + 3, i] = s2[i] * dt ** 3 / 2.0
        Q[i + 3, i + 3] = s2[i] * dt ** 2
    return Q


class PlanarPoseEstimator:
    def __init__(self, sigma_a=1.0, sigma_alpha=1.0,
                 p0_pos=1.0, p0_vel=1.0):
        self.sigma_a = float(sigma_a)
        self.sigma_alpha = float(sigma_alpha)
        P0 = np.diag([p0_pos, p0_pos, p0_pos, p0_vel, p0_vel, p0_vel]) ** 1.0
        self.ekf = EKF(x0=np.zeros(6), P0=P0)
        self._initialised = False

    # ── lifecycle ────────────────────────────────────────────────────────────
    @property
    def state(self) -> np.ndarray:
        return self.ekf.x.copy()

    @property
    def covariance(self) -> np.ndarray:
        return self.ekf.P.copy()

    def seed_pose(self, px, py, yaw):
        """Initialise position/heading from the first absolute (odom) fix."""
        x = self.ekf.x.copy()
        x[0], x[1], x[2] = px, py, wrap(yaw)
        self.ekf.x = x
        self._initialised = True

    # ── predict ──────────────────────────────────────────────────────────────
    def predict(self, dt: float):
        if dt <= 0:
            return
        F = cv_transition(dt, dim=3)
        Q = _process_noise(dt, self.sigma_a, self.sigma_alpha)
        self.ekf.predict(F, Q)
        self.ekf.x[YAW] = wrap(self.ekf.x[YAW])

    # ── measurement updates ────────────────────────────────────────────────────
    def imu_update(self, yaw, wz, sigma_yaw=0.05, sigma_wz=0.02):
        """Fuse IMU heading + yaw-rate. H selects [yaw, wz] (rows 2,5)."""
        H = np.zeros((2, 6)); H[0, YAW] = 1.0; H[1, 5] = 1.0
        R = np.diag([sigma_yaw, sigma_wz]) ** 2
        z = np.array([yaw, wz], dtype=float)
        z_pred = H @ self.ekf.x
        z[0] = z_pred[0] + wrap(z[0] - z_pred[0])     # wrap yaw innovation
        return self.ekf.update(z, H, R, z_pred=z_pred)

    def odom_update(self, px, py, yaw, sigma_xy=0.1, sigma_yaw=0.1):
        """Fuse wheel-odom *pose* (px,py,yaw) as ABSOLUTE. H selects [px,py,yaw].

        Used when wheel odom is the trusted local reference (M3 foundation). For
        the GPS-denied keystone use `odom_twist_update` instead, so odom is a
        relative source that drifts and GNSS bounds it.
        """
        H = np.zeros((3, 6)); H[0, 0] = H[1, 1] = H[2, YAW] = 1.0
        R = np.diag([sigma_xy, sigma_xy, sigma_yaw]) ** 2
        z = np.array([px, py, yaw], dtype=float)
        z_pred = H @ self.ekf.x
        z[2] = z_pred[2] + wrap(z[2] - z_pred[2])     # wrap yaw innovation
        return self.ekf.update(z, H, R, z_pred=z_pred)

    def odom_twist_update(self, v_body, wz, sigma_v=0.05, sigma_wz=0.02):
        """Fuse wheel-odom *twist* as RELATIVE motion (PLAN §5: odom is relative).

        Rotates the body forward speed into the world frame using the current yaw
        estimate (a mild EKF linearisation) and feeds it as a world-velocity
        pseudo-measurement on [vx,vy,wz]. Integrating this dead-reckons → drifts
        without an absolute fix, which is exactly what GNSS then bounds.
        """
        yaw = self.ekf.x[YAW]
        H = np.zeros((3, 6)); H[0, 3] = H[1, 4] = H[2, 5] = 1.0
        R = np.diag([sigma_v, sigma_v, sigma_wz]) ** 2
        z = np.array([v_body * math.cos(yaw), v_body * math.sin(yaw), wz])
        return self.ekf.update(z, H, R)

    def visual_delta_update(self, dx_body, dy_body, dyaw, dt,
                            sigma_v=0.05, sigma_wz=0.02):
        """Fuse a VIO motion increment as a world-velocity pseudo-measurement (M3).

        OpenVINS emits an absolute 6-DoF pose in its OWN frame — arbitrary origin,
        slowly drifting. We deliberately use only the *increment* between consecutive
        VIO frames, expressed in the body frame (dx forward, dy left, dyaw over dt):
        a body-frame delta is invariant to the VIO frame's unknown origin/orientation,
        so that offset cancels and never pollutes the EKF (loosely-coupled fusion).
        Rotating the delta by the EKF's *current* yaw gives a world velocity on
        [vx,vy,wz]; integrating it dead-reckons — which an absolute source (GNSS, M5)
        then bounds. Same shape as `odom_twist_update`, generalized to a 2-D body
        displacement (VIO sees sideways motion; pure wheel odom doesn't).
        """
        if dt <= 0:
            return None
        yaw = self.ekf.x[YAW]
        c, s = math.cos(yaw), math.sin(yaw)
        vx = (c * dx_body - s * dy_body) / dt          # body→world rotation (REP-103)
        vy = (s * dx_body + c * dy_body) / dt
        H = np.zeros((3, 6)); H[0, 3] = H[1, 4] = H[2, 5] = 1.0
        R = np.diag([sigma_v, sigma_v, sigma_wz]) ** 2
        z = np.array([vx, vy, dyaw / dt])
        return self.ekf.update(z, H, R)

    def imu_rate_update(self, wz, sigma_wz=0.02):
        """Fuse IMU yaw-RATE only (row 5). Frame-independent — use in relative mode
        where the IMU's absolute yaw is in the gz world frame, not the GPS ENU frame
        (§17.4). Absolute heading then comes from `heading_update` (GPS course)."""
        H = np.zeros((1, 6)); H[0, 5] = 1.0
        R = np.array([[sigma_wz ** 2]])
        return self.ekf.update(np.array([wz]), H, R)

    def heading_update(self, yaw, sigma_yaw=0.1):
        """Fuse an absolute heading (row 2), innovation wrapped. The GPS
        course-over-ground feeds this to anchor yaw to the ENU frame (§17.4 fix)."""
        H = np.zeros((1, 6)); H[0, YAW] = 1.0
        R = np.array([[sigma_yaw ** 2]])
        z_pred = H @ self.ekf.x
        z = np.array([z_pred[0] + wrap(yaw - z_pred[0])])
        return self.ekf.update(z, H, R, z_pred=z_pred)

    def gnss_update(self, east, north, sigma_xy=0.5):
        """Fuse a GNSS absolute position fix (local ENU metres). H selects [px,py].

        The *droppable* absolute input (PLAN §11): gate it off and the estimate
        dead-reckons from odom+IMU; gate it back on and it snaps to the fix.
        """
        H = np.zeros((2, 6)); H[0, 0] = H[1, 1] = 1.0
        R = np.diag([sigma_xy, sigma_xy]) ** 2
        z = np.array([east, north], dtype=float)
        return self.ekf.update(z, H, R)
