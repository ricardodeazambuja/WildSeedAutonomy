"""Factor-graph (GTSAM/ISAM2) variant of the planar estimator — PLAN §12 M6.

Same measurement API as `ego_localizer`'s `PlanarPoseEstimator` (the EKF twin),
so the two backends are A/B-able on identical measurement streams:

    seed_pose / predict / imu_rate_update / odom_twist_update /
    visual_delta_update / lidar_delta_update / gnss_update / heading_update /
    state / covariance

Where the EKF converts relative motion into world-velocity pseudo-measurements
(its state carries velocities), a factor graph consumes them in their *native*
form: every body-frame increment is a `BetweenFactorPose2` between consecutive
pose nodes, GNSS/heading are partial unary priors on the newest node, and the
integrated IMU yaw-rate becomes a second, yaw-only between factor per interval
(a poor-man's preintegrated gyro). ISAM2 re-linearises incrementally, so each
update also *smooths the past* — the structural difference vs filtering that
the M6 A/B table quantifies (accuracy vs per-update cost).

Timekeeping: like the EKF twin, the caller drives time exclusively through
`predict(dt)` (node.py calls it before every measurement callback; the tests
and the A/B replay do the same). One pending-interval clock accumulates dt;
between factors span it, the gyro integral marks its own progress within it.

gtsam is an optional dependency (PyPI wheel `gtsam==4.2.1`; the apt
`ros-jazzy-gtsam` ships C++ only): importing this module without it raises
with a hint, while the rest of `fusion_core` stays dependency-light.

State convention matches the EKF twin: `state` -> [px, py, yaw, vx, vy, wz]
(velocities are finite-differenced across the last between interval; a pose
graph has no velocity variables).
"""
from __future__ import annotations

import math

import numpy as np

try:
    import gtsam
except ImportError as e:                                    # pragma: no cover
    raise ImportError(
        "fusion_core.factor_graph needs the 'gtsam' python wheel "
        "(pip install gtsam==4.2.1); apt ros-jazzy-gtsam has no python bindings"
    ) from e


def _X(i: int) -> int:
    return gtsam.symbol('x', i)


def _diag(*sigmas: float):
    return gtsam.noiseModel.Diagonal.Sigmas(np.array(sigmas, dtype=float))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


LOOSE = 1e3          # sigma for the unconstrained rows of a partial prior


class PlanarFactorGraph:
    """ISAM2 pose graph behind the PlanarPoseEstimator interface."""

    def __init__(self, sigma_a=1.0, sigma_alpha=1.0,
                 p0_pos=1.0, p0_vel=1.0):
        # sigma_a / sigma_alpha accepted for interface parity only: a pose
        # graph has no process model — motion information enters purely
        # through the between factors (and their sigmas).
        self.isam = gtsam.ISAM2()
        self._k = 0                     # newest pose-node index
        self._initialised = False
        self._pose = gtsam.Pose2(0.0, 0.0, 0.0)   # newest estimate (cache)
        self._pend_t = 0.0              # predict-time since the newest node
        self._gyro_dyaw = 0.0           # integrated IMU yaw-rate over the interval
        self._gyro_mark = 0.0           # pending-clock position of the last sample
        self._sigma_gyro = 0.02
        self._last_vel = np.zeros(3)    # [vx, vy, wz] world, finite-differenced
        self._p0 = (float(p0_pos), float(p0_vel))

    # ── lifecycle ────────────────────────────────────────────────────────────
    @property
    def state(self) -> np.ndarray:
        p = self._pose
        return np.array([p.x(), p.y(), p.theta(), *self._last_vel])

    @property
    def covariance(self) -> np.ndarray:
        """6x6 to match the EKF twin; the pose block is the graph marginal,
        the velocity block a placeholder diag (no velocity variables)."""
        P = np.diag([self._p0[0]] * 3 + [self._p0[1]] * 3).astype(float)
        if self._initialised:
            P[:3, :3] = self.isam.marginalCovariance(_X(self._k))
        return P

    def seed_pose(self, px, py, yaw):
        g = gtsam.NonlinearFactorGraph()
        v = gtsam.Values()
        pose = gtsam.Pose2(float(px), float(py), float(yaw))
        g.add(gtsam.PriorFactorPose2(
            _X(0), pose, _diag(self._p0[0], self._p0[0], self._p0[0])))
        v.insert(_X(0), pose)
        self.isam.update(g, v)
        self._pose = pose
        self._k = 0
        self._initialised = True

    # ── predict ──────────────────────────────────────────────────────────────
    def predict(self, dt: float):
        """No propagation (no process model) — advance the pending-interval
        clock that scales between-factor noise and spans displacements."""
        if dt > 0:
            self._pend_t += dt

    # ── measurement updates (EKF-twin API) ───────────────────────────────────
    def imu_rate_update(self, wz, sigma_wz=0.02):
        """Integrate the gyro yaw-rate over its share of the pending interval;
        emitted as a yaw-only between factor when the next node is created."""
        if not self._initialised:
            return
        self._gyro_dyaw += float(wz) * max(self._pend_t - self._gyro_mark, 0.0)
        self._gyro_mark = self._pend_t
        self._sigma_gyro = float(sigma_wz)

    def odom_twist_update(self, v_body, wz, sigma_v=0.05, sigma_wz=0.02):
        """Wheel-odom twist -> body-frame displacement over the pending interval."""
        if not self._initialised:
            return
        dt = self._pend_t
        if dt <= 0:
            return
        self._new_node(gtsam.Pose2(float(v_body) * dt, 0.0, float(wz) * dt),
                       dt, sigma_v, sigma_wz)

    def visual_delta_update(self, dx_body, dy_body, dyaw, dt,
                            sigma_v=0.05, sigma_wz=0.02):
        """VIO body-frame increment -> BetweenFactorPose2 (its native form)."""
        if not self._initialised or dt <= 0:
            return
        self._new_node(gtsam.Pose2(float(dx_body), float(dy_body), float(dyaw)),
                       dt, sigma_v, sigma_wz)

    def lidar_delta_update(self, dx_body, dy_body, dyaw, dt,
                           sigma_v=0.05, sigma_wz=0.02):
        """Lidar-odometry increment — identical native form (M4 hook parity)."""
        return self.visual_delta_update(dx_body, dy_body, dyaw, dt,
                                        sigma_v=sigma_v, sigma_wz=sigma_wz)

    def gnss_update(self, east, north, sigma_xy=0.5):
        """Absolute position fix: partial prior (yaw row loose) on the newest node."""
        if not self._initialised:
            return
        self._unary(gtsam.Pose2(float(east), float(north), self._pose.theta()),
                    _diag(sigma_xy, sigma_xy, LOOSE))

    def heading_update(self, yaw, sigma_yaw=0.1):
        """Absolute heading (GPS course): partial prior (position rows loose)."""
        if not self._initialised:
            return
        self._unary(gtsam.Pose2(self._pose.x(), self._pose.y(), float(yaw)),
                    _diag(LOOSE, LOOSE, sigma_yaw))

    # ── internals ────────────────────────────────────────────────────────────
    def _estimate(self, key: int) -> gtsam.Pose2:
        return self.isam.calculateEstimate().atPose2(key)

    def _new_node(self, delta: gtsam.Pose2, dt: float,
                  sigma_v: float, sigma_wz: float) -> None:
        """Append node X(k+1) tied to X(k) by `delta`; fold in the integrated
        gyro yaw as a second, yaw-only between factor; solve incrementally."""
        g = gtsam.NonlinearFactorGraph()
        v = gtsam.Values()
        k, k1 = self._k, self._k + 1
        g.add(gtsam.BetweenFactorPose2(
            _X(k), _X(k1), delta,
            _diag(max(sigma_v * dt, 1e-4), max(sigma_v * dt, 1e-4),
                  max(sigma_wz * dt, 1e-4))))
        if self._gyro_mark > 0.0:       # gyro samples arrived this interval
            g.add(gtsam.BetweenFactorPose2(
                _X(k), _X(k1), gtsam.Pose2(0.0, 0.0, self._gyro_dyaw),
                _diag(LOOSE, LOOSE, max(self._sigma_gyro * self._gyro_mark, 1e-4))))
        v.insert(_X(k1), self._pose.compose(delta))
        self.isam.update(g, v)
        self._k = k1
        prev = self._pose
        self._pose = self._estimate(_X(k1))
        span = max(self._pend_t, 1e-6)
        self._last_vel = np.array([(self._pose.x() - prev.x()) / span,
                                   (self._pose.y() - prev.y()) / span,
                                   _wrap(self._pose.theta() - prev.theta()) / span])
        self._pend_t = 0.0
        self._gyro_dyaw = 0.0
        self._gyro_mark = 0.0

    def _unary(self, pose: gtsam.Pose2, noise) -> None:
        g = gtsam.NonlinearFactorGraph()
        g.add(gtsam.PriorFactorPose2(_X(self._k), pose, noise))
        self.isam.update(g, gtsam.Values())
        self._pose = self._estimate(_X(self._k))
