"""Offline, deterministic tests for the GTSAM factor-graph variant (M6).

Mirrors the behavioural tests of the EKF twin (`ego_localizer`'s
`PlanarPoseEstimator`) so the two backends are held to the same contract:
track a noisy drive, cancel a frontend's arbitrary frame via body-frame
deltas, and reproduce the GPS-denied keystone (bounded → drift → snap back).
Skips cleanly when the gtsam wheel is absent (it is a fusion-image dep, not a
host one).
"""
import math

import numpy as np
import pytest

gtsam = pytest.importorskip("gtsam")

from fusion_core.factor_graph import PlanarFactorGraph, _wrap  # noqa: E402


def test_seed_and_state_shape():
    fg = PlanarFactorGraph()
    assert not fg._initialised
    fg.seed_pose(1.0, -2.0, 0.5)
    assert fg._initialised
    s = fg.state
    assert s.shape == (6,)
    assert np.allclose(s[:3], [1.0, -2.0, 0.5])
    P = fg.covariance
    assert P.shape == (6, 6)
    assert np.all(np.isfinite(P))


def test_delta_updates_cancel_frontend_frame_and_track_truth():
    """Body-frame increments from a frontend in an arbitrary rotated/translated
    frame must still track truth (the M3/M4 loosely-coupled contract)."""
    rng = np.random.default_rng(7)
    dt = 0.1
    v, wz = 0.8, 0.2
    theta_off, tx, ty = 0.7, 5.0, -3.0

    def frontend_of(px, py, yaw, dx, dy, dyaw):
        X = tx + math.cos(theta_off) * px - math.sin(theta_off) * py + dx
        Y = ty + math.sin(theta_off) * px + math.cos(theta_off) * py + dy
        return X, Y, _wrap(yaw + theta_off + dyaw)

    fg = PlanarFactorGraph()
    fg.seed_pose(0.0, 0.0, 0.0)
    px = py = yaw = 0.0
    drift = np.zeros(3)
    Xp, Yp, Yawp = frontend_of(0, 0, 0, 0, 0, 0)
    errs = []
    for _ in range(300):
        px += v * math.cos(yaw) * dt
        py += v * math.sin(yaw) * dt
        yaw = _wrap(yaw + wz * dt)
        drift += rng.normal(scale=[0.002, 0.002, 0.001])
        X, Y, Yaw = frontend_of(px, py, yaw, *drift)
        dX, dY = X - Xp, Y - Yp
        c, s = math.cos(Yawp), math.sin(Yawp)
        dx_body, dy_body = c * dX + s * dY, -s * dX + c * dY
        dyaw = _wrap(Yaw - Yawp)
        Xp, Yp, Yawp = X, Y, Yaw
        fg.predict(dt)
        fg.imu_rate_update(wz + rng.normal(scale=0.01), sigma_wz=0.01)
        fg.lidar_delta_update(dx_body, dy_body, dyaw, dt)
        errs.append(math.hypot(fg.state[0] - px, fg.state[1] - py))

    rmse = math.sqrt(float(np.mean(np.square(errs))))
    assert rmse < 0.5, rmse
    assert abs(_wrap(fg.state[2] - yaw)) < 0.2


def test_gps_denied_keystone_drift_and_reacquire():
    """The keystone contract on the graph backend: GNSS bounds the estimate,
    denial drifts (biased odom twist), reacquisition snaps back."""
    rng = np.random.default_rng(11)
    dt = 0.05
    v, wz = 1.0, 0.15
    v_bias = 0.12
    gps_sigma = 0.2
    fg = PlanarFactorGraph()
    px = py = yaw = 0.0
    fg.seed_pose(0.0, 0.0, 0.0)

    def phase(steps, gps_on, measure_last=120):
        nonlocal px, py, yaw
        errs = []
        for k in range(steps):
            px += v * math.cos(yaw) * dt
            py += v * math.sin(yaw) * dt
            yaw = _wrap(yaw + wz * dt)
            fg.predict(dt)
            fg.imu_rate_update(wz + rng.normal(scale=0.01), sigma_wz=0.01)
            # sigma_v models the ACTUAL odom error (0.12 m/s bias + noise). A
            # smoother has no process noise to absorb an overconfident sigma
            # the way the EKF does — under-stating it here would just measure
            # a modeling error, not the backend.
            fg.odom_twist_update((v + v_bias) + rng.normal(scale=0.02),
                                 wz + rng.normal(scale=0.01), sigma_v=0.15)
            if gps_on and k % 20 == 0:      # ~1 Hz GPS
                fg.gnss_update(px + rng.normal(scale=gps_sigma),
                               py + rng.normal(scale=gps_sigma),
                               sigma_xy=gps_sigma)
            if k >= steps - measure_last:
                errs.append(math.hypot(fg.state[0] - px, fg.state[1] - py))
        return float(np.mean(errs))

    err_on = phase(300, gps_on=True)
    err_denied = phase(400, gps_on=False)
    err_reacq = phase(300, gps_on=True)

    assert err_on < 0.6, err_on
    assert err_denied > 1.0 and err_denied > 3 * err_on, (err_denied, err_on)
    assert err_reacq < 0.6, (err_reacq, err_denied)


def test_covariance_grows_without_absolute_fix_and_shrinks_with_it():
    fg = PlanarFactorGraph()
    fg.seed_pose(0, 0, 0)
    for _ in range(20):
        fg.predict(0.1)
        fg.odom_twist_update(0.5, 0.0, sigma_v=0.2, sigma_wz=0.1)
    P_dead = fg.covariance[0, 0]
    fg.gnss_update(fg.state[0], fg.state[1], sigma_xy=0.05)
    P_fixed = fg.covariance[0, 0]
    assert P_fixed < P_dead
