"""Offline, deterministic tests for the planar pose estimator (no ROS)."""
import math

import numpy as np

from ego_localizer.estimator import PlanarPoseEstimator, wrap


def test_wrap_range():
    assert math.isclose(wrap(math.pi), math.pi)
    assert math.isclose(wrap(-math.pi + 1e-9), -math.pi + 1e-9, abs_tol=1e-6)
    assert math.isclose(wrap(3 * math.pi), math.pi, abs_tol=1e-9)
    assert math.isclose(wrap(2 * math.pi), 0.0, abs_tol=1e-9)


def test_predict_advances_position_by_velocity():
    est = PlanarPoseEstimator()
    est.ekf.x = np.array([0.0, 0.0, 0.0, 1.0, 0.5, 0.0])  # vx=1, vy=0.5
    est.predict(2.0)
    assert np.allclose(est.state[:2], [2.0, 1.0])


def test_imu_pulls_yaw_with_wrap():
    est = PlanarPoseEstimator()
    est.ekf.x[2] = 3.0                      # near +pi
    est.imu_update(yaw=-3.0, wz=0.0)        # measurement just across the wrap
    # fused yaw should move the SHORT way across ±pi, staying near the boundary,
    # not swing through ~6 rad back to 0
    assert abs(wrap(est.state[2])) > 2.5


def _drive_and_fuse(seed=3, v=1.0, wz=0.4, dt=0.05, steps=1000, odom_every=4,
                    sigma_xy=0.10, sigma_yaw_odom=0.05,
                    sigma_yaw_imu=0.02, sigma_wz=0.01):
    """Simulate a curved unicycle drive, fuse noisy odom (slow) + noisy IMU (fast).

    Returns (rmse_pos_fused, rmse_pos_odom, rmse_yaw_fused, rmse_yaw_odom),
    measured at the odom steps (where the raw odom sample exists to compare).
    """
    rng = np.random.default_rng(seed)
    est = PlanarPoseEstimator()                 # default tuning
    px = py = yaw = 0.0
    est.seed_pose(px, py, yaw)
    fp = op = fy = oy = 0.0
    n = 0
    for k in range(steps):
        px += v * math.cos(yaw) * dt
        py += v * math.sin(yaw) * dt
        yaw = wrap(yaw + wz * dt)
        est.predict(dt)
        est.imu_update(yaw + rng.normal(scale=sigma_yaw_imu),
                       wz + rng.normal(scale=sigma_wz),
                       sigma_yaw=sigma_yaw_imu, sigma_wz=sigma_wz)
        if k % odom_every == 0:
            zx = px + rng.normal(scale=sigma_xy)
            zy = py + rng.normal(scale=sigma_xy)
            zyaw = yaw + rng.normal(scale=sigma_yaw_odom)
            est.odom_update(zx, zy, zyaw, sigma_xy=sigma_xy,
                            sigma_yaw=sigma_yaw_odom)
            op += (zx - px) ** 2 + (zy - py) ** 2
            fp += (est.state[0] - px) ** 2 + (est.state[1] - py) ** 2
            oy += wrap(zyaw - yaw) ** 2
            fy += wrap(est.state[2] - yaw) ** 2
            n += 1
    return (math.sqrt(fp / n), math.sqrt(op / n),
            math.sqrt(fy / n), math.sqrt(oy / n))


def test_fused_heading_beats_odom_heading():
    """The headline fusion win: IMU (yaw + yaw-rate) sharply improves heading."""
    _, _, yaw_fused, yaw_odom = _drive_and_fuse()
    assert yaw_fused < 0.5 * yaw_odom, (yaw_fused, yaw_odom)   # ~12x in practice


def test_fused_position_tracks_truth_and_beats_noisy_odom():
    """Fused position RMSE < raw odom RMSE, and tracks truth to ~10 cm."""
    pos_fused, pos_odom, _, _ = _drive_and_fuse()
    assert pos_fused < pos_odom, (pos_fused, pos_odom)
    assert pos_fused < 0.13


def test_gps_denied_keystone_drift_and_reacquire():
    """The headline keystone: GPS on → bounded; denied → drifts; reacquire → snaps back.

    Odom is consumed as RELATIVE twist with a small speed bias (→ dead-reckon drift);
    IMU gives heading; GNSS is the droppable absolute fix.
    """
    rng = np.random.default_rng(11)
    dt = 0.05
    v, wz = 1.0, 0.15
    v_bias = 0.12                  # odom over-reports speed → drift when GPS is off
    gps_sigma = 0.2
    est = PlanarPoseEstimator(sigma_a=0.5, sigma_alpha=0.5)
    px = py = yaw = 0.0
    est.seed_pose(0.0, 0.0, 0.0)

    def phase(steps, gps_on, measure_last=120):
        """Run a phase; return MEAN position error over its final `measure_last` steps."""
        nonlocal px, py, yaw
        errs = []
        for k in range(steps):
            px += v * math.cos(yaw) * dt
            py += v * math.sin(yaw) * dt
            yaw = wrap(yaw + wz * dt)
            est.predict(dt)
            est.imu_update(yaw + rng.normal(scale=0.01), wz + rng.normal(scale=0.01),
                           sigma_yaw=0.01, sigma_wz=0.01)
            est.odom_twist_update((v + v_bias) + rng.normal(scale=0.02),
                                  wz + rng.normal(scale=0.01))
            if gps_on:
                est.gnss_update(px + rng.normal(scale=gps_sigma),
                                py + rng.normal(scale=gps_sigma), sigma_xy=gps_sigma)
            if k >= steps - measure_last:
                errs.append(math.hypot(est.state[0] - px, est.state[1] - py))
        return float(np.mean(errs))

    err_on = phase(300, gps_on=True)
    err_denied = phase(400, gps_on=False)            # ~20 s denied → clear drift
    err_reacq = phase(300, gps_on=True)

    # qualitative keystone behaviour (robust bounds, not overfit thresholds):
    assert err_on < 0.6, err_on                            # GPS bounds the error
    assert err_denied > 1.0 and err_denied > 3 * err_on, (err_denied, err_on)  # drifts
    assert err_reacq < 0.6, (err_reacq, err_denied)        # snaps back on reacquire


def test_imu_rate_and_heading_updates():
    est = PlanarPoseEstimator()
    est.seed_pose(0, 0, 0)
    est.imu_rate_update(0.5, sigma_wz=0.01)
    assert abs(est.state[5] - 0.5) < 0.1          # wz pulled toward measurement
    est.heading_update(1.0, sigma_yaw=0.05)
    assert abs(est.state[2] - 1.0) < 0.2          # yaw pulled toward measurement
    # heading innovation wraps the short way across ±pi
    est.ekf.x[2] = 3.0
    est.heading_update(-3.0, sigma_yaw=0.05)
    assert abs(wrap(est.state[2])) > 2.5


def test_gps_denied_keystone_course_aided_with_heading_offset():
    """The §17.4 fix: IMU gives yaw-RATE only; GPS course anchors absolute heading.

    The IMU's absolute yaw (off-frame vs ENU in the real sim) is NEVER fused — only
    its yaw-rate. Absolute heading comes from the 1 Hz GPS course-over-ground (ENU
    direction of travel). The keystone must still bound → drift → recover.
    """
    rng = np.random.default_rng(5)
    dt = 0.05
    v, wz = 1.0, 0.15
    v_bias = 0.12
    gps_sigma = 0.2
    course_sigma = 0.1            # 1 Hz GPS course-over-ground heading noise (rad)
    gps_every = 20               # GPS at ~1 Hz (every 20th 50 ms step)
    est = PlanarPoseEstimator(sigma_a=0.5, sigma_alpha=0.5)
    px = py = yaw = 0.0
    seeded = [False]

    def phase(steps, gps_on, measure_last=120):
        nonlocal px, py, yaw
        errs = []
        for k in range(steps):
            px += v * math.cos(yaw) * dt
            py += v * math.sin(yaw) * dt
            yaw = wrap(yaw + wz * dt)
            if seeded[0]:
                est.predict(dt)
                # IMU yaw-RATE only (absolute IMU yaw is off-frame, §17.4); odom relative
                est.imu_rate_update(wz + rng.normal(scale=0.01), sigma_wz=0.01)
                est.odom_twist_update((v + v_bias) + rng.normal(scale=0.02),
                                      wz + rng.normal(scale=0.01))
            if gps_on and k % gps_every == 0:
                gx = px + rng.normal(scale=gps_sigma)
                gy = py + rng.normal(scale=gps_sigma)
                course = yaw + rng.normal(scale=course_sigma)   # GPS course (ENU heading)
                if not seeded[0]:
                    est.seed_pose(gx, gy, course); seeded[0] = True
                    continue
                est.gnss_update(gx, gy, sigma_xy=gps_sigma)
                est.heading_update(course, sigma_yaw=course_sigma)
            if seeded[0] and k >= steps - measure_last:
                errs.append(math.hypot(est.state[0] - px, est.state[1] - py))
        return float(np.mean(errs)) if errs else float('nan')

    err_on = phase(300, gps_on=True)
    err_denied = phase(400, gps_on=False)
    err_reacq = phase(300, gps_on=True)
    # absolute IMU yaw (off-frame) is never fused — heading comes only from GPS course.
    # 1 Hz GPS → larger between-fix error than the dense-GPS test, but still bounded.
    assert err_on < 0.8, err_on                            # bounded with GPS+course
    assert err_denied > 2 * err_on, (err_denied, err_on)   # drifts clearly when denied
    assert err_reacq < err_on + 0.15, (err_reacq, err_on)  # recovers to ~on-level


def test_visual_delta_update_cancels_vio_frame_and_tracks_truth():
    """M3 loosely-coupled VIO: feeding body-frame VIO *deltas* cancels OpenVINS'
    arbitrary world frame.

    OpenVINS reports an absolute pose in its own frame (here: truth rotated by
    theta_off, translated by (tx,ty), plus a slow random-walk drift). The node uses
    only the body-frame increment between consecutive VIO poses — invariant to that
    constant SE(2) offset — so the EKF tracks truth even though the raw VIO pose is
    in the 'wrong' frame. The control assert shows that naively trusting the absolute
    VIO pose would be off by ~the frame offset.
    """
    rng = np.random.default_rng(7)
    dt = 0.1                       # ~10 Hz camera
    v, wz = 0.8, 0.2
    theta_off, tx, ty = 0.7, 5.0, -3.0     # VIO frame: rotated + translated vs truth

    def vio_of(px, py, yaw, dx, dy, dyaw):
        X = tx + math.cos(theta_off) * px - math.sin(theta_off) * py + dx
        Y = ty + math.sin(theta_off) * px + math.cos(theta_off) * py + dy
        return X, Y, wrap(yaw + theta_off + dyaw)

    est = PlanarPoseEstimator(sigma_a=0.5, sigma_alpha=0.5)
    est.seed_pose(0.0, 0.0, 0.0)               # sim starts at the odom origin
    px = py = yaw = 0.0
    drift = np.zeros(3)
    Xp, Yp, Yawp = vio_of(0, 0, 0, 0, 0, 0)
    errs = []
    for _ in range(400):
        px += v * math.cos(yaw) * dt
        py += v * math.sin(yaw) * dt
        yaw = wrap(yaw + wz * dt)
        drift += rng.normal(scale=[0.002, 0.002, 0.001])     # slow VIO drift
        X, Y, Yaw = vio_of(px, py, yaw, *drift)
        # node logic: rotate the world-frame VIO delta into the previous VIO body frame
        dX, dY = X - Xp, Y - Yp
        c, s = math.cos(Yawp), math.sin(Yawp)
        dx_body, dy_body = c * dX + s * dY, -s * dX + c * dY
        dyaw = wrap(Yaw - Yawp)
        Xp, Yp, Yawp = X, Y, Yaw
        est.predict(dt)
        est.visual_delta_update(dx_body, dy_body, dyaw, dt)
        errs.append(math.hypot(est.state[0] - px, est.state[1] - py))

    rmse = math.sqrt(float(np.mean(np.square(errs))))
    assert rmse < 0.5, rmse                                  # fused tracks truth
    assert abs(wrap(est.state[2] - yaw)) < 0.2               # heading tracks too
    naive = math.hypot(Xp - px, Yp - py)                     # raw VIO pose, wrong frame
    assert naive > 5 * rmse, (naive, rmse)                   # ...far worse → why we use deltas


def test_lidar_delta_update_matches_visual_model_and_cancels_frame():
    """M4: the lidar relative hook is the SAME measurement model as the visual one
    (the sensor-agnostic spine contract). Two estimators fed identical body-frame
    deltas — one through `visual_delta_update`, one through `lidar_delta_update` —
    must evolve identically; and, as with VIO, KISS-ICP's arbitrary odometry-frame
    offset cancels because only increments are fused."""
    rng = np.random.default_rng(13)
    dt = 0.1                       # ~10 Hz lidar
    v, wz = 0.8, 0.2
    theta_off, tx, ty = -1.1, 12.0, 4.0   # lidar-odom frame: rotated + translated

    def lio_of(px, py, yaw, dx, dy, dyaw):
        X = tx + math.cos(theta_off) * px - math.sin(theta_off) * py + dx
        Y = ty + math.sin(theta_off) * px + math.cos(theta_off) * py + dy
        return X, Y, wrap(yaw + theta_off + dyaw)

    est_l = PlanarPoseEstimator(sigma_a=0.5, sigma_alpha=0.5)
    est_v = PlanarPoseEstimator(sigma_a=0.5, sigma_alpha=0.5)
    est_l.seed_pose(0.0, 0.0, 0.0)
    est_v.seed_pose(0.0, 0.0, 0.0)
    px = py = yaw = 0.0
    drift = np.zeros(3)
    Xp, Yp, Yawp = lio_of(0, 0, 0, 0, 0, 0)
    errs = []
    for _ in range(400):
        px += v * math.cos(yaw) * dt
        py += v * math.sin(yaw) * dt
        yaw = wrap(yaw + wz * dt)
        drift += rng.normal(scale=[0.002, 0.002, 0.001])     # slow lidar-odom drift
        X, Y, Yaw = lio_of(px, py, yaw, *drift)
        dX, dY = X - Xp, Y - Yp
        c, s = math.cos(Yawp), math.sin(Yawp)
        dx_body, dy_body = c * dX + s * dY, -s * dX + c * dY
        dyaw = wrap(Yaw - Yawp)
        Xp, Yp, Yawp = X, Y, Yaw
        est_l.predict(dt)
        est_l.lidar_delta_update(dx_body, dy_body, dyaw, dt)
        est_v.predict(dt)
        est_v.visual_delta_update(dx_body, dy_body, dyaw, dt)
        errs.append(math.hypot(est_l.state[0] - px, est_l.state[1] - py))

    assert np.allclose(est_l.state, est_v.state)             # identical model
    assert np.allclose(est_l.covariance, est_v.covariance)
    rmse = math.sqrt(float(np.mean(np.square(errs))))
    assert rmse < 0.5, rmse                                  # fused tracks truth
    assert abs(wrap(est_l.state[2] - yaw)) < 0.2             # heading tracks too


def test_covariance_stays_finite_and_symmetric():
    est = PlanarPoseEstimator()
    est.seed_pose(0, 0, 0)
    for _ in range(200):
        est.predict(0.05)
        est.imu_update(0.0, 0.0)
        est.odom_update(0.0, 0.0, 0.0)
        P = est.covariance
        assert np.all(np.isfinite(P))
        assert np.allclose(P, P.T, atol=1e-10)
