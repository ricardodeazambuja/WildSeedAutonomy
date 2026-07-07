#!/usr/bin/env python3
"""M6 A/B benchmark — hand-rolled EKF vs GTSAM factor graph, same streams.

Feeds IDENTICAL measurement streams (same RNG seeds) through the two planar
backends that share one interface:
  - EKF   : ego_localizer.estimator.PlanarPoseEstimator (fusion_core.EKF inside)
  - GTSAM : fusion_core.factor_graph.PlanarFactorGraph  (ISAM2 pose graph)

Scenarios (mirror the project's milestones, not toy cases):
  keystone : biased wheel-odom twist + IMU yaw-rate + 1 Hz GNSS with a
             denial window (the M5 drift->reacquire scenario).
  frontend : VIO/LIO-style body-frame deltas (arbitrary frontend frame,
             slow random-walk drift) + IMU yaw-rate, no GNSS (M3/M4).

Reports position/yaw RMSE vs truth and per-update wall time (mean / p95), as
a markdown table + CSV. Run INSIDE the fusion image (has gtsam + numpy<2):

  python3 m6_ab_benchmark.py /results/m6_ab
"""
import math
import sys
import time

import numpy as np

sys.path.insert(0, "/ros2_ws/src/fusion_core")
sys.path.insert(0, "/ros2_ws/src/ego_localizer")

from ego_localizer.estimator import PlanarPoseEstimator, wrap  # noqa: E402
from fusion_core.factor_graph import PlanarFactorGraph         # noqa: E402


def run_keystone(est, seed=11):
    """M5 scenario: odom twist (biased) + IMU rate + 1 Hz GNSS with denial."""
    rng = np.random.default_rng(seed)
    dt, v, wz, v_bias, gps_sigma = 0.05, 1.0, 0.15, 0.12, 0.2
    px = py = yaw = 0.0
    est.seed_pose(0.0, 0.0, 0.0)
    errs, yaw_errs, times = [], [], []
    phases = [(300, True), (400, False), (300, True)]
    phase_err = []
    for steps, gps_on in phases:
        tail = []
        for k in range(steps):
            px += v * math.cos(yaw) * dt
            py += v * math.sin(yaw) * dt
            yaw = wrap(yaw + wz * dt)
            t0 = time.perf_counter()
            est.predict(dt)
            est.imu_rate_update(wz + rng.normal(scale=0.01), sigma_wz=0.01)
            est.odom_twist_update((v + v_bias) + rng.normal(scale=0.02),
                                  wz + rng.normal(scale=0.01))
            if gps_on and k % 20 == 0:
                est.gnss_update(px + rng.normal(scale=gps_sigma),
                                py + rng.normal(scale=gps_sigma),
                                sigma_xy=gps_sigma)
            times.append(time.perf_counter() - t0)
            e = math.hypot(est.state[0] - px, est.state[1] - py)
            errs.append(e)
            yaw_errs.append(abs(wrap(est.state[2] - yaw)))
            if k >= steps - 120:
                tail.append(e)
        phase_err.append(float(np.mean(tail)))
    return errs, yaw_errs, times, {"on": phase_err[0], "denied": phase_err[1],
                                   "reacq": phase_err[2]}


def run_frontend(est, seed=7):
    """M3/M4 scenario: body-frame deltas from a drifting frontend frame + IMU."""
    rng = np.random.default_rng(seed)
    dt, v, wz = 0.1, 0.8, 0.2
    theta_off, tx, ty = 0.7, 5.0, -3.0

    def frontend_of(px, py, yaw, dx, dy, dyaw):
        X = tx + math.cos(theta_off) * px - math.sin(theta_off) * py + dx
        Y = ty + math.sin(theta_off) * px + math.cos(theta_off) * py + dy
        return X, Y, wrap(yaw + theta_off + dyaw)

    px = py = yaw = 0.0
    est.seed_pose(0.0, 0.0, 0.0)
    drift = np.zeros(3)
    Xp, Yp, Yawp = frontend_of(0, 0, 0, 0, 0, 0)
    errs, yaw_errs, times = [], [], []
    for _ in range(400):
        px += v * math.cos(yaw) * dt
        py += v * math.sin(yaw) * dt
        yaw = wrap(yaw + wz * dt)
        drift += rng.normal(scale=[0.002, 0.002, 0.001])
        X, Y, Yaw = frontend_of(px, py, yaw, *drift)
        dX, dY = X - Xp, Y - Yp
        c, s = math.cos(Yawp), math.sin(Yawp)
        dxb, dyb = c * dX + s * dY, -s * dX + c * dY
        dyaw = wrap(Yaw - Yawp)
        Xp, Yp, Yawp = X, Y, Yaw
        t0 = time.perf_counter()
        est.predict(dt)
        est.imu_rate_update(wz + rng.normal(scale=0.01), sigma_wz=0.01)
        est.lidar_delta_update(dxb, dyb, dyaw, dt)
        times.append(time.perf_counter() - t0)
        errs.append(math.hypot(est.state[0] - px, est.state[1] - py))
        yaw_errs.append(abs(wrap(est.state[2] - yaw)))
    return errs, yaw_errs, times, {}


def stats(errs, yaw_errs, times):
    return dict(
        pos_rmse=math.sqrt(float(np.mean(np.square(errs)))),
        pos_max=float(np.max(errs)),
        yaw_rmse=math.sqrt(float(np.mean(np.square(yaw_errs)))),
        upd_mean_us=1e6 * float(np.mean(times)),
        upd_p95_us=1e6 * float(np.percentile(times, 95)),
        upd_total_ms=1e3 * float(np.sum(times)),
    )


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "/results/m6_ab"
    backends = {"EKF (hand-rolled)": PlanarPoseEstimator,
                "GTSAM ISAM2 (factor graph)": PlanarFactorGraph}
    scenarios = {"keystone (odom+IMU+GNSS, denial window)": run_keystone,
                 "frontend (LIO/VIO deltas + IMU, no GNSS)": run_frontend}

    rows = []
    for sc_name, sc in scenarios.items():
        for be_name, Be in backends.items():
            errs, yerrs, times, extra = sc(Be())
            st = stats(errs, yerrs, times)
            st.update(scenario=sc_name, backend=be_name, **extra)
            rows.append(st)
            print(f"{sc_name:45s} {be_name:28s} "
                  f"pos_rmse={st['pos_rmse']:.3f}m yaw_rmse={st['yaw_rmse']:.4f}rad "
                  f"upd={st['upd_mean_us']:.0f}us (p95 {st['upd_p95_us']:.0f})"
                  + (f" on/denied/reacq={extra['on']:.2f}/{extra['denied']:.2f}/"
                     f"{extra['reacq']:.2f}m" if extra else ""))

    cols = ["scenario", "backend", "pos_rmse", "pos_max", "yaw_rmse",
            "upd_mean_us", "upd_p95_us", "upd_total_ms", "on", "denied", "reacq"]
    with open(prefix + ".csv", "w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(c, "")) for c in cols) + "\n")

    with open(prefix + ".md", "w") as f:
        f.write("| scenario | backend | pos RMSE [m] | yaw RMSE [rad] | "
                "update mean [µs] | update p95 [µs] |\n|---|---|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r['scenario']} | {r['backend']} | {r['pos_rmse']:.3f} | "
                    f"{r['yaw_rmse']:.4f} | {r['upd_mean_us']:.0f} | "
                    f"{r['upd_p95_us']:.0f} |\n")
    print(f"wrote {prefix}.csv and {prefix}.md")


if __name__ == "__main__":
    main()
