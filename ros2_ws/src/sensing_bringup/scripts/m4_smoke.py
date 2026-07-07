#!/usr/bin/env python3
"""M4 smoke gate — fail-fast check that the KISS-ICP lidar pipeline is alive and sane.

The lidar twin of m3_smoke.py: a pure-lidar odometry is silently worthless if the
cloud is empty/degenerate (ICP "converges" on nothing and emits a confident-but-wrong
pose — docs/kiss-icp-failure-modes.md #1/#6), so we assert the *inputs* before
trusting any number. Run with the sim + lio stack up (`deploy.sh up compute && deploy.sh
up lio`); `deploy.sh m4-smoke` wraps both.

Checks (all must pass):
  1. CLOUD    — the Ouster topic delivers a PointCloud2 with enough finite returns
                (an empty/thin cloud means the sensor or bridge is dead).
  2. GEOMETRY — the returns have 3-D structure (spread in x/y and in z), not a
                degenerate slab ICP can't constrain (failure mode #1).
  3. LIO LIVE — KISS-ICP publishes /kiss/odometry (registration is running).
  4. LIO SANE — over a short driven window, KISS-ICP's translation agrees with the
                gz ground-truth translation within a loose factor (catches gross
                divergence/slip without overfitting a threshold).

Exit 0 = PASS, non-zero = FAIL (so it works as a CI/regression gate).

Usage (inside the fusion container):
  python3 /ros2_ws/src/sensing_bringup/scripts/m4_smoke.py
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

NS = "/a200_0000"
CLOUD = f"{NS}/sensors/lidar3d_0/points"
ODOM = "/kiss/odometry"
WORLD = os.environ.get("SIM_WORLD_NAME", "pipeline")

# Thresholds — generous margins so the gate flags real regressions, not noise.
MIN_POINTS = 2000    # finite returns per scan (tuned Ouster 512x32 -> ~16k; 2k = robust floor)
MIN_XY_STD = 2.0     # m — horizontal spread; a near-empty scene can't constrain ICP
MIN_Z_STD = 0.05     # m — vertical structure; a single flat slab is degenerate (#1)
LIO_WAIT_S = 25.0    # SIM seconds to wait for /kiss/odometry
DRIVE_S = 8.0        # SIM seconds of driven window for the sanity ratio
DRIVE_V = 0.5        # commanded forward speed (m/s)
MIN_GT_MOVE = 1.0    # m — ground truth must actually move this far in the window
RATIO_LO, RATIO_HI = 0.5, 2.0   # kiss/gt translation ratio accepted band

# ── sim-time helpers ─────────────────────────────────────────────────────────
# DUPLICATED in: scripts/m3_vio_demo.py, scripts/gps_denied_demo.py,
# scripts/n1_drive.py, ros2_ws/src/sensing_bringup/scripts/m3_smoke.py,
# ros2_ws/src/sensing_bringup/scripts/m4_smoke.py
# (different container delivery paths — no shared import; keep in sync).
#
# All experiment durations are SIM seconds: at low RTF the run takes longer on
# the wall clock but the physics (drive distance, drift windows, jerk
# transients) is identical. Wall-clock ceilings only catch a wedged sim.
RTF_FLOOR = float(os.environ.get("SIM_RTF_FLOOR", "0.02"))


def sim_now(n):
    return n.get_clock().now().nanoseconds * 1e-9


def wait_for_clock(n, wall_ceiling=120.0):
    t0 = time.time()
    while rclpy.ok() and n.get_clock().now().nanoseconds == 0:
        if time.time() - t0 > wall_ceiling:
            raise SystemExit(f"FAIL: no /clock after {wall_ceiling:.0f}s wall — sim up? "
                             "clock_bridge alive? (docs/sim-debugging-notes.md #7)")
        rclpy.spin_once(n, timeout_sec=0.1)


def measure_rtf(n, sample_wall_s=3.0):
    s0, w0 = sim_now(n), time.time()
    while time.time() - w0 < sample_wall_s:
        rclpy.spin_once(n, timeout_sec=0.05)
    rtf = max((sim_now(n) - s0) / (time.time() - w0), 1e-4)
    note = (f"  (SLOW SIM: durations are SIM seconds; wall time stretches ~{1 / rtf:.0f}x)"
            if rtf < 0.5 else "")
    print(f"[simtime] RTF≈{rtf:.3f}{note}", flush=True)
    if rtf < RTF_FLOOR:
        raise SystemExit(f"FAIL: RTF {rtf:.3f} < SIM_RTF_FLOOR {RTF_FLOOR} — sim too slow "
                         "to be meaningful (see docs/operations.md 'Slow machines / low RTF').")
    return rtf


def sim_window(n, sim_secs, rtf, tick=0.02, safety=5.0):
    """Yield sim-elapsed while < sim_secs of SIM time has passed; wall-ceiling backstop.

    Each iteration ends in spin_once(tick) — at tick=0.02 the loop also paces a
    ~50 Hz publisher (the twist_mux lesson, docs/m3-vio.md)."""
    s0, w0 = sim_now(n), time.time()
    ceiling = sim_secs / max(rtf, 1e-4) * safety + 10.0
    while rclpy.ok() and sim_now(n) - s0 < sim_secs:
        if time.time() - w0 > ceiling:
            raise SystemExit(f"FAIL: wall ceiling {ceiling:.0f}s hit inside a "
                             f"{sim_secs:g} sim-s window — RTF collapsed mid-run?")
        yield sim_now(n) - s0
        rclpy.spin_once(n, timeout_sec=tick)
# ── end sim-time helpers ─────────────────────────────────────────────────────


def cloud_xyz(msg):
    """Finite xyz returns of one scan as an (N,3) float array."""
    pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1).astype(float)
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    rclpy.init()
    n = rclpy.create_node("m4_smoke",
                          parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    be = QoSProfile(depth=5); be.reliability = ReliabilityPolicy.BEST_EFFORT

    st = {"cloud": None, "kiss": None, "gt": None}
    n.create_subscription(PointCloud2, CLOUD, lambda m: st.__setitem__("cloud", m), be)
    n.create_subscription(Odometry, ODOM, lambda m: st.__setitem__("kiss", m), be)
    n.create_subscription(TFMessage, f"/world/{WORLD}/dynamic_pose/info",
                          lambda m: st.__setitem__("gt", (m.transforms[0].transform.translation.x,
                                                          m.transforms[0].transform.translation.y))
                          if m.transforms else None, 10)
    drive = n.create_publisher(TwistStamped, f"{NS}/joy_teleop/cmd_vel", 10)

    def send(vx):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "base_link"; m.twist.linear.x = float(vx); drive.publish(m)

    fails = []

    # sim clock first: every window below is SIM-time so the gate is RTF-robust.
    wait_for_clock(n)
    rtf = measure_rtf(n)

    # --- 1+2. CLOUD arrives + has geometry (30 SIM-s grace for bring-up) ---
    for _ in sim_window(n, 30.0, rtf, tick=0.1):
        if st["cloud"] is not None:
            break
    if st["cloud"] is None:
        fails.append(f"no PointCloud2 on {CLOUD} after 30 sim-s at RTF≈{rtf:.2f} "
                     "(sim up? lidar bridged?)")
        return _report(n, fails)
    xyz = cloud_xyz(st["cloud"])
    npts = len(xyz)
    xy_std = float(xyz[:, :2].std(axis=0).mean()) if npts else 0.0
    z_std = float(xyz[:, 2].std()) if npts else 0.0
    ok = npts >= MIN_POINTS and xy_std >= MIN_XY_STD and z_std >= MIN_Z_STD
    print(f"  cloud: points={npts}  xy_std={xy_std:.1f}m  z_std={z_std:.2f}m  "
          f"[{'OK' if ok else 'FAIL'}]")
    if npts < MIN_POINTS:
        fails.append(f"thin cloud ({npts} < {MIN_POINTS} finite returns) — lidar/bridge dead?")
    else:
        if xy_std < MIN_XY_STD:
            fails.append(f"cloud has no horizontal spread (xy_std {xy_std:.1f} < {MIN_XY_STD}) "
                         "— empty scene, ICP unconstrained (#1)")
        if z_std < MIN_Z_STD:
            fails.append(f"cloud is a flat slab (z_std {z_std:.2f} < {MIN_Z_STD}) "
                         "— geometric degeneracy risk (#1)")

    # --- 3. LIO LIVE: KISS-ICP registers scans (needs a couple of sweeps) ---
    for _ in sim_window(n, LIO_WAIT_S, rtf):
        if st["kiss"] is not None:
            break
    live = st["kiss"] is not None
    print(f"  KISS-ICP {ODOM}: {'publishing' if live else 'SILENT'}  [{'OK' if live else 'FAIL'}]")
    if not live:
        fails.append(f"no {ODOM} within {LIO_WAIT_S:.0f} sim-s at RTF≈{rtf:.2f} "
                     "— kissicp container up? topic remap right?")
        return _report(n, fails)

    # --- 4. LIO SANE: drive, compare kiss vs gz-truth translation ---
    if st["gt"] is None:
        for _ in sim_window(n, 10.0, rtf, tick=0.1):
            if st["gt"] is not None:
                break
    if st["gt"] is None:
        fails.append(f"no ground truth on /world/{WORLD}/dynamic_pose/info — gtbridge up? "
                     "SIM_WORLD_NAME matches the loaded world?")
        return _report(n, fails)
    k0 = (st["kiss"].pose.pose.position.x, st["kiss"].pose.pose.position.y)
    g0 = st["gt"]
    for _ in sim_window(n, DRIVE_S, rtf):     # spin_once(0.02) paces ~50 Hz commands
        send(DRIVE_V)
    send(0.0)
    k1 = (st["kiss"].pose.pose.position.x, st["kiss"].pose.pose.position.y)
    g1 = st["gt"]
    d_kiss = math.hypot(k1[0] - k0[0], k1[1] - k0[1])
    d_gt = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
    if d_gt < MIN_GT_MOVE:
        fails.append(f"robot barely moved (gt {d_gt:.2f} m < {MIN_GT_MOVE}) — controllers up? "
                     "terrain blocking? (can't judge LIO sanity)")
    else:
        ratio = d_kiss / d_gt
        ok = RATIO_LO <= ratio <= RATIO_HI
        print(f"  LIO sanity: kiss={d_kiss:.2f}m vs gt={d_gt:.2f}m  ratio={ratio:.2f}  "
              f"[{'OK' if ok else 'FAIL'}]")
        if not ok:
            fails.append(f"KISS-ICP translation off (kiss {d_kiss:.2f} m vs gt {d_gt:.2f} m, "
                         f"ratio {ratio:.2f} outside [{RATIO_LO}, {RATIO_HI}]) — degeneracy/"
                         "divergence? see docs/kiss-icp-failure-modes.md")

    return _report(n, fails)


def _report(n, fails):
    print()
    if fails:
        print(f"M4 SMOKE: FAIL ({len(fails)} check(s))")
        for f in fails:
            print(f"  - {f}")
        rc = 1
    else:
        print("M4 SMOKE: PASS — lidar clouds are rich, KISS-ICP is live and tracks truth.")
        rc = 0
    n.destroy_node(); rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
