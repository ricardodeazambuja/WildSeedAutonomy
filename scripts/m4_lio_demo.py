#!/usr/bin/env python3
"""M4 LIO demo (in sim) — drive the Husky, record KISS-ICP + ego_localizer vs truth.

The lidar twin of m3_vio_demo.py, run INSIDE a container on the sim's ROS graph
(use_sim_time). Drives the SAME curved path as the M3 chart while recording, on the
same clock:
  - GROUND TRUTH : gz model pose, bridged from /world/<world>/dynamic_pose/info
  - KISS-ICP     : /kiss/odometry (raw lidar odometry, its own frame — for reference)
  - OpenVINS     : /odomimu (raw stereo VIO) — OPTIONAL; recorded when the vio
                   profile is up, so the M3-vs-M4 A/B comes from ONE drive.
  - ego_localizer: /ego_localizer/odom (LIO fused with the IMU yaw-rate — the M4 output;
                   start it with config/ego_localizer_lidar.yaml)

Writes headerless x,y,z CSVs (eval_tools.evaluate inputs) + a combined CSV, and prints
ATE (Umeyama-aligned, so each estimator's arbitrary frame doesn't matter).

Needs up: husky sim, kissicp (deploy.sh up lio), the dynamic_pose bridge, and
ego_localizer in the lidar config; optionally OpenVINS (deploy.sh up vio) for the A/B.

Usage: m4_lio_demo.py <out_prefix> [v] [wz] [secs]

All durations (secs, the jerk-start, the CSV `t` column) are SIM seconds — at low
RTF the demo takes proportionally longer in wall time but the drive path, accel
transients, and recorded physics are identical. Measured RTF printed at start.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

NS = "/a200_0000"
# gz world name: 'pipeline' or, for external bundles, whatever `deploy.sh
# world` wrote to SIM_WORLD_NAME (the gtbridge topic is derived from it).
WORLD = os.environ.get("SIM_WORLD_NAME", "pipeline")

# ── sim-time helpers ─────────────────────────────────────────────────────────
# DUPLICATED in: scripts/m3_vio_demo.py, scripts/m4_lio_demo.py,
# scripts/gps_denied_demo.py, scripts/n1_drive.py,
# ros2_ws/src/sensing_bringup/scripts/m3_smoke.py, .../m4_smoke.py
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


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "/results/m4"
    V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    WZ = float(sys.argv[3]) if len(sys.argv) > 3 else 0.12
    SECS = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0

    rclpy.init()
    n = rclpy.create_node("m4_lio_demo",
                          parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    drive = n.create_publisher(TwistStamped, f"{NS}/joy_teleop/cmd_vel", 10)
    st = {"gt": None, "kiss": None, "ov": None, "ego": None}
    # GT: first transform in the dynamic-pose vector is the robot model root.
    n.create_subscription(TFMessage, f"/world/{WORLD}/dynamic_pose/info",
                          lambda m: st.__setitem__("gt", (m.transforms[0].transform.translation.x,
                                                          m.transforms[0].transform.translation.y))
                          if m.transforms else None, 10)
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    be = QoSProfile(depth=20); be.reliability = ReliabilityPolicy.BEST_EFFORT
    n.create_subscription(Odometry, "/kiss/odometry",
                          lambda m: st.__setitem__("kiss", (m.pose.pose.position.x,
                                                            m.pose.pose.position.y)), be)
    n.create_subscription(Odometry, "/odomimu",
                          lambda m: st.__setitem__("ov", (m.pose.pose.position.x,
                                                          m.pose.pose.position.y)), be)
    n.create_subscription(Odometry, "/ego_localizer/odom",
                          lambda m: st.__setitem__("ego", (m.pose.pose.position.x,
                                                           m.pose.pose.position.y)), 10)

    def pub_vel(vx, wz=0.0):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "base_link"; m.twist.linear.x = float(vx); m.twist.angular.z = float(wz)
        drive.publish(m)

    # JERK-START (kept from m3_vio_demo): KISS-ICP itself needs no init transient,
    # but when the vio profile is up for the A/B, OpenVINS' static init only fires
    # on an accel jerk — so the shared drive keeps it. Harmless for LIO-only runs.
    wait_for_clock(n)
    rtf = measure_rtf(n)
    for _ in sim_window(n, 2.0, rtf):          # sit still: gravity for static init
        pub_vel(0.0)
    for dur, vx in ((0.4, 1.2), (0.4, 0.0), (0.4, 1.2), (0.5, 0.0)):
        for _ in sim_window(n, dur, rtf):      # SIM-s jabs: real accel transient at any RTF
            pub_vel(vx)

    # wait for the REQUIRED streams (gt, kiss, ego); ov is optional — give it the
    # same window, then record it only if it showed up (vio profile not up ⇒ no A/B).
    for _ in sim_window(n, 30.0, rtf):
        if None not in (st["gt"], st["kiss"], st["ego"]):
            break
        pub_vel(0.3)                           # window's spin(0.02) keeps this ~50 Hz
    missing = [k for k in ("gt", "kiss", "ego") if st[k] is None]
    if missing:
        print(f"FAIL: missing streams {missing} after 30 sim-s at RTF≈{rtf:.2f} "
              "(sim/kissicp/ego up? ego on the lidar config?)", file=sys.stderr)
        return 1
    with_ov = st["ov"] is not None
    if not with_ov:
        print("(no /odomimu — vio profile down; recording LIO-only, no in-run A/B)")

    rows = []
    next_row = -1.0
    for el in sim_window(n, SECS, rtf):
        # spin_once(0.02) BOTH paces the publisher at ~50 Hz wall and processes
        # callbacks (the twist_mux stutter lesson — see m3_vio_demo.py).
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "base_link"; m.twist.linear.x = V; m.twist.angular.z = WZ
        drive.publish(m)
        # decimate rows to one per 0.02 SIM-s (comparable row counts at any RTF)
        if el >= next_row:
            ov = st["ov"] if with_ov else (float("nan"), float("nan"))
            rows.append((el, *st["gt"], *st["kiss"], *ov, *st["ego"]))
            next_row = el + 0.02
    # stop (zero commands for 0.5 sim-s — the diff_drive ramp-down)
    for _ in sim_window(n, 0.5, rtf):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        drive.publish(m)

    # write the combined CSV + the headerless x,y,z trajectory files for eval_tools
    with open(f"{prefix}_lio.csv", "w") as f:
        f.write("t,gt_x,gt_y,kiss_x,kiss_y,ov_x,ov_y,ego_x,ego_y\n")
        for r in rows:
            f.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)
    streams = {"gt": (1, 2), "kiss": (3, 4), "ego": (7, 8)}
    if with_ov:
        streams["ov"] = (5, 6)
    for name, (ix, iy) in streams.items():
        with open(f"{prefix}_{name}.csv", "w") as f:
            for r in rows:
                f.write("%.4f,%.4f,0.0\n" % (r[ix], r[iy]))

    # quick ATE (Umeyama-aligned) so the run self-reports without a second tool
    try:
        sys.path.insert(0, "/ros2_ws/src/eval_tools")
        import numpy as np
        from eval_tools.metrics import ate
        gt = np.array([[r[1], r[2], 0.0] for r in rows])
        kiss = np.array([[r[3], r[4], 0.0] for r in rows])
        ego = np.array([[r[7], r[8], 0.0] for r in rows])
        a_kiss = ate(kiss, gt)[0]       # ate() -> (ErrorStats, aligned); rigid SE(3) align
        a_ego = ate(ego, gt)[0]
        gt_len = float(np.sum(np.linalg.norm(np.diff(gt, axis=0), axis=1)))
        line = (f"rows={len(rows)}  rtf≈{rtf:.2f}  gt_path={gt_len:.2f}m  "
                f"ATE(kiss_raw)={a_kiss.rmse:.3f}m  ATE(ego_localizer)={a_ego.rmse:.3f}m")
        if with_ov:
            ov = np.array([[r[5], r[6], 0.0] for r in rows])
            line += f"  ATE(openvins_raw)={ate(ov, gt)[0].rmse:.3f}m"
        print(line)
    except Exception as e:                                  # pragma: no cover
        print(f"rows={len(rows)} (ATE calc skipped: {e})")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
