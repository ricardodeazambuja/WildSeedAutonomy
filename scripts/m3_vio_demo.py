#!/usr/bin/env python3
"""M3 VIO demo (in sim) — drive the Husky, record OpenVINS + ego_localizer vs truth.

Runs INSIDE a container on the sim's ROS graph (use_sim_time). Drives a curved path
while recording three trajectories sampled on the same clock:
  - GROUND TRUTH : gz model pose, bridged from /world/pipeline/dynamic_pose/info
                   (the moving entities; transforms[0] is the robot model root).
  - OpenVINS     : /odomimu  (raw VIO, in its own drifting frame — for reference)
  - ego_localizer: /ego_localizer/odom (VIO fused with the IMU yaw-rate — the M3 output)

Writes headerless x,y,z CSVs (eval_tools.evaluate inputs) + a combined CSV, and prints
ATE (Umeyama-aligned, so the arbitrary VIO/seed frame doesn't matter).

Needs up: husky sim, OpenVINS (run_subscribe_msckf), the dynamic_pose bridge, and
ego_localizer in the visual config (launch/ego_localizer_visual.launch.py).

Usage: m3_vio_demo.py <out_prefix> [v] [wz] [secs]

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
# DUPLICATED in: scripts/m3_vio_demo.py, scripts/gps_denied_demo.py,
# scripts/n1_drive.py, ros2_ws/src/sensing_bringup/scripts/m3_smoke.py
# (three different container delivery paths — no shared import; keep in sync).
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


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "/results/m3"
    V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    WZ = float(sys.argv[3]) if len(sys.argv) > 3 else 0.12
    SECS = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0

    rclpy.init()
    n = rclpy.create_node("m3_vio_demo",
                          parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    drive = n.create_publisher(TwistStamped, f"{NS}/joy_teleop/cmd_vel", 10)
    st = {"gt": None, "ov": None, "ego": None}
    # GT: first transform in the dynamic-pose vector is the robot model root.
    n.create_subscription(TFMessage, f"/world/{WORLD}/dynamic_pose/info",
                          lambda m: st.__setitem__("gt", (m.transforms[0].transform.translation.x,
                                                          m.transforms[0].transform.translation.y))
                          if m.transforms else None, 10)
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    be = QoSProfile(depth=20); be.reliability = ReliabilityPolicy.BEST_EFFORT
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

    # JERK-START: OpenVINS static init only fires on an accel "jerk" at motion onset.
    # A smooth diff-drive ramp never trips it (and dynamic init needs rotation-disparity
    # a straight start never builds), so /odomimu would never appear and the warmup below
    # would time out. Sit still ~2 s (gravity), then two sharp forward jabs give the IMU
    # the transient -> "successful initialization". Verified: fires reliably; without it
    # the demo aborts with missing streams ['ov']. See docs/m3-vio.md.
    wait_for_clock(n)
    rtf = measure_rtf(n)
    for _ in sim_window(n, 2.0, rtf):          # sit still: gravity for static init
        pub_vel(0.0)
    for dur, vx in ((0.4, 1.2), (0.4, 0.0), (0.4, 1.2), (0.5, 0.0)):
        for _ in sim_window(n, dur, rtf):      # SIM-s jabs: real accel transient at any RTF
            pub_vel(vx)

    # wait for all three streams (ego only appears once it has moved+seeded,
    # so we nudge the robot a little while waiting).
    for _ in sim_window(n, 30.0, rtf):
        if None not in (st["gt"], st["ov"], st["ego"]):
            break
        pub_vel(0.3)                           # window's spin(0.02) keeps this ~50 Hz
    missing = [k for k, v in st.items() if v is None]
    if missing:
        print(f"FAIL: missing streams {missing} after 30 sim-s at RTF≈{rtf:.2f} "
              "(sim/openvins/ego up?)", file=sys.stderr)
        return 1

    rows = []
    next_row = -1.0
    for el in sim_window(n, SECS, rtf):
        # sim_window's spin_once(0.02) BOTH paces the publisher at ~50 Hz wall and
        # processes callbacks. Do NOT switch to spin_once(0.0)+time.sleep(0.06): that
        # publishes at only ~16 Hz, twist_mux times the command out between publishes,
        # and the robot stutters to ~0.09 m/s (3.5 m over a 40 s drive) -> no
        # translation -> mono-VIO scale blows up. 50 Hz holds the commanded speed.
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "base_link"; m.twist.linear.x = V; m.twist.angular.z = WZ
        drive.publish(m)
        # decimate rows to one per 0.02 SIM-s (the pre-sim-time ~50 Hz cadence, so
        # row counts stay comparable at any RTF)
        if el >= next_row:
            rows.append((el, *st["gt"], *st["ov"], *st["ego"]))
            next_row = el + 0.02
    # stop (zero commands for 0.5 sim-s — the diff_drive ramp-down)
    for _ in sim_window(n, 0.5, rtf):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        drive.publish(m)

    # write the combined CSV + the three headerless x,y,z trajectory files for eval_tools
    with open(f"{prefix}_vio.csv", "w") as f:
        f.write("t,gt_x,gt_y,ov_x,ov_y,ego_x,ego_y\n")
        for r in rows:
            f.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)
    for name, (ix, iy) in {"gt": (1, 2), "ov": (3, 4), "ego": (5, 6)}.items():
        with open(f"{prefix}_{name}.csv", "w") as f:
            for r in rows:
                f.write("%.4f,%.4f,0.0\n" % (r[ix], r[iy]))

    # quick ATE (Umeyama-aligned) so the run self-reports without a second tool
    try:
        sys.path.insert(0, "/ros2_ws/src/eval_tools")
        import numpy as np
        from eval_tools.metrics import ate
        gt = np.array([[r[1], r[2], 0.0] for r in rows])
        ego = np.array([[r[5], r[6], 0.0] for r in rows])
        ov = np.array([[r[3], r[4], 0.0] for r in rows])
        a_ego = ate(ego, gt)[0]         # ate() -> (ErrorStats, aligned); rigid SE(3) align
        a_ov = ate(ov, gt)[0]
        gt_len = float(np.sum(np.linalg.norm(np.diff(gt, axis=0), axis=1)))
        print(f"rows={len(rows)}  rtf≈{rtf:.2f}  gt_path={gt_len:.2f}m  "
              f"ATE(ego_localizer)={a_ego.rmse:.3f}m  ATE(openvins_raw)={a_ov.rmse:.3f}m")
    except Exception as e:                                  # pragma: no cover
        print(f"rows={len(rows)} (ATE calc skipped: {e})")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
