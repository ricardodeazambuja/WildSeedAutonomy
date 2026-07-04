#!/usr/bin/env python3
"""Publish a correctly-stamped geometry_msgs/TwistStamped for a fixed duration.

Why this exists (a real ROS gotcha): the diff_drive controller runs on sim time
(use_sim_time) and rejects commands whose header.stamp is too old. `ros2 topic pub`
stamps every message with 0, so once sim time has advanced past the controller's
cmd_vel timeout, every bare-CLI command is silently dropped and the robot won't
move (it only "works" right after a fresh bring-up, while sim time is still near 0).
A node with use_sim_time:=true can stamp each message with clock.now() — sim time —
so the command is always fresh. This is the reliable teleop publish path (the same
thing teleop_twist_keyboard does with stamped:=true).

Usage: n1_drive.py <topic> <linear_x> <angular_z> <duration_s> [rate_hz]
Used by scripts/n1_worker.sh (N1 teleop demo).

<duration_s> is SIM seconds (so the driven path is identical at any RTF);
[rate_hz] stays a WALL rate (message freshness for twist_mux/diff_drive).
"""
import os
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped

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


def main():
    if len(sys.argv) < 5:
        print("usage: n1_drive.py <topic> <lin_x> <ang_z> <dur_s> [rate_hz]",
              file=sys.stderr)
        return 2
    topic = sys.argv[1]
    lx, az, dur = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    rate = float(sys.argv[5]) if len(sys.argv) > 5 else 20.0

    rclpy.init()
    node = rclpy.create_node(
        "n1_drive",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    pub = node.create_publisher(TwistStamped, topic, 10)

    # Wait for the sim clock to start (stamping with 0 would defeat the purpose).
    wait_for_clock(node)
    rtf = measure_rtf(node, sample_wall_s=1.0)

    # Drive for <dur> SIM seconds, publishing at ~rate Hz WALL (freshness).
    for _ in sim_window(node, dur, rtf, tick=1.0 / rate):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()   # sim-time "now"
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = lx
        msg.twist.angular.z = az
        pub.publish(msg)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
