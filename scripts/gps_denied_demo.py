#!/usr/bin/env python3
"""GPS-denied keystone demo (in sim) — drive, drop GPS, reacquire; record the result.

Runs INSIDE a container on the sim's ROS graph (use_sim_time). Drives the Husky on
a curved path while ego_localizer (odom_mode:=relative + GNSS) fuses. Phases:
  on → denied (GPS toggled off) → reacquire (on). Records ego_localizer pose and the
GNSS fix (converted to the same local ENU = ground-truth track) to a CSV so the
drift→reacquire can be charted (PLAN chart #1 / §11, M5 in sim).

Needs: husky sim up (with the gps sensor) and ego_localizer running in relative+GNSS
mode (launch/ego_localizer_gnss.launch.py).

Usage: gps_denied_demo.py <out.csv> [v] [wz] [on_s] [denied_s] [reacq_s]
  Defaults drive slow with a long denial so the dead-reckoning drift clearly
  exceeds the ~1 Hz GPS sawtooth (a cleaner keystone chart).

All durations (and the CSV `t` column) are SIM seconds — at low RTF the demo
takes proportionally longer in wall time but the recorded physics is identical.
The measured RTF is printed at start and in the summary.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

EARTH_R = 6378137.0
NS = "/a200_0000"

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
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/gps_denied.csv"
    V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
    WZ = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10
    ON_S = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0
    DENIED_S = float(sys.argv[5]) if len(sys.argv) > 5 else 40.0
    REACQ_S = float(sys.argv[6]) if len(sys.argv) > 6 else 20.0
    rclpy.init()
    n = rclpy.create_node("gps_denied_demo",
                          parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    drive = n.create_publisher(TwistStamped, f"{NS}/joy_teleop/cmd_vel", 10)
    toggle = n.create_publisher(Bool, "/ego_localizer/set_gps_enabled", 10)
    state = {"ego": None, "gps": None, "enu0": None}
    n.create_subscription(Odometry, "/ego_localizer/odom",
                          lambda m: state.__setitem__("ego", (m.pose.pose.position.x,
                                                              m.pose.pose.position.y)), 10)

    def on_gps(m):
        if state["enu0"] is None:
            state["enu0"] = (m.latitude, m.longitude)
        lat0, lon0 = state["enu0"]
        e = math.radians(m.longitude - lon0) * EARTH_R * math.cos(math.radians(lat0))
        nth = math.radians(m.latitude - lat0) * EARTH_R
        state["gps"] = (e, nth)
    n.create_subscription(NavSatFix, f"{NS}/sensors/gps_0/fix", on_gps, 10)

    # Wait for sim clock + first GPS only (NOT ego): in course-aided relative mode
    # ego_localizer doesn't seed/publish until the robot MOVES, and we're the one
    # that moves it — so requiring ego here would deadlock. ego data appears during
    # the first drive phase once it seeds from the GPS course.
    wait_for_clock(n)
    rtf = measure_rtf(n)
    for _ in sim_window(n, 30.0, rtf, tick=0.1):   # GPS is ~1 Hz SIM time
        if state["gps"] is not None:
            break
    if state["gps"] is None:
        print(f"FAIL: no gps data after 30 sim-s at RTF≈{rtf:.2f} "
              "(is the husky sim with the gps sensor up?)", file=sys.stderr)
        return 1

    rows = []
    t0s = sim_now(n)   # CSV t column = SIM seconds since the demo start

    def set_gps(on):
        b = Bool(); b.data = on
        for _ in range(5):
            toggle.publish(b); rclpy.spin_once(n, timeout_sec=0.02)

    def run_phase(label, secs, gps_on):
        set_gps(gps_on)
        next_row = -1.0
        for el in sim_window(n, secs, rtf):
            m = TwistStamped()
            m.header.stamp = n.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x = V
            m.twist.angular.z = WZ
            drive.publish(m)
            # decimate rows to one per 0.05 SIM-s (the pre-sim-time cadence, so
            # CSV row counts stay comparable at any RTF)
            if state["ego"] and state["gps"] and el >= next_row:
                ex, ey = state["ego"]; gx, gy = state["gps"]
                rows.append((sim_now(n) - t0s, label, ex, ey, gx, gy,
                             math.hypot(ex - gx, ey - gy)))
                next_row = el + 0.05

    run_phase("on", ON_S, True)
    run_phase("denied", DENIED_S, False)
    run_phase("reacq", REACQ_S, True)
    # stop the robot (zero commands for 0.5 sim-s — the diff_drive ramp-down)
    for _ in sim_window(n, 0.5, rtf):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        drive.publish(m)

    with open(out, "w") as f:
        f.write("t,phase,ego_x,ego_y,gps_x,gps_y,err\n")
        for r in rows:
            f.write("%.3f,%s,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)

    def mean_err(label):
        e = [r[6] for r in rows if r[1] == label]
        return sum(e) / len(e) if e else float("nan")
    print(f"rows={len(rows)}  rtf≈{rtf:.2f}  mean|ego-gps|  on={mean_err('on'):.3f}  "
          f"denied={mean_err('denied'):.3f}  reacq={mean_err('reacq'):.3f}  -> {out}")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
