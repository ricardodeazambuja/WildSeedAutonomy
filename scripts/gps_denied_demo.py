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
"""
import math
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
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 30 and (
            n.get_clock().now().nanoseconds == 0 or state["gps"] is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    if state["gps"] is None:
        print("FAIL: no gps data (is the husky sim with the gps sensor up?)", file=sys.stderr)
        return 1

    rows = []

    def set_gps(on):
        b = Bool(); b.data = on
        for _ in range(5):
            toggle.publish(b); rclpy.spin_once(n, timeout_sec=0.02)

    def run_phase(label, secs, gps_on):
        set_gps(gps_on)
        end = time.time() + secs
        while rclpy.ok() and time.time() < end:
            m = TwistStamped()
            m.header.stamp = n.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x = V
            m.twist.angular.z = WZ
            drive.publish(m)
            rclpy.spin_once(n, timeout_sec=0.0)
            if state["ego"] and state["gps"]:
                ex, ey = state["ego"]; gx, gy = state["gps"]
                rows.append((time.time() - t0, label, ex, ey, gx, gy,
                             math.hypot(ex - gx, ey - gy)))
            time.sleep(0.05)

    run_phase("on", ON_S, True)
    run_phase("denied", DENIED_S, False)
    run_phase("reacq", REACQ_S, True)
    # stop the robot
    for _ in range(10):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        drive.publish(m); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.02)

    with open(out, "w") as f:
        f.write("t,phase,ego_x,ego_y,gps_x,gps_y,err\n")
        for r in rows:
            f.write("%.3f,%s,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)

    def mean_err(label):
        e = [r[6] for r in rows if r[1] == label]
        return sum(e) / len(e) if e else float("nan")
    print(f"rows={len(rows)}  mean|ego-gps|  on={mean_err('on'):.3f}  "
          f"denied={mean_err('denied'):.3f}  reacq={mean_err('reacq'):.3f}  -> {out}")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
