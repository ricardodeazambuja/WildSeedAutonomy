#!/usr/bin/env python3
"""Rotate the Husky a full turn in place via TwistStamped (sim-stamped) to
platform/cmd_vel (twist_mux idle -> no competition). Stops at ~target deg of
cumulative yaw or on timeout. Args: [ang_z] [target_deg] [timeout_s]."""
import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

ANG = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
TARGET = float(sys.argv[2]) if len(sys.argv) > 2 else 370.0
TIMEOUT = float(sys.argv[3]) if len(sys.argv) > 3 else 160.0

rclpy.init()
n = Node("rotate_full", parameter_overrides=[Parameter("use_sim_time", value=True)])
S = {"y": None}


def od(m):
    q = m.pose.pose.orientation
    S["y"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", od, 10)
p = n.create_publisher(TwistStamped, "/a200_0000/platform/cmd_vel", 10)

t0 = time.time()
while (S["y"] is None or n.get_clock().now().nanoseconds == 0) and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)

prev = S["y"]
cum = 0.0
t0 = time.time()
last_print = 0
while abs(cum) < math.radians(TARGET) and time.time() - t0 < TIMEOUT:
    m = TwistStamped()
    m.header.stamp = n.get_clock().now().to_msg()
    m.header.frame_id = "base_link"
    m.twist.angular.z = ANG
    p.publish(m)
    rclpy.spin_once(n, timeout_sec=0.02)
    if S["y"] is not None and prev is not None:
        d = S["y"] - prev
        if d > math.pi: d -= 2 * math.pi
        if d < -math.pi: d += 2 * math.pi
        cum += d
        prev = S["y"]
    deg = math.degrees(cum)
    if int(abs(deg)) // 30 != last_print:
        last_print = int(abs(deg)) // 30
        print(f"  cumulative yaw {deg:+.0f} deg  (t={time.time()-t0:.0f}s)", flush=True)

# stop
for _ in range(15):
    m = TwistStamped()
    m.header.stamp = n.get_clock().now().to_msg()
    p.publish(m)
    rclpy.spin_once(n, timeout_sec=0.02)
print(f"DONE cumulative yaw {math.degrees(cum):+.0f} deg in {time.time()-t0:.0f}s",
      flush=True)
