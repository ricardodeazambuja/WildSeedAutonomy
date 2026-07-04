#!/usr/bin/env python3
"""Rotate the Husky in place with TwistStamped (sim-time stamped) and report yaw
delta. Args: [ang_z] [dur] [topic]. Default topic = highest-prio twist_mux input."""
import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

ANG = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
TOP = sys.argv[3] if len(sys.argv) > 3 else "/a200_0000/rc_teleop/cmd_vel"

rclpy.init()
n = Node("rotate_stamped", parameter_overrides=[Parameter("use_sim_time", value=True)])
S = {"y": None}


def od(m):
    q = m.pose.pose.orientation
    S["y"] = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z)))


n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", od, 10)
p = n.create_publisher(TwistStamped, TOP, 10)

# wait for sim clock + odom
t0 = time.time()
while (S["y"] is None or n.get_clock().now().nanoseconds == 0) and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
y0 = S["y"]
t0 = time.time()
while time.time() - t0 < DUR:
    msg = TwistStamped()
    msg.header.stamp = n.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    msg.twist.angular.z = ANG
    p.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.02)
d = (S["y"] - y0) if (S["y"] is not None and y0 is not None) else float("nan")
print(f"topic={TOP}  yaw {y0:.1f} -> {S['y']:.1f}  (delta {d:+.1f} deg)")
