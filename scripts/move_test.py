#!/usr/bin/env python3
"""Publish angular.z to a chosen topic for N s, report yaw delta. Args: topic [ang] [dur]"""
import sys, math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

TOP = sys.argv[1]
ANG = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

rclpy.init()
n = Node("move_test")
S = {"y": None}


def od(m):
    q = m.pose.pose.orientation
    S["y"] = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z)))


n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", od, 10)
p = n.create_publisher(Twist, TOP, 10)
t0 = time.time()
while S["y"] is None and time.time() - t0 < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
y0 = S["y"]
tw = Twist()
tw.angular.z = ANG
t0 = time.time()
while time.time() - t0 < DUR:
    p.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.02)
d = (S["y"] - y0) if (S["y"] is not None and y0 is not None) else float("nan")
print(f"topic={TOP}  yaw {y0:.1f} -> {S['y']:.1f}  (delta {d:+.1f} deg)")
