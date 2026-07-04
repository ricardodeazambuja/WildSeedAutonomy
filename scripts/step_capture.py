#!/usr/bin/env python3
"""Rotate to discrete headings, STOP, settle, capture camera std + save image.
Isolates heading-dependence from motion (each sample is taken stationary)."""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge

TARGETS = list(range(0, 360, 45))   # 0,45,...,315
br = CvBridge()
rclpy.init()
n = Node("step_capture", parameter_overrides=[Parameter("use_sim_time", value=True)])
q = QoSProfile(depth=5); q.reliability = ReliabilityPolicy.RELIABLE; q.durability = DurabilityPolicy.VOLATILE
S = {"img": None, "yaw": None}
n.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", lambda m: S.__setitem__("img", m), q)


def od(m):
    o = m.pose.pose.orientation
    S["yaw"] = math.degrees(math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y*o.y+o.z*o.z)))


n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", od, 10)
pub = n.create_publisher(TwistStamped, "/a200_0000/platform/cmd_vel", 10)

t0 = time.time()
while (S["yaw"] is None or n.get_clock().now().nanoseconds == 0) and time.time()-t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)


def norm(a): return (a+180) % 360 - 180


def drive_to(tgt, timeout=15):
    t0 = time.time()
    while time.time()-t0 < timeout:
        err = norm(tgt - S["yaw"])
        if abs(err) < 2.0:
            break
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        m.twist.angular.z = max(-0.6, min(0.6, math.radians(err)*1.5))
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02)
    for _ in range(20):   # stop
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02)


print(f"{'target':>7}{'actual':>8}{'std':>8}  state   (45-mult = cardinal/diagonal)")
results = []
tiles = []
for tgt in TARGETS:
    drive_to(tgt)
    time.sleep(0.5)
    for _ in range(15):   # settle + fetch fresh frame
        rclpy.spin_once(n, timeout_sec=0.1)
    img = br.imgmsg_to_cv2(S["img"], desired_encoding="bgr8")
    std = float(img.reshape(-1, 3).std(axis=0).max())
    actual = S["yaw"]
    state = "BLANK" if std < 5 else "SCENE"
    kind = "cardinal" if tgt % 90 == 0 else "diagonal"
    print(f"{tgt:>7}{actual:>8.1f}{std:>8.1f}  {state:<6} {kind}")
    results.append((tgt, std))
    lab = img.copy()
    cv2.putText(lab, f"{tgt}deg std={std:.0f} {state}", (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)
    tiles.append(lab)

rows = [np.hstack(tiles[i:i+4]) for i in range(0, 8, 4)]
cv2.imwrite("/results/step_headings.png", np.vstack(rows))
blank = sum(1 for _, s in results if s < 5)
print(f"\nblank {blank}/8 headings; saved /results/step_headings.png")
