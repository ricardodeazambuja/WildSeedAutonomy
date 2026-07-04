#!/usr/bin/env python3
"""One-shot camera peek: grab the latest raw OAK-D frame + robot yaw, save a PNG,
report mean/std colour and BLANK/SCENE. Run in the fusion container."""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge

OUT = sys.argv[1] if len(sys.argv) > 1 else "/results/peek.png"

rclpy.init()
n = Node("peek")
qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.VOLATILE
state = {"img": None, "yaw": None}
br = CvBridge()


def on_img(m):
    state["img"] = m


def on_odom(m):
    q = m.pose.pose.orientation
    state["yaw"] = math.degrees(
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


n.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", on_img, qos)
n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", on_odom, 10)

t0 = time.time()
while (state["img"] is None or state["yaw"] is None) and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.2)

if state["img"] is None:
    print("NO IMAGE on topic")
    raise SystemExit(1)

img = br.imgmsg_to_cv2(state["img"], desired_encoding="bgr8")
flat = img.reshape(-1, 3)
std = flat.std(axis=0)
mean = flat.mean(axis=0)
cv2.imwrite(OUT, img)
yaw = state["yaw"] if state["yaw"] is not None else float("nan")
tag = "BLANK" if std.max() < 5 else "SCENE"
print(f"saved {OUT}  yaw={yaw:.1f} deg  mean(BGR)={mean.round(1)}  "
      f"std(BGR)={std.round(2)}  -> {tag}")
