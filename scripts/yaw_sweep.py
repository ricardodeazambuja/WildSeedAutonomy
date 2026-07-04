#!/usr/bin/env python3
"""Rotate the Husky in place and log yaw vs per-frame texture (std).

Publishes a constant angular.z to a cmd_vel input while recording, per camera
frame, the robot yaw and max per-channel std. Saves:
  - /results/yaw_sweep.csv         (t, yaw_deg, std_max, tag)
  - /results/sweep_scene.png       a representative textured frame
  - /results/sweep_blank.png       a representative solid frame (if any seen)
Prints the yaw band(s) where std collapses below the blank threshold.

Run in the fusion container. Args: [duration_s] [ang_z]
"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 26.0
ANG = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
CMD = sys.argv[3] if len(sys.argv) > 3 else "/a200_0000/joy_teleop/cmd_vel"
BLANK = 5.0

rclpy.init()
n = Node("yaw_sweep")
qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.VOLATILE
br = CvBridge()
S = {"img": None, "yaw": None}


def on_img(m):
    S["img"] = m


def on_odom(m):
    q = m.pose.pose.orientation
    S["yaw"] = math.degrees(
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


n.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", on_img, qos)
n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", on_odom, 10)
pub = n.create_publisher(Twist, CMD, 10)

# wait for first data
t0 = time.time()
while (S["img"] is None or S["yaw"] is None) and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
if S["img"] is None or S["yaw"] is None:
    print("NO image/odom — aborting")
    raise SystemExit(1)

yaw_start = S["yaw"]
tw = Twist()
tw.angular.z = ANG
rows = []
saved_scene = saved_blank = False
last_img_id = None
t0 = time.time()
while time.time() - t0 < DUR:
    pub.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.02)
    m = S["img"]
    if m is None or id(m) == last_img_id:
        continue
    last_img_id = id(m)
    img = br.imgmsg_to_cv2(m, desired_encoding="bgr8")
    std = float(img.reshape(-1, 3).std(axis=0).max())
    yaw = S["yaw"]
    tag = "BLANK" if std < BLANK else "SCENE"
    rows.append((time.time() - t0, yaw, std, tag))
    if tag == "SCENE" and not saved_scene:
        cv2.imwrite("/results/sweep_scene.png", img); saved_scene = True
    if tag == "BLANK" and not saved_blank:
        cv2.imwrite("/results/sweep_blank.png", img); saved_blank = True

# stop the robot
stop = Twist()
for _ in range(10):
    pub.publish(stop); rclpy.spin_once(n, timeout_sec=0.02)

# write CSV
with open("/results/yaw_sweep.csv", "w") as f:
    f.write("t,yaw_deg,std_max,tag\n")
    for r in rows:
        f.write(f"{r[0]:.3f},{r[1]:.2f},{r[2]:.3f},{r[3]}\n")

n_blank = sum(1 for r in rows if r[3] == "BLANK")
print(f"frames={len(rows)}  yaw_range=[{min(r[1] for r in rows):.0f},"
      f"{max(r[1] for r in rows):.0f}] deg  blank={n_blank} "
      f"({100*n_blank/max(1,len(rows)):.0f}%)")
# report blank yaw bands (sorted by yaw)
blanks = sorted(r[1] for r in rows if r[3] == "BLANK")
if blanks:
    bands = []
    lo = prev = blanks[0]
    for y in blanks[1:]:
        if y - prev > 8:
            bands.append((lo, prev)); lo = y
        prev = y
    bands.append((lo, prev))
    print("BLANK yaw bands (deg):", ", ".join(f"[{a:.0f}..{b:.0f}]" for a, b in bands))
    print(f"SCENE std typical: "
          f"{np.median([r[2] for r in rows if r[3]=='SCENE']):.1f}; "
          f"BLANK std max: {max(r[2] for r in rows if r[3]=='BLANK'):.2f}")
else:
    print("no blank frames seen in this sweep")
print(f"saved_scene={saved_scene} saved_blank={saved_blank}")
