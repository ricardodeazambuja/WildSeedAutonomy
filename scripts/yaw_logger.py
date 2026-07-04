#!/usr/bin/env python3
"""PASSIVE yaw/texture logger — subscribe only, publishes nothing, drives nothing.

While you teleop the robot, this records per camera frame:
  (t, yaw_deg, std_max, tag)  -> /results/yaw_log.csv
and saves representative frames as the scene/blank states are seen:
  /results/log_scene.png   first textured frame
  /results/log_blank.png   first solid frame (std < BLANK)
  /results/log_edge.png    a frame right at a scene->blank transition
Prints a live line whenever the tag flips (SCENE<->BLANK) with the yaw it flipped at.

Run in the fusion container. Arg: [duration_s] (default 600).
"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
BLANK = 5.0

rclpy.init()
n = Node("yaw_logger")
qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.VOLATILE
br = CvBridge()
S = {"img": None, "yaw": None, "x": None, "y": None, "roll": None, "pitch": None}


def on_img(m):
    S["img"] = m


def on_odom(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    S["x"], S["y"] = p.x, p.y
    # full roll/pitch/yaw from quaternion
    S["roll"] = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z),
                                        1 - 2 * (q.x * q.x + q.y * q.y)))
    sinp = 2 * (q.w * q.y - q.z * q.x)
    S["pitch"] = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
    S["yaw"] = math.degrees(
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


n.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", on_img, qos)
n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", on_odom, 10)

print(f"[yaw_logger] passive capture started ({DUR:.0f}s). Drive the robot now.",
      flush=True)
rows = []
saved = {"scene": False, "blank": False, "edge": False}
prev_tag = None
last_id = None
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    m = S["img"]
    if m is None or S["yaw"] is None or id(m) == last_id:
        continue
    last_id = id(m)
    img = br.imgmsg_to_cv2(m, desired_encoding="bgr8")
    std = float(img.reshape(-1, 3).std(axis=0).max())
    yaw = S["yaw"]
    tag = "BLANK" if std < BLANK else "SCENE"
    rows.append((time.time() - t0, yaw, std, tag))
    if tag == "SCENE" and not saved["scene"]:
        cv2.imwrite("/results/log_scene.png", img); saved["scene"] = True
    if tag == "BLANK" and not saved["blank"]:
        cv2.imwrite("/results/log_blank.png", img); saved["blank"] = True
    if prev_tag is not None and tag != prev_tag:
        print(f"[flip] {prev_tag}->{tag} at yaw={yaw:.1f} deg  std={std:.1f}  "
              f"t={time.time()-t0:.1f}s", flush=True)
        if not saved["edge"]:
            cv2.imwrite("/results/log_edge.png", img); saved["edge"] = True
    prev_tag = tag

with open("/results/yaw_log.csv", "w") as f:
    f.write("t,yaw_deg,std_max,tag\n")
    for r in rows:
        f.write(f"{r[0]:.3f},{r[1]:.2f},{r[2]:.3f},{r[3]}\n")
nb = sum(1 for r in rows if r[3] == "BLANK")
print(f"[yaw_logger] done. frames={len(rows)} blank={nb} "
      f"saved={saved}", flush=True)
