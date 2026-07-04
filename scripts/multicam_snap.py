#!/usr/bin/env python3
"""Snapshot all 4 robot cameras at one instant + robot yaw.

For each camera: per-channel std (BLANK if <5), and the WORLD heading it faces
(robot_yaw + mount_yaw). Saves each frame and a 2x2 montage to /results.
This tells us, at a single robot pose, which world directions render vs blank —
no rotation, no position variable. Run in the fusion container."""
import time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge

MOUNT = {0: 0.0, 1: 90.0, 2: 180.0, 3: -90.0}  # camera_i yaw offset (deg)
br = CvBridge()
rclpy.init()
n = Node("multicam_snap")
qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.VOLATILE
S = {f"c{i}": None for i in range(4)}
S["yaw"] = None


def mk(i):
    def cb(m):
        S[f"c{i}"] = m
    return cb


for i in range(4):
    n.create_subscription(
        Image, f"/a200_0000/sensors/camera_{i}/color/image", mk(i), qos)


def od(m):
    q = m.pose.pose.orientation
    S["yaw"] = math.degrees(
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", od, 10)

t0 = time.time()
while time.time() - t0 < 6 and (S["yaw"] is None or any(S[f"c{i}"] is None for i in range(4))):
    rclpy.spin_once(n, timeout_sec=0.1)


def norm(a):
    return (a + 180) % 360 - 180


tiles = []
print(f"robot_yaw = {S['yaw']:.1f} deg")
print(f"{'cam':<6}{'mount':>7}{'world_face':>12}{'std_max':>10}  state")
for i in range(4):
    m = S[f"c{i}"]
    if m is None:
        print(f"camera_{i}: NO FRAME")
        tiles.append(np.zeros((240, 320, 3), np.uint8))
        continue
    img = br.imgmsg_to_cv2(m, desired_encoding="bgr8")
    std = float(img.reshape(-1, 3).std(axis=0).max())
    wf = norm(S["yaw"] + MOUNT[i])
    state = "BLANK" if std < 5 else "SCENE"
    print(f"cam_{i:<2}{MOUNT[i]:>7.0f}{wf:>12.1f}{std:>10.2f}  {state}")
    cv2.imwrite(f"/results/mc_cam{i}.png", img)
    lab = img.copy()
    cv2.putText(lab, f"cam{i} face={wf:+.0f} std={std:.0f} {state}",
                (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
    tiles.append(lab)

top = np.hstack([tiles[0], tiles[1]])
bot = np.hstack([tiles[2], tiles[3]])
cv2.imwrite("/results/mc_montage.png", np.vstack([top, bot]))
print("saved /results/mc_montage.png (cam0 cam1 / cam2 cam3)")
