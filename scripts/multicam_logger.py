#!/usr/bin/env python3
"""Continuous 4-camera logger for the blank-band test. Subscribe-only.

While you slowly rotate the robot, logs per camera frame:
  (t, robot_yaw, cam_i, world_face, std)  -> /results/multicam_log.csv
Saves a montage the first instant ALL FOUR cameras are blank:
  /results/mc_all_blank.png
At the end prints, per camera, the blank bands in WORLD-FACE coordinates — if the
blank is locked to world direction, all four cameras share the same bands."""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
MOUNT = {0: 0.0, 1: 90.0, 2: 180.0, 3: -90.0}
BLANK = 5.0
br = CvBridge()
rclpy.init()
n = Node("multicam_logger")
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


def norm(a):
    return (a + 180) % 360 - 180


print(f"[multicam_logger] started ({DUR:.0f}s). Slowly rotate the robot ~360.",
      flush=True)
rows = []
saved_allblank = False
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.03)
    if S["yaw"] is None:
        continue
    stds = {}
    for i in range(4):
        m = S[f"c{i}"]
        if m is None:
            continue
        img = br.imgmsg_to_cv2(m, desired_encoding="bgr8")
        std = float(img.reshape(-1, 3).std(axis=0).max())
        stds[i] = (std, img)
        rows.append((time.time() - t0, S["yaw"], i, norm(S["yaw"] + MOUNT[i]), std))
    if len(stds) == 4 and all(stds[i][0] < BLANK for i in range(4)) and not saved_allblank:
        tiles = []
        for i in range(4):
            lab = stds[i][1].copy()
            wf = norm(S["yaw"] + MOUNT[i])
            cv2.putText(lab, f"cam{i} face={wf:+.0f} std={stds[i][0]:.0f} BLANK",
                        (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            tiles.append(lab)
        mont = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
        cv2.imwrite("/results/mc_all_blank.png", mont)
        saved_allblank = True
        print(f"[ALL BLANK] robot_yaw={S['yaw']:.1f}  saved mc_all_blank.png", flush=True)

with open("/results/multicam_log.csv", "w") as f:
    f.write("t,robot_yaw,cam,world_face,std\n")
    for r in rows:
        f.write(f"{r[0]:.3f},{r[1]:.2f},{r[2]},{r[3]:.2f},{r[4]:.3f}\n")

# per-camera blank world-face bands
print("\nblank world-face angles per camera (deg):", flush=True)
for i in range(4):
    faces = sorted(r[3] for r in rows if r[2] == i and r[4] < BLANK)
    if not faces:
        print(f"  cam{i}: none blank"); continue
    print(f"  cam{i}: {faces[0]:.0f}..{faces[-1]:.0f}  "
          f"(n={len(faces)}, median {np.median(faces):.0f})")
print(f"[multicam_logger] done. rows={len(rows)} all_blank_seen={saved_allblank}",
      flush=True)
