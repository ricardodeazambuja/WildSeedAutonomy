#!/usr/bin/env python3
"""M3 smoke gate — fail-fast check that the stereo VIO pipeline is alive and tracking.

This is the "texture/feature gate" (PLAN §14 / §17.4) extended from one camera to the
whole stereo-OpenVINS path. A feature-based VIO is silently worthless if the camera
renders blank or low-texture (it tracks nothing, emits a confident-but-wrong pose), so
we assert the *inputs* before trusting any number. Run with the sim + vio stack up
(`deploy.sh up compute && deploy.sh up vio`); `deploy.sh m3-smoke` wraps both.

Checks (all must pass), mirroring the failures this milestone actually hit:
  1. RENDER   — both camera_0 (LEFT) + camera_1 (RIGHT) deliver a frame that is SCENE,
                not a uniform blank (the llvmpipe/EGL + sub-deck-mount bug, #8).
  2. TEXTURE  — each frame yields enough Shi-Tomasi corners for KLT to hold lock
                (OpenVINS tracks ~100-200; <50 starves the front-end).
  3. STEREO   — the two frames are time-synced (a guarded/raw topic mix would desync
                the pair and OpenVINS would drop every stereo match).
  4. VIO LIVE — after a jerk-start (static init needs an accel transient the smooth
                drive never makes), OpenVINS publishes /odomimu = stereo init fired.

Exit 0 = PASS, non-zero = FAIL (so it works as a CI/regression gate).

Usage (inside the fusion container):
  python3 /ros2_ws/src/sensing_bringup/scripts/m3_smoke.py
"""
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

NS = "/a200_0000"
CAM = {0: f"{NS}/sensors/camera_0/color/image",   # LEFT
       1: f"{NS}/sensors/camera_1/color/image"}   # RIGHT
ODOM = "/odomimu"

# Thresholds — generous margins so the gate flags real regressions, not noise.
MIN_STD = 8.0        # mean per-channel std below this == blank/near-uniform frame
MIN_CORNERS = 80     # Shi-Tomasi corners per frame (PASS>=150 ideal; 80 = robust floor)
MAX_SYNC_MS = 5.0    # sim is perfectly synced (~0 ms); >5 ms means a topic-path mismatch
VIO_WAIT_S = 30.0    # how long to wait for /odomimu after the jerk-start


def stamp_s(h):
    return h.sec + h.nanosec * 1e-9


def main():
    rclpy.init()
    n = rclpy.create_node("m3_smoke",
                          parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    br = CvBridge()
    rel = QoSProfile(depth=5); rel.reliability = ReliabilityPolicy.RELIABLE
    be = QoSProfile(depth=20); be.reliability = ReliabilityPolicy.BEST_EFFORT

    frames = {0: None, 1: None}
    stamps = {0: [], 1: []}

    def on_img(cc, m):
        frames[cc] = m
        stamps[cc].append(stamp_s(m.header.stamp))

    for c in (0, 1):
        n.create_subscription(Image, CAM[c],
                              (lambda cc: (lambda m: on_img(cc, m)))(c), rel)
    ov = {"msg": None}
    n.create_subscription(Odometry, ODOM, lambda m: ov.__setitem__("msg", m), be)

    drive = n.create_publisher(TwistStamped, f"{NS}/joy_teleop/cmd_vel", 10)

    fails = []

    def frame_std(msg):
        img = br.imgmsg_to_cv2(msg, "bgr8")
        return float(img.reshape(-1, 3).std(0).mean())

    # --- wait for a NON-BLANK frame from each camera ---
    # Tolerate transient startup blanks: the gz camera emits a few uniform frames
    # before the GPU render warms up, so grabbing the *first* frame races and false-
    # fails. The callback keeps the latest frame, so we wait until both are non-blank
    # (or time out — a persistent blank is then a real render-bug-#8 failure).
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
        if (frames[0] is not None and frames[1] is not None
                and frame_std(frames[0]) >= MIN_STD and frame_std(frames[1]) >= MIN_STD):
            break
    for c in (0, 1):
        if frames[c] is None:
            fails.append(f"no frame on {CAM[c]} (sim up? camera_{c} bridged?)")
    if frames[0] is None or frames[1] is None:
        return _report(n, fails)

    # --- 1+2. RENDER (not blank) + TEXTURE (enough corners), per camera ---
    side = {0: "LEFT", 1: "RIGHT"}
    for c in (0, 1):
        img = br.imgmsg_to_cv2(frames[c], "bgr8")
        std = float(img.reshape(-1, 3).std(0).mean())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray, maxCorners=1000, qualityLevel=0.01, minDistance=8)
        ncorn = 0 if corners is None else len(corners)
        tag = "OK" if (std >= MIN_STD and ncorn >= MIN_CORNERS) else "FAIL"
        print(f"  cam{c} ({side[c]:5s}): std={std:5.1f}  corners={ncorn:4d}  [{tag}]")
        if std < MIN_STD:
            fails.append(f"cam{c} blank/uniform (std {std:.1f} < {MIN_STD}) — render bug #8?")
        elif ncorn < MIN_CORNERS:
            fails.append(f"cam{c} low texture ({ncorn} < {MIN_CORNERS} corners) — VIO would starve")

    # --- 3. STEREO time-sync: do matching stamps EXIST? ---
    # The two gz cameras emit IDENTICAL sim stamps when paired, but comparing the
    # independently-latest frame from each topic is racy (one's newer frame arrives a
    # beat before the other's). So collect ~3 s of stamps and take the BEST cross-match
    # — that is what OpenVINS does to pair the stereo frames, and it's phase-robust.
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if stamps[0] and stamps[1]:
        dms = min(abs(a - b) for a in stamps[0][-15:] for b in stamps[1][-15:]) * 1e3
    else:
        dms = float("inf")
    print(f"  stereo sync: best pair |Δt| = {dms:.2f} ms  [{'OK' if dms <= MAX_SYNC_MS else 'FAIL'}]")
    if dms > MAX_SYNC_MS:
        fails.append(f"stereo pair desynced (best |Δt| {dms:.1f} ms > {MAX_SYNC_MS}) — cam0/cam1 topic/rate mismatch?")

    # --- 4. VIO LIVE: jerk-start, then wait for /odomimu ---
    while rclpy.ok() and n.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(n, timeout_sec=0.1)

    def send(vx):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "base_link"; m.twist.linear.x = float(vx); drive.publish(m)

    if ov["msg"] is None:                       # only jerk if not already initialised
        for dur, vx in ((2.0, 0.0), (0.4, 1.2), (0.4, 0.0), (0.4, 1.2), (0.5, 0.0)):
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < dur:
                send(vx); rclpy.spin_once(n, timeout_sec=0.02)
    t0 = time.time()
    while rclpy.ok() and ov["msg"] is None and time.time() - t0 < VIO_WAIT_S:
        send(0.3); rclpy.spin_once(n, timeout_sec=0.02)   # keep nudging (sustained motion)
    send(0.0)
    live = ov["msg"] is not None
    print(f"  OpenVINS {ODOM}: {'publishing' if live else 'SILENT'}  [{'OK' if live else 'FAIL'}]")
    if not live:
        fails.append(f"no {ODOM} within {VIO_WAIT_S:.0f}s of jerk-start — OpenVINS stereo init failed")

    return _report(n, fails)


def _report(n, fails):
    print()
    if fails:
        print(f"M3 SMOKE: FAIL ({len(fails)} check(s))")
        for f in fails:
            print(f"  - {f}")
        rc = 1
    else:
        print("M3 SMOKE: PASS — stereo renders, tracks, syncs, and OpenVINS VIO is live.")
        rc = 0
    n.destroy_node(); rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
