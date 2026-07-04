#!/usr/bin/env python3
"""M3 texture gate — does the sim camera give a VIO front-end enough to track?

PLAN §14 / §17.4 wall: a filter-based VIO like OpenVINS tracks *corner features*
(Shi-Tomasi / FAST) frame-to-frame with KLT. If the world is low-texture — flat
untextured ground, blank sky, repetitive walls — too few corners survive and the
front-end diverges, taking all downstream fusion with it. So before wiring any
fusion we sanity-check the actual image: grab one frame and count the corners a
KLT tracker would find. This mirrors what OpenVINS does internally (Shi-Tomasi +
a per-cell cap), so the count here predicts whether OpenVINS will hold lock.

Rule of thumb: OpenVINS defaults to ~100-200 tracked features; a healthy frame
should yield several hundred strong Shi-Tomasi corners. < ~50 is a red flag.

Run inside the fusion container (has cv2 + cv_bridge):
  ros2 run ... no — just: python3 scripts/check_sim_texture.py [topic] [out_dir]
with the husky sim up.
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/a200_0000/sensors/camera_0/color/image"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/results"


def main():
    rclpy.init()
    n = rclpy.create_node("texture_gate")
    bridge = CvBridge()
    got = {"img": None}
    n.create_subscription(
        Image, TOPIC, lambda m: got.__setitem__("img", bridge.imgmsg_to_cv2(m, "bgr8")), 10)

    # wait for one frame
    import time
    t0 = time.time()
    while rclpy.ok() and got["img"] is None and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if got["img"] is None:
        print(f"FAIL: no image on {TOPIC} (is the husky sim up?)", file=sys.stderr)
        return 1

    img = got["img"]
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Shi-Tomasi corners (what KLT/OpenVINS track). qualityLevel 0.01 is OpenVINS-ish.
    corners = cv2.goodFeaturesToTrack(gray, maxCorners=1000, qualityLevel=0.01,
                                      minDistance=8)
    n_shi = 0 if corners is None else len(corners)
    # FAST corners as a second opinion (OpenVINS can use FAST too).
    fast = cv2.FastFeatureDetector_create(threshold=20)
    n_fast = len(fast.detect(gray, None))
    # Image texture proxy: stddev of the Laplacian (focus/detail measure).
    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()

    print(f"image      : {w}x{h}  encoding bgr8")
    print(f"Shi-Tomasi : {n_shi:4d} strong corners (KLT-trackable)")
    print(f"FAST       : {n_fast:4d} corners")
    print(f"Laplacian  : var={lap_var:8.1f}  (higher = more high-freq detail)")
    verdict = "PASS" if n_shi >= 150 else ("MARGINAL" if n_shi >= 50 else "FAIL")
    print(f"VERDICT    : {verdict}  (PASS>=150  MARGINAL>=50  else FAIL)")

    # Save the frame + an overlay of the corners so a human can eyeball the world.
    cv2.imwrite(f"{OUT}/sim_camera_frame.png", img)
    vis = img.copy()
    if corners is not None:
        for c in corners.astype(int):
            cv2.circle(vis, tuple(c.ravel()), 3, (0, 255, 0), -1)
    cv2.imwrite(f"{OUT}/sim_camera_features.png", vis)
    print(f"wrote {OUT}/sim_camera_frame.png and sim_camera_features.png")
    n.destroy_node(); rclpy.shutdown()
    return 0 if verdict != "FAIL" else 2


if __name__ == "__main__":
    sys.exit(main())
