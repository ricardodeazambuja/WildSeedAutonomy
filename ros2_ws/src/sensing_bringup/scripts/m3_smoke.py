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
import os
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
VIO_WAIT_S = 30.0    # SIM seconds to wait for /odomimu after the jerk-start

# ── sim-time helpers ─────────────────────────────────────────────────────────
# DUPLICATED in: scripts/m3_vio_demo.py, scripts/gps_denied_demo.py,
# scripts/n1_drive.py, ros2_ws/src/sensing_bringup/scripts/m3_smoke.py
# (three different container delivery paths — no shared import; keep in sync).
#
# All experiment durations are SIM seconds: at low RTF the run takes longer on
# the wall clock but the physics (drive distance, drift windows, jerk
# transients) is identical. Wall-clock ceilings only catch a wedged sim.
RTF_FLOOR = float(os.environ.get("SIM_RTF_FLOOR", "0.02"))


def sim_now(n):
    return n.get_clock().now().nanoseconds * 1e-9


def wait_for_clock(n, wall_ceiling=120.0):
    t0 = time.time()
    while rclpy.ok() and n.get_clock().now().nanoseconds == 0:
        if time.time() - t0 > wall_ceiling:
            raise SystemExit(f"FAIL: no /clock after {wall_ceiling:.0f}s wall — sim up? "
                             "clock_bridge alive? (docs/sim-debugging-notes.md #7)")
        rclpy.spin_once(n, timeout_sec=0.1)


def measure_rtf(n, sample_wall_s=3.0):
    s0, w0 = sim_now(n), time.time()
    while time.time() - w0 < sample_wall_s:
        rclpy.spin_once(n, timeout_sec=0.05)
    rtf = max((sim_now(n) - s0) / (time.time() - w0), 1e-4)
    note = (f"  (SLOW SIM: durations are SIM seconds; wall time stretches ~{1 / rtf:.0f}x)"
            if rtf < 0.5 else "")
    print(f"[simtime] RTF≈{rtf:.3f}{note}", flush=True)
    if rtf < RTF_FLOOR:
        raise SystemExit(f"FAIL: RTF {rtf:.3f} < SIM_RTF_FLOOR {RTF_FLOOR} — sim too slow "
                         "to be meaningful (see docs/operations.md 'Slow machines / low RTF').")
    return rtf


def sim_window(n, sim_secs, rtf, tick=0.02, safety=5.0):
    """Yield sim-elapsed while < sim_secs of SIM time has passed; wall-ceiling backstop.

    Each iteration ends in spin_once(tick) — at tick=0.02 the loop also paces a
    ~50 Hz publisher (the twist_mux lesson, docs/m3-vio.md)."""
    s0, w0 = sim_now(n), time.time()
    ceiling = sim_secs / max(rtf, 1e-4) * safety + 10.0
    while rclpy.ok() and sim_now(n) - s0 < sim_secs:
        if time.time() - w0 > ceiling:
            raise SystemExit(f"FAIL: wall ceiling {ceiling:.0f}s hit inside a "
                             f"{sim_secs:g} sim-s window — RTF collapsed mid-run?")
        yield sim_now(n) - s0
        rclpy.spin_once(n, timeout_sec=tick)
# ── end sim-time helpers ─────────────────────────────────────────────────────


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

    # sim clock first: every window below is SIM-time so the gate is RTF-robust.
    wait_for_clock(n)
    rtf = measure_rtf(n)

    # --- wait for a NON-BLANK frame from each camera (30 SIM-s) ---
    # Tolerate transient startup blanks: the gz camera emits a few uniform frames
    # before the GPU render warms up, so grabbing the *first* frame races and false-
    # fails. The callback keeps the latest frame, so we wait until both are non-blank
    # (or time out — a persistent blank is then a real render-bug-#8 failure).
    for _ in sim_window(n, 30.0, rtf, tick=0.1):
        if (frames[0] is not None and frames[1] is not None
                and frame_std(frames[0]) >= MIN_STD and frame_std(frames[1]) >= MIN_STD):
            break
    for c in (0, 1):
        if frames[c] is None:
            fails.append(f"no frame on {CAM[c]} after 30 sim-s at RTF≈{rtf:.2f} "
                         f"(sim up? camera_{c} bridged?)")
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
    # beat before the other's). So collect ~3 SIM-s of stamps (≈30 frames of the 10 Hz
    # cameras at any RTF) and take the BEST cross-match — that is what OpenVINS does
    # to pair the stereo frames, and it's phase-robust.
    for _ in sim_window(n, 3.0, rtf, tick=0.05):
        pass
    if stamps[0] and stamps[1]:
        dms = min(abs(a - b) for a in stamps[0][-15:] for b in stamps[1][-15:]) * 1e3
    else:
        dms = float("inf")
    print(f"  stereo sync: best pair |Δt| = {dms:.2f} ms  [{'OK' if dms <= MAX_SYNC_MS else 'FAIL'}]")
    if dms > MAX_SYNC_MS:
        fails.append(f"stereo pair desynced (best |Δt| {dms:.1f} ms > {MAX_SYNC_MS}) — cam0/cam1 topic/rate mismatch?")

    # --- 4. VIO LIVE: jerk-start, then wait for /odomimu (all SIM-time windows —
    # the jabs must be real sim accel transients or static init never fires) ---
    def send(vx):
        m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "base_link"; m.twist.linear.x = float(vx); drive.publish(m)

    if ov["msg"] is None:                       # only jerk if not already initialised
        for dur, vx in ((2.0, 0.0), (0.4, 1.2), (0.4, 0.0), (0.4, 1.2), (0.5, 0.0)):
            for _ in sim_window(n, dur, rtf):
                send(vx)
    for _ in sim_window(n, VIO_WAIT_S, rtf):
        if ov["msg"] is not None:
            break
        send(0.3)                               # keep nudging (sustained motion)
    send(0.0)
    live = ov["msg"] is not None
    print(f"  OpenVINS {ODOM}: {'publishing' if live else 'SILENT'}  [{'OK' if live else 'FAIL'}]")
    if not live:
        fails.append(f"no {ODOM} within {VIO_WAIT_S:.0f} sim-s of jerk-start at "
                     f"RTF≈{rtf:.2f} — OpenVINS stereo init failed")

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
