#!/usr/bin/env python3
"""S1 corner logger — sample KLT-trackable corner counts ALONG a drive.

The continuous version of check_sim_texture.py's one-shot probe: texture is a
*route* property (the M4 alpine divergence was caused by starved patches
mid-drive that a spawn-point gate missed), so S1 logs the corner count over
the whole drive and correlates it with the estimator error afterwards — the
mechanism evidence for the texture A/B, not just the outcome.

Subscribes to the camera, samples every SAMPLE_DT sim-seconds (use_sim_time).
Each sample takes a CONSECUTIVE frame pair and measures what a VIO front-end
actually consumes: corner count AND KLT forward-backward match survival — the
S1 uniform ground is an *aliasing* worst case, where Shi-Tomasi corners are
plentiful but ambiguous, so count alone misses the failure (measured: mean
167 corners on texture 0.0 while OpenVINS refused to init).

CSV rows:  t_sim, n_shi, n_fast, lap_var, klt_surv, klt_err_px
  klt_surv   fraction of corners that track F1->F2 and survive the backward
             check (|fb - start| < 1 px)
  klt_err_px median forward-backward error of survivors

Flushes per row; runs until killed (m4_lio_eval.sh CORNER_LOG=1 starts it
before the drive and pkills it after).

Usage (inside the fusion container):
  python3 s1_corner_log.py <out_csv> [topic] [sample_dt_sim_s]
"""
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else "/results/s1_corners.csv"
TOPIC = sys.argv[2] if len(sys.argv) > 2 else \
    "/a200_0000/sensors/camera_0/color/image"
SAMPLE_DT = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0


def main():
    rclpy.init()
    n = rclpy.create_node("s1_corner_log", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    bridge = CvBridge()
    latest = {"img": None}
    n.create_subscription(
        Image, TOPIC,
        lambda m: latest.__setitem__("img", bridge.imgmsg_to_cv2(m, "bgr8")), 5)

    def grab_next_frame(prev):
        """Spin until a frame OBJECT different from `prev` arrives (~next
        camera frame at 10-30 Hz), or give up after ~2 wall-s."""
        import time as _time
        t0 = _time.time()
        while _time.time() - t0 < 2.0:
            rclpy.spin_once(n, timeout_sec=0.05)
            if latest["img"] is not None and latest["img"] is not prev:
                return latest["img"]
        return None

    f = open(OUT, "w", buffering=1)
    f.write("t_sim,n_shi,n_fast,lap_var,klt_surv,klt_err_px\n")
    last_t = -1e9
    print(f"logging {TOPIC} -> {OUT} every {SAMPLE_DT} sim-s", flush=True)
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        t = n.get_clock().now().nanoseconds / 1e9
        if latest["img"] is None or t - last_t < SAMPLE_DT:
            continue
        last_t = t
        f1 = latest["img"]
        g1 = cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(g1, maxCorners=1000,
                                          qualityLevel=0.01, minDistance=8)
        n_shi = 0 if corners is None else len(corners)
        n_fast = len(cv2.FastFeatureDetector_create(threshold=20)
                     .detect(g1, None))
        lap_var = cv2.Laplacian(g1, cv2.CV_64F).var()
        # KLT forward-backward survival on a consecutive frame pair — the
        # aliasing metric: ambiguous corners track forward but fail the
        # backward check.
        surv, err = "", ""
        f2 = grab_next_frame(f1)
        if f2 is not None and corners is not None and len(corners) >= 8:
            import numpy as np
            g2 = cv2.cvtColor(f2, cv2.COLOR_BGR2GRAY)
            p1 = corners.astype("float32")
            p2, st1, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p1, None)
            p1b, st2, _ = cv2.calcOpticalFlowPyrLK(g2, g1, p2, None)
            fb = np.linalg.norm((p1 - p1b).reshape(-1, 2), axis=1)
            ok = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0)
            surv = f"{ok.mean():.3f}"
            err = f"{np.median(fb[ok]):.3f}" if ok.any() else ""
        f.write(f"{t:.3f},{n_shi},{n_fast},{lap_var:.1f},{surv},{err}\n")


if __name__ == "__main__":
    main()
