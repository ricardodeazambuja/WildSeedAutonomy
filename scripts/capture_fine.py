#!/usr/bin/env python3
"""Capture std from bare cameras at fine angles /c{a}/image (a in 0..90 step 10)."""
import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

ANGLES = list(range(0, 91, 10))
rclpy.init()
n = Node("capture_fine")
q = QoSProfile(depth=5); q.reliability = ReliabilityPolicy.RELIABLE; q.durability = DurabilityPolicy.VOLATILE
S = {}
def mk(a):
    def cb(m): S[a] = m
    return cb
for a in ANGLES:
    n.create_subscription(Image, f"/c{a}/image", mk(a), q)

t0 = time.time()
while len(S) < len(ANGLES) and time.time() - t0 < 25:
    rclpy.spin_once(n, timeout_sec=0.2)

print(f"{'yaw':>5}{'std':>9}  state")
for a in ANGLES:
    m = S.get(a)
    if m is None:
        print(f"{a:>5}      NO FRAME"); continue
    ch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(m.encoding, 3)
    arr = np.frombuffer(bytes(m.data), np.uint8)
    npx = m.height * m.width * ch
    std = float(arr[:npx].reshape(-1, ch).std(0).max()) if arr.size >= npx else -1
    print(f"{a:>5}{std:>9.1f}  {'BLANK' if 0 <= std < 5 else 'SCENE'}")
