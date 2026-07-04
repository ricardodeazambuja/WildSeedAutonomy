#!/usr/bin/env python3
"""Subscribe to the 4 bare-camera color topics, grab one frame each, report std
(BLANK if <5). No cv2 — decode raw Image bytes with numpy."""
import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

TOPICS = {"y0": "/cy0/image", "y45": "/cy45/image", "y90": "/cy90/image", "y135": "/cy135/image"}
rclpy.init()
n = Node("capture_bare")
q = QoSProfile(depth=5); q.reliability = ReliabilityPolicy.RELIABLE; q.durability = DurabilityPolicy.VOLATILE
S = {}
def mk(k):
    def cb(m): S[k] = m
    return cb
for k, t in TOPICS.items():
    n.create_subscription(Image, t, mk(k), q)

t0 = time.time()
while len(S) < 4 and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.2)

print(f"{'cam':>6}{'kind':>10}{'std':>9}  state")
for k in ["y0", "y45", "y90", "y135"]:
    m = S.get(k)
    if m is None:
        print(f"{k:>6}{'?':>10}      NO FRAME"); continue
    ch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(m.encoding, 3)
    a = np.frombuffer(bytes(m.data), np.uint8)
    npx = m.height * m.width * ch
    std = float(a[:npx].reshape(-1, ch).std(0).max()) if a.size >= npx else -1
    kind = "DIAGONAL" if k in ("y45", "y135") else "cardinal"
    print(f"{k:>6}{kind:>10}{std:>9.1f}  {'BLANK' if 0 <= std < 5 else 'SCENE'}  (enc={m.encoding})")
