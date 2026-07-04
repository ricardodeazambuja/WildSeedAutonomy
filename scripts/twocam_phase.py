#!/usr/bin/env python3
"""Log cam0 (mount 0) and cam1 (mount +45) std vs robot-yaw during rotation, then
report blank% by robot-yaw bin for each. In-phase => Husky-orientation locked;
45deg out-of-phase => world-azimuth (render bug) locked. Subscribe-only logging;
drive separately."""
import sys, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
br = CvBridge()
rclpy.init()
n = Node("twocam_phase")
q = QoSProfile(depth=5); q.reliability = ReliabilityPolicy.RELIABLE; q.durability = DurabilityPolicy.VOLATILE
S = {"c0": None, "c1": None, "yaw": None}
n.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", lambda m: S.__setitem__("c0", m), q)
n.create_subscription(Image, "/a200_0000/sensors/camera_1/color/image", lambda m: S.__setitem__("c1", m), q)


def od(m):
    o = m.pose.pose.orientation
    S["yaw"] = math.degrees(math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y*o.y+o.z*o.z)))


n.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", od, 10)

print(f"[twocam_phase] logging {DUR:.0f}s — drive a slow 360 now", flush=True)
rows = []
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.03)
    if S["yaw"] is None or S["c0"] is None or S["c1"] is None:
        continue
    s0 = float(br.imgmsg_to_cv2(S["c0"], "bgr8").reshape(-1, 3).std(0).max())
    s1 = float(br.imgmsg_to_cv2(S["c1"], "bgr8").reshape(-1, 3).std(0).max())
    rows.append((S["yaw"], s0, s1))

y = np.array([r[0] for r in rows]); s0 = np.array([r[1] for r in rows]); s1 = np.array([r[2] for r in rows])
print(f"frames={len(rows)} yaw span {y.min():.0f}..{y.max():.0f}")
print(f"{'robot_yaw':>12}{'cam0(m0) blank%':>18}{'cam1(m45) blank%':>18}")
for lo in range(-180, 180, 45):
    m = (y >= lo) & (y < lo+45)
    if m.sum():
        print(f"  [{lo:>4},{lo+45:>4})   {100*(s0[m]<5).mean():>10.0f}      {100*(s1[m]<5).mean():>10.0f}   (n={m.sum()})")
