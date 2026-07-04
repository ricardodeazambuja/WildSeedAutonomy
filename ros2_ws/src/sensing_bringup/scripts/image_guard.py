#!/usr/bin/env python3
"""Blank-frame guard for the VIO camera feed (PLAN M3 sub-step).

Sits in front of OpenVINS: republishes camera frames that have real content and
DROPS near-uniform ("blank") frames. The sim's OAK-D occasionally renders a solid-
colour frame at certain robot yaw angles (a gz/ogre2 frustum-culling artefact —
sim-debugging-notes #8); a KLT-tracking VIO that ingests such a frame loses all its
tracks and the estimate diverges (the §14 runtime-texture failure class). Dropping
the frame instead makes OpenVINS simply coast on the IMU through the gap and re-lock
when real frames resume — the correct, robust behaviour.

"Blank" = near-zero spatial variance. A real scene has per-channel std ~30-65; a
solid fill (e.g. RGB 138,138,0 or the blue background) has std ~0. We gate on the
max per-channel std so a uniform fill of ANY colour is caught.

The original Image message is republished UNCHANGED (same stamp/encoding/frame_id)
so OpenVINS' time sync is untouched — we only decide pass/drop.

Run (in the fusion image, which has cv_bridge/numpy):
  python3 image_guard.py --ros-args -p input_topic:=... -p output_topic:=... -p min_std:=5.0
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "mono16": 1}


class ImageGuard(Node):
    def __init__(self):
        super().__init__("image_guard")
        g = self.declare_parameter
        self.in_topic = g("input_topic", "/a200_0000/sensors/camera_0/color/image").value
        self.out_topic = g("output_topic",
                           "/a200_0000/sensors/camera_0/color/image_guarded").value
        # Blank threshold (per-channel std, 0-255). Real frames ~30-65, blanks ~0.
        self.min_std = float(g("min_std", 5.0).value)

        # Match the sim camera's QoS (RELIABLE/VOLATILE) so we connect to it and feed
        # OpenVINS the same way it already consumed the raw topic.
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(Image, self.out_topic, qos)
        self.create_subscription(Image, self.in_topic, self.on_image, qos)

        self._passed = 0
        self._dropped = 0
        self.get_logger().info(
            f"image_guard: '{self.in_topic}' -> '{self.out_topic}' "
            f"(drop frames with max per-channel std < {self.min_std})")

    def on_image(self, msg: Image):
        c = _CHANNELS.get(msg.encoding)
        if c is None:                       # unknown encoding: pass through, don't gate
            self.pub.publish(msg)
            return
        n = msg.height * msg.width * c
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if buf.size < n:                    # malformed: pass through rather than block
            self.pub.publish(msg)
            return
        # max per-channel spatial std — ~0 for a uniform fill of any colour
        std = float(buf[:n].reshape(-1, c).std(axis=0).max())
        if std < self.min_std:
            self._dropped += 1
            if self._dropped % 20 == 1:     # throttle: the culled-frame bands are bursty
                self.get_logger().warn(
                    f"dropping blank frame (std={std:.1f}); dropped={self._dropped} "
                    f"passed={self._passed}")
            return
        self._passed += 1
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
