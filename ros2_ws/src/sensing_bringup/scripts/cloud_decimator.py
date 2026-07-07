#!/usr/bin/env python3
"""Count-based PointCloud2 decimator — feeds KISS-ICP scans it can actually use.

KISS-ICP is built for the automotive motion regime: its adaptive correspondence
threshold only adapts when inter-scan motion exceeds min_motion_th (0.1 m), and
below that regime ICP's robust kernel treats real motion as error — the pose
systematically UNDER-reports translation (measured: 0.27 m/s mean on a 0.5 m/s
drive at the Ouster's 20 Hz = 2.5 cm/scan). Registering every Nth scan restores
~10 cm of true motion per registration (0.5 m/s @ 5 Hz) without touching the
sensor that other consumers (deskew experiments, future LIO fallbacks) rely on.

Count-based (not timer-based) so it is RTF-proof — the same scans are selected
at any sim speed. Same relay pattern as image_guard.py (the camera_guard).

Params: input_topic, output_topic, keep_every (default 4: 20 Hz -> 5 Hz).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudDecimator(Node):
    def __init__(self):
        super().__init__('cloud_decimator')
        g = self.declare_parameter
        self.in_topic = g('input_topic', '/a200_0000/sensors/lidar3d_0/points').value
        self.out_topic = g('output_topic', '/a200_0000/sensors/lidar3d_0/points_decimated').value
        self.keep_every = int(g('keep_every', 4).value)
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(PointCloud2, self.out_topic, qos)
        self.create_subscription(PointCloud2, self.in_topic, self.on_cloud, qos)
        self._n = 0
        self.get_logger().info(
            f"cloud_decimator: '{self.in_topic}' -> '{self.out_topic}' "
            f"(1 of every {self.keep_every})")

    def on_cloud(self, msg: PointCloud2):
        if self._n % self.keep_every == 0:
            self.pub.publish(msg)
        self._n += 1


def main(args=None):
    rclpy.init(args=args)
    node = CloudDecimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
