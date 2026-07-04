#!/usr/bin/env python3
"""Append a per-point relative timestamp to a Gazebo gpu_lidar PointCloud2.

Gazebo's gpu_lidar publishes sensor_msgs/PointCloud2 with x,y,z,intensity,ring
but NO per-point time field, so lidar-odometry deskewing is disabled: LIO-SAM
warns "deskew function disabled, system will drift significantly!", and KISS-ICP
falls back to deskew-off. This node intercepts the cloud, computes each point's
time within the sweep, appends it as a field, and republishes.

See docs/kiss-icp-failure-modes.md (#3) and PLAN §17.2.

Methods:
  column  : organized cloud (height>1) — every point in a column shares an
            azimuth, so time = col / width * scan_period (t in [0, T)). Exact &
            cheapest for a spinning lidar emitted in scan order.
  azimuth : unorganized cloud — time from atan2(y,x) vs a reference azimuth,
            normalised by 2*pi * scan_period.
  auto    : column when the cloud is organized, else azimuth.

Field name/type must match the CONSUMER's sensor profile, or it won't be found:
  velodyne : name 'time', FLOAT32, seconds       (LIO-SAM default; KISS-ICP ok)
  ouster   : name 't',    UINT32,  nanoseconds

Assumes little-endian point data (true for gz on x86). Vectorised with numpy so
it keeps up with dense clouds (e.g. 128x2048).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class TimestampInjector(Node):
    def __init__(self):
        super().__init__('gz_lidar_timestamp')
        self.declare_parameter('input_topic', '/cloud_in')
        self.declare_parameter('output_topic', '/cloud_with_time')
        self.declare_parameter('scan_rate_hz', 10.0)
        self.declare_parameter('profile', 'velodyne')      # velodyne | ouster
        self.declare_parameter('method', 'auto')           # auto | column | azimuth
        self.declare_parameter('time_reference', 'start')  # start | end
        self.declare_parameter('clockwise', False)         # spin direction
        self.declare_parameter('azimuth_start_rad', -math.pi)

        self.period = 1.0 / float(self.get_parameter('scan_rate_hz').value)
        self.profile = str(self.get_parameter('profile').value)
        self.method = str(self.get_parameter('method').value)
        self.time_ref = str(self.get_parameter('time_reference').value)
        self.cw = bool(self.get_parameter('clockwise').value)
        self.az0 = float(self.get_parameter('azimuth_start_rad').value)

        if self.profile == 'ouster':
            self.field_name = 't'
            self.field_dt = PointField.UINT32
            self.np_dt = np.dtype('<u4')
        else:
            self.field_name = 'time'
            self.field_dt = PointField.FLOAT32
            self.np_dt = np.dtype('<f4')

        in_t = str(self.get_parameter('input_topic').value)
        out_t = str(self.get_parameter('output_topic').value)
        self.pub = self.create_publisher(PointCloud2, out_t, 10)
        self.sub = self.create_subscription(PointCloud2, in_t, self.cb, 10)
        self._warned = False
        self.get_logger().info(
            f"injecting '{self.field_name}' ({self.profile}) {in_t} -> {out_t}; "
            f"method={self.method}, period={self.period * 1e3:.1f} ms")

    def cb(self, msg: PointCloud2):
        # Already has the field (e.g. a real Ouster/Velodyne)? Pass through.
        if any(f.name == self.field_name for f in msg.fields):
            self.pub.publish(msg)
            return
        n = msg.width * msg.height
        if n == 0:
            self.pub.publish(msg)
            return

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

        method = self.method
        if method == 'auto':
            method = 'column' if (msg.height > 1 and msg.width > 1) else 'azimuth'

        if method == 'column':
            col = np.arange(n) % msg.width
            if self.cw:
                col = (msg.width - 1) - col
            # width azimuth bins span the full sweep period → t in [0, T).
            t = (col / msg.width) * self.period
        else:
            x = self._read_f32(msg, raw, 'x')
            y = self._read_f32(msg, raw, 'y')
            if x is None or y is None:
                if not self._warned:
                    self.get_logger().warn(
                        'azimuth method needs x/y fields; emitting zero times')
                    self._warned = True
                t = np.zeros(n, dtype=np.float64)
            else:
                az = np.arctan2(y, x)
                sign = -1.0 if self.cw else 1.0
                rel = np.mod(sign * (az - self.az0), 2.0 * math.pi)
                t = np.nan_to_num((rel / (2.0 * math.pi)) * self.period, nan=0.0)

        if self.time_ref == 'end':
            t = t - self.period

        if self.field_dt == PointField.UINT32:
            tb = np.clip(t * 1e9, 0, np.iinfo(np.uint32).max).astype(self.np_dt)
        else:
            tb = t.astype(self.np_dt)
        tb = np.ascontiguousarray(tb).view(np.uint8).reshape(n, 4)

        new = np.ascontiguousarray(np.hstack([raw, tb]))

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.is_bigendian = msg.is_bigendian
        out.is_dense = msg.is_dense
        out.fields = list(msg.fields) + [
            PointField(name=self.field_name, offset=msg.point_step,
                       datatype=self.field_dt, count=1)]
        out.point_step = msg.point_step + 4
        out.row_step = out.point_step * msg.width
        out.data = new.tobytes()
        self.pub.publish(out)

    @staticmethod
    def _read_f32(msg, raw, name):
        """Read a FLOAT32 field column out of the raw (N, point_step) byte array."""
        for f in msg.fields:
            if f.name == name:
                off = f.offset
                col = np.ascontiguousarray(raw[:, off:off + 4])
                return np.frombuffer(col.tobytes(), dtype='<f4')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = TimestampInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
