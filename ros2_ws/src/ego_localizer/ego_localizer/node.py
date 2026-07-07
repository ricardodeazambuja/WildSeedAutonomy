"""ego_localizer ROS node — thin ROS plumbing around PlanarPoseEstimator.

Fuses the spine IMU + wheel odometry (+ GNSS) into one fused `nav_msgs/Odometry`.
Two odom modes (PLAN §5):

- `absolute` (default, M3 foundation): wheel odom consumed as an absolute pose —
  the fused estimate tracks odom; good when odom is the trusted local reference.
- `relative` (the GPS-denied keystone, M5): wheel odom consumed as a *twist*
  (relative velocity) that dead-reckons and drifts, with **GNSS** as the
  droppable absolute fix that bounds it. Toggle GNSS at runtime by publishing
  `std_msgs/Bool` on `~/set_gps_enabled` — false = "GPS denied" (drift), true =
  reacquire (snap back). This is the chart-#1 demo.

GNSS (`sensor_msgs/NavSatFix`) is converted to a local ENU plane whose origin is
the first received fix (small-area equirectangular approx; the sim datum is the
world `<spherical_coordinates>`). Frames: publishes `output_frame`→`child_frame`;
TF off by default so it never fights the sim's own odom→base_link broadcaster.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool

from ego_localizer.estimator import PlanarPoseEstimator, wrap

_EARTH_R = 6378137.0   # WGS-84 mean radius (m), good enough for a local ENU plane


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EgoLocalizer(Node):
    def __init__(self):
        super().__init__('ego_localizer')
        g = self.declare_parameter
        self.imu_topic = g('imu_topic', '/a200_0000/sensors/imu_0/data').value
        self.odom_topic = g('odom_topic', '/a200_0000/platform/odom').value
        self.gps_topic = g('gps_topic', '/a200_0000/sensors/gps_0/fix').value
        self.out_topic = g('output_topic', '/ego_localizer/odom').value
        self.out_frame = g('output_frame', 'odom').value
        self.child_frame = g('child_frame', 'base_link').value
        self.publish_rate = float(g('publish_rate', 30.0).value)
        self.publish_tf = bool(g('publish_tf', False).value)
        self.odom_mode = str(g('odom_mode', 'absolute').value)   # absolute | relative
        self.gps_enabled = bool(g('gps_enabled', True).value)
        # Frontends: wheel odom and/or visual-inertial odometry (OpenVINS, M3)
        # and/or lidar odometry (KISS-ICP, M4). For the M3/M4 charts we run one
        # frontend ALONE (use_odom=false) so the result reflects that frontend,
        # not the sim's near-perfect wheel odom propping it up.
        self.use_odom = bool(g('use_odom', True).value)
        self.use_visual = bool(g('use_visual', False).value)
        self.visual_topic = g('visual_topic', '/odomimu').value
        self.use_lidar = bool(g('use_lidar', False).value)
        self.lidar_topic = g('lidar_topic', '/kiss/odometry').value
        # Minimum sim-time baseline between consumed lidar poses. Scan-to-scan
        # ICP poses carry cm-level registration noise; at a slow UGV's 2-5 cm
        # per-scan motion the per-delta SNR is < 1, so differentiating every
        # scan feeds the EKF noise (measured: 15 Hz deltas read 1.9 m/s mean on
        # a 0.5 m/s drive). Spanning >= this baseline restores SNR without
        # touching the frontend. 0 = consume every pose (the M3 VIO behaviour).
        self.lidar_min_dt = float(g('lidar_min_dt', 0.0).value)
        # Seed the EKF at the origin on the first frontend message (no GPS needed).
        # Fine for evaluation: eval_tools ATE does a Umeyama alignment, which removes
        # any constant origin/heading offset — only trajectory SHAPE is scored.
        self.seed_at_origin = bool(g('seed_at_origin', False).value)
        self.est = PlanarPoseEstimator(
            sigma_a=float(g('sigma_a', 1.0).value),
            sigma_alpha=float(g('sigma_alpha', 1.0).value))
        self.sigma_imu_yaw = float(g('sigma_imu_yaw', 0.05).value)
        self.sigma_imu_wz = float(g('sigma_imu_wz', 0.02).value)
        self.sigma_odom_xy = float(g('sigma_odom_xy', 0.1).value)
        self.sigma_odom_yaw = float(g('sigma_odom_yaw', 0.1).value)
        self.sigma_odom_v = float(g('sigma_odom_v', 0.05).value)
        self.sigma_odom_wz = float(g('sigma_odom_wz', 0.02).value)
        self.sigma_gnss = float(g('sigma_gnss', 0.5).value)
        self.sigma_visual_v = float(g('sigma_visual_v', 0.05).value)
        self.sigma_visual_wz = float(g('sigma_visual_wz', 0.02).value)
        self.sigma_lidar_v = float(g('sigma_lidar_v', 0.05).value)
        self.sigma_lidar_wz = float(g('sigma_lidar_wz', 0.02).value)
        # GPS course-over-ground anchors heading to ENU (§17.4 fix, relative mode):
        # only trusted when the robot has moved at least this far between fixes.
        self.gnss_course_min_move = float(g('gnss_course_min_move', 0.25).value)
        self.sigma_gnss_course = float(g('sigma_gnss_course', 0.15).value)

        sensor_qos = QoSProfile(depth=20)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Imu, self.imu_topic, self.on_imu, sensor_qos)
        if self.use_odom:
            self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        if self.use_visual:
            self.create_subscription(Odometry, self.visual_topic, self.on_visual, sensor_qos)
        if self.use_lidar:
            self.create_subscription(Odometry, self.lidar_topic, self.on_lidar, sensor_qos)
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, sensor_qos)
        self.create_subscription(Bool, '~/set_gps_enabled', self.on_gps_toggle, 10)
        self.pub = self.create_publisher(Odometry, self.out_topic, 10)
        if self.publish_tf:
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self._last_t = None
        self._last_yaw = 0.0          # latest IMU yaw (absolute mode seeding)
        self._enu0 = None             # (lat0, lon0) ENU origin
        self._prev_enu = None         # previous ENU fix, for course-over-ground
        self._prev_vio = None         # previous VIO pose (X,Y,yaw) for body-frame deltas
        self._prev_lio = None         # previous lidar-odom pose, same delta scheme
        srcs = [f"IMU '{self.imu_topic}'"]
        if self.use_odom:
            srcs.append(f"odom '{self.odom_topic}'")
        if self.use_visual:
            srcs.append(f"VIO '{self.visual_topic}'")
        if self.use_lidar:
            srcs.append(f"LIO '{self.lidar_topic}'")
        if self.odom_mode == 'relative':
            srcs.append(f"GNSS '{self.gps_topic}'")
        self.get_logger().info(
            f"ego_localizer[{self.odom_mode}]: " + " + ".join(srcs) +
            f" -> '{self.out_topic}' ({self.out_frame}->{self.child_frame}, "
            f"gps_enabled={self.gps_enabled})")

    # ── time / predict ─────────────────────────────────────────────────────────
    def _predict_to_now(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        if dt > 0:
            self.est.predict(dt)
            self._last_t = now

    def _seed(self, px, py, yaw):
        self.est.seed_pose(px, py, yaw)
        self._last_t = self.get_clock().now().nanoseconds * 1e-9

    # ── callbacks ──────────────────────────────────────────────────────────────
    def on_imu(self, msg: Imu):
        self._last_yaw = yaw_from_quat(msg.orientation)
        if not self.est._initialised:
            if self.seed_at_origin:        # seed once, no GPS needed (M3 VIO config)
                self._seed(0.0, 0.0, 0.0)
            return
        self._predict_to_now()
        if self.odom_mode == 'relative':
            # absolute IMU yaw is in the gz world frame, NOT the GPS ENU frame
            # (§17.4) — use only the (frame-independent) yaw-RATE here; heading is
            # anchored to ENU by the GPS course in on_gps.
            self.est.imu_rate_update(msg.angular_velocity.z, sigma_wz=self.sigma_imu_wz)
        else:
            self.est.imu_update(self._last_yaw, msg.angular_velocity.z,
                                sigma_yaw=self.sigma_imu_yaw, sigma_wz=self.sigma_imu_wz)

    def on_odom(self, msg: Odometry):
        if self.odom_mode == 'absolute':
            p = msg.pose.pose
            yaw = yaw_from_quat(p.orientation)
            if not self.est._initialised:
                self._seed(p.position.x, p.position.y, yaw)
                return
            self._predict_to_now()
            self.est.odom_update(p.position.x, p.position.y, yaw,
                                 sigma_xy=self.sigma_odom_xy,
                                 sigma_yaw=self.sigma_odom_yaw)
        else:  # relative: odom twist is a velocity (dead-reckoning) source
            if not self.est._initialised:
                return                  # wait for the first GNSS fix to set the origin
            self._predict_to_now()
            self.est.odom_twist_update(msg.twist.twist.linear.x,
                                       msg.twist.twist.angular.z,
                                       sigma_v=self.sigma_odom_v,
                                       sigma_wz=self.sigma_odom_wz)

    def on_visual(self, msg: Odometry):
        """OpenVINS VIO odometry → relative body-frame motion (M3, loosely coupled).

        OpenVINS publishes an absolute pose in its OWN drifting frame. We feed only
        the body-frame increment between consecutive VIO poses (frame-offset cancels;
        see estimator.visual_delta_update). dt comes from the message stamps.
        """
        p = msg.pose.pose
        X, Y, Yaw = p.position.x, p.position.y, yaw_from_quat(p.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._prev_vio is None:
            self._prev_vio = (X, Y, Yaw, t)
            return
        Xp, Yp, Yawp, tp = self._prev_vio
        self._prev_vio = (X, Y, Yaw, t)
        if not self.est._initialised:
            return
        dt = t - tp
        if dt <= 0:
            return
        dX, dY = X - Xp, Y - Yp                    # delta in the VIO world frame
        c, s = math.cos(Yawp), math.sin(Yawp)      # rotate into the prev VIO body frame
        dx_body, dy_body = c * dX + s * dY, -s * dX + c * dY
        self._predict_to_now()
        self.est.visual_delta_update(dx_body, dy_body, wrap(Yaw - Yawp), dt,
                                     sigma_v=self.sigma_visual_v,
                                     sigma_wz=self.sigma_visual_wz)

    def on_lidar(self, msg: Odometry):
        """KISS-ICP lidar odometry → relative body-frame motion (M4, loosely coupled).

        Same scheme as `on_visual`: KISS-ICP publishes an absolute pose in its own
        drifting lidar-odometry frame; we feed only the body-frame increment between
        consecutive poses (frame offset cancels). The Ouster is mounted rpy 0,0,0
        (robot.yaml), so the lidar frame is axis-aligned with base_link and the
        planar delta transfers directly. dt comes from the message stamps.
        """
        p = msg.pose.pose
        X, Y, Yaw = p.position.x, p.position.y, yaw_from_quat(p.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._prev_lio is None:
            self._prev_lio = (X, Y, Yaw, t)
            return
        Xp, Yp, Yawp, tp = self._prev_lio
        dt = t - tp
        if 0 <= dt < self.lidar_min_dt:
            return                      # keep the anchor; wait for a longer baseline
        self._prev_lio = (X, Y, Yaw, t)
        if not self.est._initialised:
            return
        if dt <= 0:
            return
        dX, dY = X - Xp, Y - Yp                    # delta in the lidar-odom frame
        c, s = math.cos(Yawp), math.sin(Yawp)      # rotate into the prev body frame
        dx_body, dy_body = c * dX + s * dY, -s * dX + c * dY
        self._predict_to_now()
        self.est.lidar_delta_update(dx_body, dy_body, wrap(Yaw - Yawp), dt,
                                    sigma_v=self.sigma_lidar_v,
                                    sigma_wz=self.sigma_lidar_wz)

    def on_gps(self, msg: NavSatFix):
        if self._enu0 is None:
            self._enu0 = (msg.latitude, msg.longitude)
        lat0, lon0 = self._enu0
        east = math.radians(msg.longitude - lon0) * _EARTH_R * math.cos(math.radians(lat0))
        north = math.radians(msg.latitude - lat0) * _EARTH_R

        # course over ground from consecutive fixes — the ENU heading when moving.
        course = None
        if self._prev_enu is not None:
            de, dn = east - self._prev_enu[0], north - self._prev_enu[1]
            if math.hypot(de, dn) >= self.gnss_course_min_move:
                course = math.atan2(dn, de)
        self._prev_enu = (east, north)

        if self.odom_mode != 'relative':
            return                      # absolute mode uses odom pose, not GNSS

        if not self.est._initialised:
            if course is not None:      # need motion to know the ENU heading
                self._seed(east, north, course)
            return
        if self.gps_enabled:
            self._predict_to_now()
            self.est.gnss_update(east, north, sigma_xy=self.sigma_gnss)
            if course is not None:      # anchor heading to ENU (§17.4 fix)
                self.est.heading_update(course, sigma_yaw=self.sigma_gnss_course)

    def on_gps_toggle(self, msg: Bool):
        if msg.data != self.gps_enabled:
            self.get_logger().info(
                f"GPS {'ENABLED (reacquire)' if msg.data else 'DENIED (dropout)'}")
        self.gps_enabled = msg.data

    def on_timer(self):
        if not self.est._initialised:
            return
        self._predict_to_now()
        self.publish()

    # ── output ─────────────────────────────────────────────────────────────────
    def publish(self):
        x = self.est.state
        P = self.est.covariance
        stamp = self.get_clock().now().to_msg()
        px, py, yaw, vx, vy, wz = x

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.out_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = vx * math.cos(yaw) + vy * math.sin(yaw)
        odom.twist.twist.angular.z = wz
        idx = {0: 0, 1: 1, 2: 5}
        for a, ia in idx.items():
            for b, ib in idx.items():
                odom.pose.covariance[ia * 6 + ib] = P[a, b]
        self.pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.out_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EgoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
