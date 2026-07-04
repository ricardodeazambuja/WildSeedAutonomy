#!/usr/bin/env python3
"""Publish a correctly-stamped geometry_msgs/TwistStamped for a fixed duration.

Why this exists (a real ROS gotcha): the diff_drive controller runs on sim time
(use_sim_time) and rejects commands whose header.stamp is too old. `ros2 topic pub`
stamps every message with 0, so once sim time has advanced past the controller's
cmd_vel timeout, every bare-CLI command is silently dropped and the robot won't
move (it only "works" right after a fresh bring-up, while sim time is still near 0).
A node with use_sim_time:=true can stamp each message with clock.now() — sim time —
so the command is always fresh. This is the reliable teleop publish path (the same
thing teleop_twist_keyboard does with stamped:=true).

Usage: n1_drive.py <topic> <linear_x> <angular_z> <duration_s> [rate_hz]
Used by scripts/n1_worker.sh (N1 teleop demo).
"""
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped


def main():
    if len(sys.argv) < 5:
        print("usage: n1_drive.py <topic> <lin_x> <ang_z> <dur_s> [rate_hz]",
              file=sys.stderr)
        return 2
    topic = sys.argv[1]
    lx, az, dur = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    rate = float(sys.argv[5]) if len(sys.argv) > 5 else 20.0

    rclpy.init()
    node = rclpy.create_node(
        "n1_drive",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    pub = node.create_publisher(TwistStamped, topic, 10)

    # Wait for the sim clock to start (stamping with 0 would defeat the purpose).
    t0 = time.time()
    while rclpy.ok() and node.get_clock().now().nanoseconds == 0 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    period = 1.0 / rate
    for _ in range(int(dur * rate)):
        if not rclpy.ok():
            break
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()   # sim-time "now"
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = lx
        msg.twist.angular.z = az
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
