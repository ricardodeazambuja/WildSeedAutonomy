#!/usr/bin/env bash
# diag_sim.sh — bring up the headless Husky sim and report its full health:
# sensors, controller_manager, joint_states, /clock, odom, and a teleop movement
# test. Run on the host where the images are built (server OR laptop). This
# reproduces the verification used while debugging — see docs/sim-debugging-notes.md.
#
#   ./scripts/diag_sim.sh
#
# Env: NS (robot namespace, default a200_0000).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT/.env"; [ -f "$ENV_FILE" ] || ENV_FILE="$ROOT/.env.example"
DC=(docker compose --env-file "$ENV_FILE" -f "$ROOT/docker/compose.yaml")
NS="${NS:-a200_0000}"

ex() { "${DC[@]}" --profile compute exec -T husky bash -lc "source /opt/ros/jazzy/setup.bash; $*" 2>/dev/null; }
cleanup() { echo "== teardown =="; "${DC[@]}" --profile compute down >/dev/null 2>&1; }
trap cleanup EXIT

echo "== up husky (compute) =="
"${DC[@]}" --profile compute up -d husky >/dev/null 2>&1

echo "== wait for controller_manager (condition, not a fixed sleep; max ~140s) =="
for i in $(seq 1 70); do
  ex "ros2 node list" | grep -q controller_manager && { echo "  controller_manager up after ~$((i * 2))s"; break; }
  sleep 2
done

echo "-- sensors present --"
ex "ros2 topic list | grep -E 'lidar3d_0/points|camera_0/color/image|imu_0/data' | sort"
echo "-- lidar rate --";          ex "timeout 6 ros2 topic hz /$NS/sensors/lidar3d_0/points 2>&1 | grep -m1 'average rate'"
echo "-- controller_manager --";  ex "ros2 node list | grep controller_manager || echo MISSING"
echo "-- joint_states rate --";   ex "timeout 6 ros2 topic hz /$NS/platform/joint_states 2>&1 | grep -m1 'average rate'"
echo "-- /clock publishers (want >0) --"; ex "ros2 topic info /clock | grep 'Publisher count'"
echo "-- /clock advancing --";    ex "timeout 6 ros2 topic hz /clock 2>&1 | grep -m1 'average rate' || echo 'NOT advancing'"
echo "-- odom rate --";           ex "timeout 6 ros2 topic hz /$NS/platform/odom 2>&1 | grep -m1 'average rate' || echo 'no odom'"

echo "== movement test: teleop forward 6s, odom x before vs after =="
ex "timeout 4 ros2 topic echo /$NS/platform/odom --field pose.pose.position.x --once" | sed 's/^/  x before: /'
ex "timeout 6 ros2 topic pub -r 20 /$NS/joy_teleop/cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: base_link}, twist: {linear: {x: 0.8}}}' >/dev/null 2>&1; true"
ex "timeout 4 ros2 topic echo /$NS/platform/odom --field pose.pose.position.x --once" | sed 's/^/  x after:  /'
echo "(x should grow ~linear.x * 6s; unchanged ⇒ regression of the /clock bridge — see docs/sim-debugging-notes.md #7)"
