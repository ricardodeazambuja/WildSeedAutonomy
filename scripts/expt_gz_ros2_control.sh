#!/usr/bin/env bash
# expt_gz_ros2_control.sh — the isolated controlled experiment that root-caused
# the missing controllers. In a throwaway container it: generates the Clearpath
# description, expands it with is_sim:=true (sim hardware + the gz_ros2_control
# system plugin), starts a gz server, spawns the robot, and reports whether a
# controller_manager appears. This is the "slice it into smaller pieces" probe
# that separated "does gz_ros2_control work" from "is the launch feeding it right".
# See docs/sim-debugging-notes.md.
#
#   ./scripts/expt_gz_ros2_control.sh                 # empty world (clean baseline)
#   ./scripts/expt_gz_ros2_control.sh /opt/ros/jazzy/share/clearpath_gz/worlds/pipeline.sdf
#
# Env: IMG (default sensing-node/sim:local), ROS_DOMAIN_ID (default 59).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-empty.sdf}"
IMG="${IMG:-sensing-node/sim:local}"
CFG="$ROOT/ros2_ws/src/sensing_bringup/config"

docker run --rm -i --gpus all --network host --ipc host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}" \
  -e GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib \
  -e GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/clearpath_gz/meshes:/opt/ros/jazzy/share \
  -e WORLD="$WORLD" \
  -v "$CFG":/csrc:ro "$IMG" bash <<'INNER'
set -uo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /clearpath && cp /csrc/robot.yaml /clearpath/
for g in clearpath_generator_common/generate_description \
         clearpath_generator_common/generate_semantic_description \
         clearpath_generator_gz/generate_launch \
         clearpath_generator_gz/generate_param; do
  "/opt/ros/jazzy/lib/$g" -s /clearpath/ >/dev/null 2>&1 || true
done
xacro /clearpath/robot.urdf.xacro is_sim:=true \
  gazebo_controllers:=/clearpath/platform/config/control.yaml > /tmp/r.urdf 2>/dev/null
echo "URDF: $(wc -c </tmp/r.urdf) bytes | gz_ros2_control refs: $(grep -c gz_ros2_control /tmp/r.urdf) | hardware: $(grep -oE 'GazeboSimSystem|A200Hardware' /tmp/r.urdf | head -1)"
timeout 70 gz sim -s -r "$WORLD" >/tmp/gz.log 2>&1 &
sleep 12
timeout 25 ros2 run ros_gz_sim create -string "$(cat /tmp/r.urdf)" -name iso -z 0.5 >/tmp/c.log 2>&1
echo "create rc=$? ($(tail -1 /tmp/c.log))"
sleep 12
echo "RESULT controller_manager service:"
timeout 8 ros2 service list 2>/dev/null | grep -m1 controller_manager || echo "  NONE (controllers did not load)"
echo "gz plugin log (tail):"
grep -iE "GazeboSim|gz_ros2_control|Error opening|controller_manager|Failed to load" /tmp/gz.log | tail -5
pkill -f "gz sim" 2>/dev/null || true
INNER
