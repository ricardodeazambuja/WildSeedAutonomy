#!/bin/bash
# m4_lio_eval.sh — one-shot M4 evaluation: fresh stack -> steady-RTF gate ->
# FRESH KISS-ICP -> ego_localizer (lidar config) -> the M3-chart drive ->
# eval_tools ATE/RPE chart.
#
# Encodes the two M4 bring-up lessons (docs/m3-vio.md had the VIO versions):
#  1. Demos must start only after the world clears its slow-load transient
#     (steady RTF > 0.4) — at RTF ~0.001 the drive is meaningless and the
#     frontends init on garbage.
#  2. KISS-ICP builds its local map from scan #1 and NEVER resets — if its
#     container comes up during the transient, the garbage-registered scans
#     poison the whole run (measured: 12.8 m spurious pose before the drive
#     started, WildSeed recipe world). So it is restarted FRESH after the gate.
#     (OpenVINS is inherently safe: static init waits for the demo's jerk.)
#
# Usage: ./scripts/m4_lio_eval.sh [/results/prefix]   (default /results/m4)
# World: whatever `deploy.sh world` selected (pipeline or a WildSeed bundle).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${1:-/results/m4}"
cd "$ROOT"

./scripts/deploy.sh down
./scripts/deploy.sh up compute
./scripts/deploy.sh up lio
./scripts/deploy.sh up vio     # optional A/B stream; demo records /odomimu if present

echo "== waiting for cameras + OpenVINS + kiss node =="
for i in $(seq 1 80); do
  if docker exec sensing-node-fusion-1 bash -lc 'source /opt/ros/jazzy/setup.bash;
      t=$(ros2 topic list 2>/dev/null);
      echo "$t" | grep -q "camera_1/color/image$" &&
      echo "$t" | grep -q "lidar3d_0/points$" &&
      ros2 node list 2>/dev/null | grep -q msckf'; then
    echo "stack ready after $((i*3))s"; break
  fi
  sleep 3
done

echo "== waiting for steady RTF > 0.4 (load-transient trap) =="
for i in $(seq 1 60); do
  rtf=$(docker exec sensing-node-fusion-1 bash -lc 'source /opt/ros/jazzy/setup.bash && python3 - <<PYEOF
import rclpy, time
from rclpy.parameter import Parameter
rclpy.init()
n = rclpy.create_node("rtfprobe", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
t0 = time.time()
while n.get_clock().now().nanoseconds == 0 and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
s0 = n.get_clock().now().nanoseconds; w0 = time.time()
while time.time() < w0 + 3:
    rclpy.spin_once(n, timeout_sec=0.05)
print((n.get_clock().now().nanoseconds - s0) / 1e9 / (time.time() - w0))
PYEOF' 2>/dev/null | tail -1)
  echo "  rtf=$rtf"
  awk -v r="${rtf:-0}" 'BEGIN{exit !(r>0.4)}' && break
  sleep 5
done

echo "== restarting kissicp FRESH (its local map must not include load-transient scans) =="
docker restart sensing-node-kissicp-1
sleep 3

echo "== starting ego_localizer (lidar config) =="
docker exec -d sensing-node-fusion-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && PYTHONPATH=/ros2_ws/src/fusion_core:/ros2_ws/src/ego_localizer:$PYTHONPATH \
   python3 /ros2_ws/src/ego_localizer/ego_localizer/node.py --ros-args \
   --params-file /ros2_ws/src/ego_localizer/config/ego_localizer_lidar.yaml \
   > /tmp/ego.log 2>&1'
sleep 5
docker exec sensing-node-fusion-1 bash -lc 'head -5 /tmp/ego.log || true'

echo "== demo drive (same params as the M3 chart) =="
docker cp scripts/m4_lio_demo.py sensing-node-fusion-1:/tmp/m4_lio_demo.py
docker exec sensing-node-fusion-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /tmp/m4_lio_demo.py '"$PREFIX"' 0.5 0.1 45'

echo "== eval chart =="
docker exec sensing-node-fusion-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && PYTHONPATH=/ros2_ws/src/eval_tools python3 -m eval_tools.evaluate \
   --gt '"$PREFIX"'_gt.csv \
   --est kiss_raw:'"$PREFIX"'_kiss.csv \
   --est openvins_raw:'"$PREFIX"'_ov.csv \
   --est ego_localizer:'"$PREFIX"'_ego.csv \
   --out '"$PREFIX"'_lio.png'
echo "== DONE =="
