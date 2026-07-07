#!/bin/bash
# m5_keystone_eval.sh — one-shot GPS-denied keystone run: fresh stack ->
# steady-RTF gate -> ego_localizer (relative+GNSS config) -> drive through
# on -> denied -> reacquire -> chart.
#
# Same bring-up rules as m4_lio_eval.sh: demos start only after the world
# clears its slow-load transient (steady RTF > 0.4) — essential on WildSeed
# bundles, whose first minutes run at RTF ~0.001.
#
# Usage: ./scripts/m5_keystone_eval.sh [/results/prefix]  (default /results/gps_denied)
# World: whatever `deploy.sh world` selected (pipeline or a WildSeed bundle).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${1:-/results/gps_denied}"
cd "$ROOT"

./scripts/deploy.sh down
./scripts/deploy.sh up compute

echo "== waiting for the GPS fix topic =="
for i in $(seq 1 80); do
  # NOTE: grep -c exits 1 on zero matches -> the || true keeps set -e from
  # killing the wait loop on a cold world (the count still lands in n).
  n=$(docker exec sensing-node-fusion-1 bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic list 2>/dev/null | grep -c "gps_0/fix$" || true' 2>/dev/null | tr -d "[:space:]")
  [ "${n:-0}" -ge 1 ] && { echo "stack ready after $((i*3))s"; break; }
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

echo "== starting ego_localizer (relative + GNSS keystone config) =="
docker exec -d sensing-node-fusion-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && PYTHONPATH=/ros2_ws/src/fusion_core:/ros2_ws/src/ego_localizer:$PYTHONPATH \
   python3 /ros2_ws/src/ego_localizer/ego_localizer/node.py --ros-args \
   --params-file /ros2_ws/src/ego_localizer/config/ego_localizer_gnss.yaml \
   > /tmp/ego_gnss.log 2>&1'
sleep 5
docker exec sensing-node-fusion-1 bash -lc 'head -3 /tmp/ego_gnss.log || true'

echo "== keystone drive: GPS on -> denied -> reacquire =="
docker cp scripts/gps_denied_demo.py sensing-node-fusion-1:/tmp/gps_denied_demo.py
docker cp scripts/plot_gps_denied.py sensing-node-fusion-1:/tmp/plot_gps_denied.py
docker exec sensing-node-fusion-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /tmp/gps_denied_demo.py '"$PREFIX"'.csv'

echo "== chart =="
docker exec sensing-node-fusion-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /tmp/plot_gps_denied.py '"$PREFIX"'.csv '"$PREFIX"'.png'
echo "== DONE =="
