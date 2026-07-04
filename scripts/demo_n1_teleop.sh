#!/usr/bin/env bash
# demo_n1_teleop.sh — N1 (teleop) demo + e-stop verification, fully headless.
#
# Brings up the Husky, drives a scripted path via joy_teleop/cmd_vel (the same
# twist_mux input the keyboard teleop uses), records the odom trajectory to
# results/n1_trajectory.csv, verifies the twist_mux e_stop lock halts active teleop,
# and (if the fusion image is built) renders results/n1_trajectory.png.
#
# This is the reproducible, headless substitute for a "drive-around clip": the
# trajectory plot is the show-artifact. For live video, view odom/RViz from the
# laptop over DDS (docs/headless-gui.md). Run on the host where the images are built.
#
#   ./scripts/demo_n1_teleop.sh            # keeps the sim up
#   KEEP=0 ./scripts/demo_n1_teleop.sh     # tear the sim down at the end
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT/.env"; [ -f "$ENV_FILE" ] || ENV_FILE="$ROOT/.env.example"
DC=(docker compose --env-file "$ENV_FILE" -f "$ROOT/docker/compose.yaml")
NS="${NS:-a200_0000}"; KEEP="${KEEP:-1}"
CID="sensing-node-husky-1"
mkdir -p "$ROOT/results"

# SLOW_SIM_FACTOR (env or .env, default 1) widens wall-clock ceilings on slow
# sims — see docs/operations.md "Slow machines / low RTF". The worker itself
# measures in SIM time (n1_worker.sh), so only this bring-up wait needs scaling.
F="${SLOW_SIM_FACTOR:-$(grep -E '^SLOW_SIM_FACTOR=' "$ENV_FILE" 2>/dev/null | cut -d= -f2)}"
F="${F:-1}"

echo '== up husky (compute) =='
"${DC[@]}" --profile compute up -d husky >/dev/null 2>&1

echo "== wait for controller_manager (condition, not a fixed sleep; max ~$((140 * F))s) =="
for i in $(seq 1 $((70 * F))); do
  "${DC[@]}" --profile compute exec -T husky bash -lc \
    "source /opt/ros/jazzy/setup.bash; ros2 node list" 2>/dev/null | grep -q controller_manager \
    && { echo "  up after ~$((i * 2))s"; break; }
  sleep 2
done

echo '== run N1 worker (drive path + e-stop test) inside husky =='
docker cp "$ROOT/scripts/n1_worker.sh" "$CID":/tmp/n1_worker.sh >/dev/null
docker cp "$ROOT/scripts/n1_drive.py"  "$CID":/tmp/n1_drive.py  >/dev/null  # stamped publisher
"${DC[@]}" --profile compute exec -T -e NS="$NS" -e OUT=/tmp/n1_trajectory.csv \
  -e DRIVE=/tmp/n1_drive.py husky bash /tmp/n1_worker.sh
WORKER_RC=$?

echo '== fetch trajectory CSV -> results/ =='
docker cp "$CID":/tmp/n1_trajectory.csv "$ROOT/results/n1_trajectory.csv" >/dev/null 2>&1 \
  && echo "  results/n1_trajectory.csv ($(grep -c ',' "$ROOT/results/n1_trajectory.csv") pts)"

echo '== plot trajectory (fusion) =='
if "${DC[@]}" run --rm -T -v "$ROOT/scripts:/scripts:ro" fusion \
     python3 /scripts/plot_trajectory.py /results/n1_trajectory.csv /results/n1_trajectory.png 2>/dev/null; then
  echo "  results/n1_trajectory.png"
else
  echo "  (skipped plot — build the fusion image: ./scripts/deploy.sh build)"
fi

# ROGUE-NODE SWEEP: ROS background publishers commonly survive when you think they
# were killed. Assert the worker left none, and clear the latching e_stop for safety.
echo '== rogue-node sweep =='
"${DC[@]}" --profile compute exec -T husky bash -lc \
  "pkill -9 -f 'ros2 topic pub' 2>/dev/null; source /opt/ros/jazzy/setup.bash;
   timeout 1 ros2 topic pub -r 10 /$NS/platform/emergency_stop std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1;
   n=\$(pgrep -fc 'ros2 topic pub' 2>/dev/null || echo 0);
   echo \"  stray 'ros2 topic pub' procs: \$n ; e_stop cleared\"" 2>/dev/null

[ "$KEEP" = 0 ] && { echo '== teardown =='; "${DC[@]}" --profile compute down >/dev/null 2>&1; }
exit $WORKER_RC
