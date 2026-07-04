#!/usr/bin/env bash
# n1_worker.sh — runs INSIDE the husky container (invoked by scripts/demo_n1_teleop.sh).
#
# N1 (teleop) demonstration + e-stop verification:
#  1. Drives a scripted path through joy_teleop/cmd_vel (the priority-10 twist_mux
#     input the keyboard teleop uses) and records the odom trajectory to $OUT.
#  2. Verifies the Clearpath twist_mux `e_stop` LOCK (topic platform/emergency_stop,
#     std_msgs/Bool, priority 255) halts an ACTIVE teleop command, then releases.
#
# Driving uses scripts/n1_drive.py (NOT `ros2 topic pub`): the diff_drive controller
# runs on sim time and DROPS commands whose header.stamp is stale, and `ros2 topic
# pub` always stamps 0 — so it only moves the robot right after a fresh bring-up
# (while sim time ~ 0) and silently fails later. A use_sim_time node stamps now().
set -o pipefail
source /opt/ros/jazzy/setup.bash 2>/dev/null   # source with nounset OFF (ROS setup
set -u                                          # uses unbound vars); enable -u after.
NS="${NS:-a200_0000}"
OUT="${OUT:-/tmp/n1_trajectory.csv}"
DRIVE="${DRIVE:-/tmp/n1_drive.py}"
J="/$NS/joy_teleop/cmd_vel"; ODOM="/$NS/platform/odom"; ES="/$NS/platform/emergency_stop"

# robust odom forward-velocity read (rejects the DDS "message lost" line)
rd(){ local v; for k in $(seq 1 6); do
  v=$(ros2 topic echo "$ODOM" --field twist.twist.linear.x --once 2>/dev/null \
      | grep -vi message | grep -oE '^-?[0-9]+(\.[0-9]+)?([eE][+-]?[0-9]+)?' | head -1)
  [ -n "$v" ] && { printf '%.4f' "$v"; return; }; sleep 0.2; done; echo READFAIL; }
# drive (lin,ang) for DUR s via the correctly-stamped publisher (blocking)
seg(){ python3 "$DRIVE" "$J" "$2" "$3" "$1" 20 >/dev/null 2>&1; }
# set the e_stop lock; engage holds `true` for ENG_S to arm the latch, release=false
lock(){ timeout "${2:-1}" ros2 topic pub -r 20 "$ES" std_msgs/msg/Bool "{data: $1}" >/dev/null 2>&1; }

# ROGUE-NODE SAFETY: kill every publisher we spawn and clear the latching e_stop on
# ANY exit — a stray driver keeps the robot moving and a left-on e_stop latches and
# silently blocks all future teleop (both bit us while building this).
cleanup(){ pkill -9 -f 'n1_drive.py' 2>/dev/null; pkill -9 -f 'ros2 topic pub' 2>/dev/null;
           timeout 1 ros2 topic pub -r 10 "$ES" std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM
cleanup; sleep 1

echo '== clear e_stop lock (false) =='; lock false 1

echo "== record odom trajectory while driving a path -> $OUT =="
timeout 16 ros2 topic echo "$ODOM" --field pose.pose.position --csv > "$OUT" 2>/dev/null &
REC=$!
seg 4 0.5  0.0     # forward
seg 4 0.3  0.5     # arc left
seg 4 0.5  0.0     # forward
wait $REC 2>/dev/null
N=$(grep -c ',' "$OUT" 2>/dev/null || echo 0)
echo "   recorded $N odom samples"

echo '== e_stop verification: drive (priority 10) -> engage -> release =='
python3 "$DRIVE" "$J" 0.5 0.0 22 20 >/dev/null 2>&1 &  TP=$!   # continuous stamped drive
sleep 4; V1=$(rd); echo "   v driving:          $V1"
# engage: HOLD `true` in the background (heartbeat) through the measurement
ros2 topic pub -r 20 "$ES" std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 &  EP=$!
sleep 3; V2=$(rd); echo "   v e_stop engaged:   $V2  (e_stop held)"
# release: stop the true heartbeat and publish a sustained `false`
kill -9 "$EP" 2>/dev/null; pkill -9 -f 'emergency_stop' 2>/dev/null
timeout 2 ros2 topic pub -r 20 "$ES" std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1
DRV_ALIVE=$(kill -0 "$TP" 2>/dev/null && echo yes || echo no)
sleep 2; V3=$(rd); echo "   v e_stop released:  $V3  (driver alive=$DRV_ALIVE)"
kill -9 "$TP" 2>/dev/null; pkill -9 -f 'n1_drive.py' 2>/dev/null; pkill -9 -f 'ros2 topic pub' 2>/dev/null
sleep 1
sp=$(pgrep -fc 'n1_drive.py' 2>/dev/null || true);   sp=${sp:-0}
qp=$(pgrep -fc 'ros2 topic pub' 2>/dev/null || true); qp=${qp:-0}
STRAY=$(( sp + qp ))
echo "   stray driver/pub procs after cleanup: $STRAY"

pass=1
awk "BEGIN{exit !($V1>0.3)}"               || pass=0   # was driving
awk "BEGIN{exit !($V2<0.05 && $V2>-0.05)}" || pass=0   # e_stop halted active teleop
awk "BEGIN{exit !($V3>0.3)}"               || pass=0   # resumed after release
[ "$STRAY" = 0 ]                           || pass=0   # no rogue nodes left
[ "$N" -gt 50 ]                            || pass=0   # trajectory actually recorded
if [ "$pass" = 1 ]; then
  echo 'N1 TELEOP + E-STOP: PASS  (drives -> e_stop halts active teleop -> resumes; no strays)'
else
  echo 'N1 TELEOP + E-STOP: FAIL  (see values above)'; exit 1
fi
