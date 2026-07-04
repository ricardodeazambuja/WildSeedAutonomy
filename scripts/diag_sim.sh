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

# SLOW_SIM_FACTOR (env or .env, default 1) widens wall-clock ceilings on slow
# sims — see docs/operations.md "Slow machines / low RTF".
F="${SLOW_SIM_FACTOR:-$(grep -E '^SLOW_SIM_FACTOR=' "$ENV_FILE" 2>/dev/null | cut -d= -f2)}"
F="${F:-1}"

echo "== up husky (compute) =="
"${DC[@]}" --profile compute up -d husky >/dev/null 2>&1

echo "== wait for controller_manager (condition, not a fixed sleep; max ~$((140 * F))s) =="
for i in $(seq 1 $((70 * F))); do
  ex "ros2 node list" | grep -q controller_manager && { echo "  controller_manager up after ~$((i * 2))s"; break; }
  sleep 2
done

# Measure RTF first: topic *wall* rates scale with it (wall-rate ≈ sim-rate × RTF),
# so the observation windows below must stretch on a slow sim or `topic hz`
# reports nothing and the movement test drives for ~no sim time (false alarms).
RTF=$(ex "gz topic -e -n 5 -t /world/\${SIM_WORLD_NAME:-pipeline}/stats 2>/dev/null | grep -oE 'real_time_factor: [0-9.eE+-]+' | awk '{s+=\$2;c++} END {if (c) printf \"%.3f\", s/c}'")
[ -n "$RTF" ] || RTF=1.0
S=$(awk -v r="$RTF" 'BEGIN{r=(r>0.05)?r:0.05; w=6/r; if (w>120) w=120; if (w<6) w=6; printf "%d", w}')
E=$(( S / 2 < 4 ? 4 : S / 2 ))
echo "-- measured RTF ≈ $RTF  (rate windows scaled 6s -> ${S}s; expect wall-rate ≈ sim-rate × RTF) --"

echo "-- sensors present --"
ex "ros2 topic list | grep -E 'lidar3d_0/points|camera_0/color/image|imu_0/data' | sort"
echo "-- lidar rate --";          ex "timeout $S ros2 topic hz /$NS/sensors/lidar3d_0/points 2>&1 | grep -m1 'average rate'"
echo "-- controller_manager --";  ex "ros2 node list | grep controller_manager || echo MISSING"
echo "-- joint_states rate --";   ex "timeout $S ros2 topic hz /$NS/platform/joint_states 2>&1 | grep -m1 'average rate'"
echo "-- /clock publishers (want >0) --"; ex "ros2 topic info /clock | grep 'Publisher count'"
echo "-- /clock advancing --";    ex "timeout $S ros2 topic hz /clock 2>&1 | grep -m1 'average rate' || echo 'NOT advancing'"
echo "-- odom rate --";           ex "timeout $S ros2 topic hz /$NS/platform/odom 2>&1 | grep -m1 'average rate' || echo 'no odom'"

echo "== movement test: teleop forward ${S}s wall, judged against SIM time covered =="
# NOTE: bare `ros2 topic pub` stamps 0, which the sim-time diff_drive only accepts
# while sim time is still near 0 — fine HERE because diag runs right after a fresh
# bring-up (that's also why this doubles as a fresh-bringup check). The VERDICT is
# computed from the actual sim-time delta, so a slow sim doesn't false-fail it.
X0=$(ex "timeout $E ros2 topic echo /$NS/platform/odom --field pose.pose.position.x --once" | head -1)
C0=$(ex "timeout $E ros2 topic echo /clock --field clock.sec --once" | grep -oE '[0-9]+' | head -1)
echo "  x before: ${X0:-?}  (sim t=${C0:-?}s)"
ex "timeout $S ros2 topic pub -r 20 /$NS/joy_teleop/cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: base_link}, twist: {linear: {x: 0.8}}}' >/dev/null 2>&1; true"
X1=$(ex "timeout $E ros2 topic echo /$NS/platform/odom --field pose.pose.position.x --once" | head -1)
C1=$(ex "timeout $E ros2 topic echo /clock --field clock.sec --once" | grep -oE '[0-9]+' | head -1)
echo "  x after:  ${X1:-?}  (sim t=${C1:-?}s)"
if [ -n "${X0:-}" ] && [ -n "${X1:-}" ] && [ -n "${C0:-}" ] && [ -n "${C1:-}" ]; then
  awk -v x0="$X0" -v x1="$X1" -v c0="$C0" -v c1="$C1" 'BEGIN{
    dx=x1-x0; dt=c1-c0; want=0.8*dt;   # NB: "exp" is a reserved awk builtin
    printf "  Δx=%.2f m over Δt_sim=%ds (expected ≈ %.2f m at 0.8 m/s)\n", dx, dt, want;
    if (dt <= 0)            { print "  VERDICT: FAIL — sim time did not advance (/clock bridge dead? sim-debugging-notes #7)"; exit 1 }
    else if (dx > 0.3*want) { print "  VERDICT: PASS — robot moved consistently with the sim time covered" }
    else                    { print "  VERDICT: FAIL — sim time advanced but the robot barely moved (controllers? stale-stamp drop if sim time is already large)"; exit 1 }
  }'
else
  echo "  VERDICT: INCONCLUSIVE — missing odom/clock reads (see rates above)"
fi
