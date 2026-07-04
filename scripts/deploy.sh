#!/usr/bin/env bash
# deploy.sh — Docker-only lifecycle for the sensing-node sim stack.
# Same script on laptop and server; per-host behaviour comes from .env.
# Pattern follows the LTSpice deploy (deploy.sh + .env + compose + docs).
#
# Usage:
#   ./deploy.sh init                 create .env from .env.example (host-tuned)
#   ./deploy.sh check                run host readiness checks
#   ./deploy.sh build [svc...]       build images (sim, fusion)
#   ./deploy.sh up [role]            bring up services for ROLE (all|compute|gui)
#   ./deploy.sh down                 stop everything
#   ./deploy.sh smoke                run the DDS talker/listener smoke test
#   ./deploy.sh m3-smoke             M3 stereo-VIO gate: cameras render+track, OpenVINS live
#   ./deploy.sh render               minimal headless EGL render check (smoke world)
#   ./deploy.sh teleop               interactive keyboard teleop (SSH: ssh -t; or laptop)
#   ./deploy.sh estop on|off         engage/release the twist_mux e-stop (latches!)
#   ./deploy.sh viz                  LOCAL live view: sim + RViz + Gazebo GUI (no fusion)
#   ./deploy.sh logs [svc]           follow logs
#   ./deploy.sh shell <svc>          open a shell in a (new) container
#   ./deploy.sh set --cpus N --mem Xg   edit resource caps in .env
#   ./deploy.sh world <bundle>|default  select the sim world (a prepared
#                                    worlds_external/ bundle, or the pipeline
#                                    default) — then 'restart' to apply
#   ./deploy.sh restart [role]       down + up
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_DIR="$ROOT/docker"
ENV_FILE="$ROOT/.env"
EXAMPLE="$ROOT/.env.example"

dc() { ( cd "$DOCKER_DIR" && docker compose --env-file "$ENV_FILE" -f compose.yaml "$@" ); }

getenv() { # read KEY from .env, fallback to $2
  local v; v=$(grep -E "^$1=" "$ENV_FILE" 2>/dev/null | tail -1 | cut -d= -f2-)
  echo "${v:-${2:-}}"
}
setenv() { # set KEY=VALUE in .env (in place)
  if grep -qE "^$1=" "$ENV_FILE"; then
    sed -i "s|^$1=.*|$1=$2|" "$ENV_FILE"
  else
    echo "$1=$2" >> "$ENV_FILE"
  fi
}

role_to_profiles() {
  case "$1" in
    all)     echo "--profile compute --profile gui" ;;
    compute) echo "--profile compute" ;;
    gui)     echo "--profile gui" ;;
    smoke)   echo "--profile smoke" ;;
    vio)     echo "--profile vio" ;;     # M3: OpenVINS + ground-truth bridge (add on top of compute)
    *) echo "unknown ROLE '$1' (use all|compute|gui|vio)" >&2; exit 2 ;;
  esac
}

need_env() { [ -f "$ENV_FILE" ] || { echo "no .env — run './deploy.sh init' first" >&2; exit 1; }; }

cmd_init() {
  if [ -f "$ENV_FILE" ]; then
    echo ".env already exists — leaving it. Edit it directly or use 'set'."; return
  fi
  cp "$EXAMPLE" "$ENV_FILE"
  # Tune caps to THIS host.
  local cores mem_gb cap_cpu cap_mem
  cores=$(nproc 2>/dev/null || echo 8)
  mem_gb=$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo 2>/dev/null || echo 16)
  cap_cpu=$(( cores > 2 ? cores - 2 : cores ))
  cap_mem=$(( mem_gb * 3 / 4 )); [ "$cap_mem" -lt 4 ] && cap_mem=4
  setenv CPUS "$cap_cpu"
  setenv MEM_LIMIT "${cap_mem}g"
  [ -n "${DISPLAY:-}" ] && setenv DISPLAY "$DISPLAY"
  echo "created .env  (CPUS=$cap_cpu, MEM_LIMIT=${cap_mem}g, DISPLAY=${DISPLAY:-unset})"
  echo "→ review ROLE in $ENV_FILE  (all = laptop-only, compute = GPU box, gui = your seat)"
}

cmd_check() { ROLE="$([ -f "$ENV_FILE" ] && getenv ROLE all || echo all)" bash "$ROOT/scripts/check_host.sh" "${1:-}"; }

cmd_build() { need_env; dc --profile compute --profile gui --profile smoke --profile render --profile vio build "$@"; }

cmd_up() {
  need_env
  local role="${1:-$(getenv ROLE all)}"
  # GUI needs local X access (Option 1: render locally).
  if [ "$role" = "gui" ] || [ "$role" = "all" ]; then
    command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null 2>&1 || \
      echo "(note: xhost not available — GUI may be denied X access)"
  fi
  # shellcheck disable=SC2046
  dc $(role_to_profiles "$role") up -d
  echo "up [$role]. 'deploy.sh logs' to watch, 'deploy.sh down' to stop."
}

cmd_down() { need_env; dc --profile compute --profile gui --profile smoke --profile render --profile teleop --profile vio down ; }

cmd_teleop() { # interactive keyboard teleop (SSH: use ssh -t; or run on the laptop)
  need_env
  echo "── keyboard teleop → /a200_0000/joy_teleop/cmd_vel (Ctrl-C to stop) ──"
  dc --profile teleop run --rm teleop
}

cmd_estop() { # engage/release the twist_mux e_stop lock: deploy.sh estop on|off
  need_env
  local ns topic; ns="$(getenv ROBOT_NS a200_0000)"; topic="/${ns}/platform/emergency_stop"
  case "${1:-}" in
    on|true|engage)
      # The lock (priority 255) must be HELD true to stay engaged, so this blocks
      # and publishes a heartbeat until Ctrl-C — then run 'estop off' to release.
      echo "── e_stop ENGAGED: holding ${topic}=true  (Ctrl-C, then 'deploy.sh estop off') ──"
      dc --profile compute exec -T husky bash -lc \
        "source /opt/ros/jazzy/setup.bash && exec ros2 topic pub -r 10 ${topic} std_msgs/msg/Bool '{data: true}'" ;;
    off|false|release)
      echo "── e_stop RELEASE: ${topic}=false ──"
      dc --profile compute exec -T husky bash -lc \
        "source /opt/ros/jazzy/setup.bash && timeout 2 ros2 topic pub -r 20 ${topic} std_msgs/msg/Bool '{data: false}'" \
        && echo "   released — robot can drive again." ;;
    *) echo "usage: deploy.sh estop on|off   (on holds the stop; off releases it)" >&2; return 2 ;;
  esac
}

cmd_viz() { # LOCAL live visualization (workflow A, laptop-only): sim + RViz (preloaded) + Gazebo GUI
  need_env                                       # brings up only husky+rviz+gzgui (NO fusion)
  command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null 2>&1 || \
    echo "(note: xhost not available — GUI may be denied X access)"
  dc --profile compute --profile gui up -d husky rviz gzgui
  echo "── viz up: husky (sim) + RViz + Gazebo GUI on the local GPU ──"
  echo "   drive it:  drag the 'Teleop' marker in RViz (Interact tool / 'i'), OR ./scripts/deploy.sh teleop"
  echo "   stop all:  ./scripts/deploy.sh down"
  echo "   (RViz loads config/husky.rviz: lidar + odom + TF + camera + teleop marker, fixed frame 'odom')"
}

cmd_render() { # minimal headless EGL render check (smoke.sdf), no Clearpath
  need_env
  echo "── headless render: gz sim smoke world (Ctrl-C to stop) ──"
  echo "   in another shell: deploy.sh shell sim → gz topic -e -n1 -t /smoke/camera"
  dc --profile render up sim
}

cmd_smoke() {
  need_env
  echo "── DDS smoke: talker → listener across two containers ──"
  dc --profile smoke up -d
  echo "waiting 6s for messages..."; sleep 6
  if dc --profile smoke logs listener 2>/dev/null | grep -q "I heard"; then
    echo "PASS: listener received talker messages (cross-container DDS works)"; rc=0
  else
    echo "FAIL: no messages received — check DDS/network (see docs/headless-gui.md §multi-host)"; rc=1
  fi
  dc --profile smoke down >/dev/null 2>&1 || true
  return $rc
}

cmd_logs()  { need_env; dc --profile compute --profile gui logs -f "${@:-}"; }
cmd_shell() { need_env; [ -n "${1:-}" ] || { echo "usage: deploy.sh shell <svc>" >&2; exit 2; }
              dc run --rm --entrypoint bash "$1"; }
cmd_restart() { cmd_down; cmd_up "${1:-}"; }

cmd_set() {
  need_env
  while [ $# -gt 0 ]; do
    case "$1" in
      --cpus) setenv CPUS "$2"; shift 2 ;;
      --mem)  setenv MEM_LIMIT "$2"; shift 2 ;;
      --shm)  setenv SHM_SIZE "$2"; shift 2 ;;
      *) echo "unknown set flag: $1" >&2; exit 2 ;;
    esac
  done
  echo "updated .env: CPUS=$(getenv CPUS), MEM_LIMIT=$(getenv MEM_LIMIT), SHM_SIZE=$(getenv SHM_SIZE)"
  echo "→ 'deploy.sh restart' to apply"
}

cmd_world() { # select the world the husky sim loads (see prepare_wildseed_world.sh)
  need_env
  local bundle="${1:-}"
  case "$bundle" in
    "") echo "current: SIM_WORLD='$(getenv SIM_WORLD)' (empty = pipeline default)"
        echo "bundles available:"; ls -1 "$ROOT/worlds_external" 2>/dev/null | grep -v '^\.' || true
        echo "usage: deploy.sh world <bundle>|default"; return 0 ;;
    default|pipeline)
        setenv SIM_WORLD ""; setenv SIM_WORLD_NAME pipeline
        echo "world -> pipeline (Clearpath default). 'deploy.sh restart compute' to apply." ;;
    *)  local sdf="$ROOT/worlds_external/$bundle/world.sdf"
        [ -f "$sdf" ] || { echo "no such bundle: $sdf — run scripts/prepare_wildseed_world.sh first" >&2; return 1; }
        # world name feeds gtbridge's gz topic; keep it in step with SIM_WORLD
        local name
        name=$(grep -oE '<world name="[^"]+"' "$sdf" | head -1 | cut -d'"' -f2)
        [ -n "$name" ] || { echo "could not parse <world name=...> from $sdf" >&2; return 1; }
        setenv SIM_WORLD "$bundle"; setenv SIM_WORLD_NAME "$name"
        echo "world -> $bundle (gz world '$name'). 'deploy.sh restart compute' to apply." ;;
  esac
}

cmd_m3smoke() {
  need_env
  echo "── M3 stereo-VIO smoke gate (cameras render+track, stereo synced, OpenVINS live) ──"
  dc --profile compute --profile vio up -d            # idempotent
  echo "waiting for the stereo cameras + OpenVINS to come up..."
  local i n
  for i in $(seq 1 40); do
    n=$(dc exec -T fusion bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic list 2>/dev/null | grep -c "camera_[01]/color/image$"' 2>/dev/null | tr -d "[:space:]")
    [ "${n:-0}" -ge 2 ] && break
    sleep 3
  done
  dc exec -T fusion bash -lc \
    'source /opt/ros/jazzy/setup.bash && python3 /ros2_ws/src/sensing_bringup/scripts/m3_smoke.py'
  local rc=$?
  echo "(stack left up — 'deploy.sh down' to stop)"
  return $rc
}

case "${1:-}" in
  init)    shift; cmd_init "$@" ;;
  check)   shift; cmd_check "$@" ;;
  build)   shift; cmd_build "$@" ;;
  up)      shift; cmd_up "$@" ;;
  down)    shift; cmd_down "$@" ;;
  smoke)   shift; cmd_smoke "$@" ;;
  m3-smoke) shift; cmd_m3smoke "$@" ;;
  render)  shift; cmd_render "$@" ;;
  teleop)  shift; cmd_teleop "$@" ;;
  estop)   shift; cmd_estop "$@" ;;
  viz)     shift; cmd_viz "$@" ;;
  logs)    shift; cmd_logs "$@" ;;
  shell)   shift; cmd_shell "$@" ;;
  set)     shift; cmd_set "$@" ;;
  world)   shift; cmd_world "$@" ;;
  restart) shift; cmd_restart "$@" ;;
  ""|-h|--help|help)
    sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ;;
  *) echo "unknown command '$1' (try: deploy.sh help)" >&2; exit 2 ;;
esac
