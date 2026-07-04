#!/usr/bin/env bash
# remote.sh — run the sim on the SERVER, view it on the LAPTOP (workflow B).
#
# `deploy.sh` is host-local. This drives the headless sim on the compute box over
# SSH and (for `viz`) brings up a LOCAL RViz that sees it over DDS.
#
#   ./scripts/remote.sh sync             rsync this repo -> server (keeps server's .env)
#   ./scripts/remote.sh up [role]        deploy.sh up on the server (default: compute)
#   ./scripts/remote.sh viz              server sim + LOCAL RViz over DDS  (workflow B)
#   ./scripts/remote.sh viz-stop         stop the server sim AND the local RViz
#   ./scripts/remote.sh down             stop the server sim
#   ./scripts/remote.sh logs [svc]       follow server logs
#   ./scripts/remote.sh estop on|off     server e-stop
#   ./scripts/remote.sh teleop           interactive teleop (TTY) against the server sim
#   ./scripts/remote.sh diag             run scripts/diag_sim.sh on the server
#   ./scripts/remote.sh demo             run scripts/demo_n1_teleop.sh on the server
#   ./scripts/remote.sh run <cmd...>     run an arbitrary command in the server repo
#   ./scripts/remote.sh ssh              open a shell on the server in the repo dir
#
# Config (env or .env): SENSING_SERVER (host), SENSING_SERVER_DIR (repo path).
# RViz-over-DDS needs the laptop + server on the same ROS_DOMAIN_ID and a LAN that
# allows multicast (else set the CycloneDDS unicast peers — docs/headless-gui.md).
# The Gazebo GUI (gzgui) over gz-transport cross-host needs extra config, so `viz`
# brings up RViz only; for the full Gazebo GUI run all-local (deploy.sh viz).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENVF="$ROOT/.env"
cfg(){ [ -f "$ENVF" ] || return 0; grep -E "^$1=" "$ENVF" 2>/dev/null | tail -1 | cut -d= -f2- || true; }
SERVER="${SENSING_SERVER:-$(cfg SENSING_SERVER)}"; : "${SERVER:=server.local}"
DIR="${SENSING_SERVER_DIR:-$(cfg SENSING_SERVER_DIR)}"; : "${DIR:=WildSeedAutonomy}"

dep(){ ssh "$SERVER" "cd '$DIR' && ./scripts/deploy.sh $*"; }
lcompose(){ ( cd "$ROOT/docker" && docker compose --env-file "$ENVF" "$@" ); }

# remote.sh is ONLY for the two-box "server + laptop" workflow (B). On a
# laptop-only setup (no server) every subcommand here would hang/fail on SSH —
# give a clear pointer to deploy.sh instead of a cryptic ssh error.
require_server(){
  if ! timeout 4 bash -c "exec 3<>/dev/tcp/${SERVER}/22" 2>/dev/null; then
    echo "remote.sh: can't reach SSH on '$SERVER:22'." >&2
    echo "  This script drives a SEPARATE compute server (workflow B)." >&2
    echo "  Laptop-only? Use ./scripts/deploy.sh instead (e.g. 'deploy.sh viz')." >&2
    echo "  Have a server? Set SENSING_SERVER in .env / the environment." >&2
    exit 1
  fi
}

case "${1:-}" in
  sync)
    shift; require_server
    # Mirror the repo to the server, but NEVER clobber the server's own .env
    # (ROLE=compute) or large/local dirs.
    rsync -az --delete \
      --exclude '.git' --exclude '.env' --exclude 'results' --exclude 'datasets' \
      --exclude '__pycache__' --exclude '*.pyc' \
      "$ROOT/" "$SERVER:$DIR/"
    echo "synced $ROOT -> $SERVER:$DIR  (server .env preserved)" ;;
  viz)
    require_server
    echo "── start headless sim on $SERVER ──"
    dep up compute
    echo "── start LOCAL RViz (must share the server's ROS_DOMAIN_ID) ──"
    command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null 2>&1 || true
    lcompose --profile gui up -d rviz
    echo "RViz up locally, viewing the SERVER sim over DDS."
    echo "  drive:  ./scripts/remote.sh teleop      stop both:  ./scripts/remote.sh viz-stop" ;;
  viz-stop)
    require_server
    lcompose --profile gui down >/dev/null 2>&1 || true
    dep down
    echo "stopped local RViz + server sim." ;;
  teleop)   require_server; ssh -t "$SERVER" "cd '$DIR' && ./scripts/deploy.sh teleop" ;;
  diag)     require_server; ssh -t "$SERVER" "cd '$DIR' && ./scripts/diag_sim.sh" ;;
  demo)     require_server; ssh -t "$SERVER" "cd '$DIR' && ./scripts/demo_n1_teleop.sh" ;;
  run)      shift; require_server; ssh "$SERVER" "cd '$DIR' && $*" ;;
  ssh)      require_server; ssh -t "$SERVER" "cd '$DIR' && exec bash -l" ;;
  ""|-h|--help|help)
    sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//' ;;
  *)        require_server; dep "$@" ;;   # up/down/logs/estop/restart/shell/... pass through to deploy.sh
esac
