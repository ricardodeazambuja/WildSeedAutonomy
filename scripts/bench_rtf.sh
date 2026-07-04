#!/usr/bin/env bash
# bench_rtf.sh — discriminating RTF measurement for WildSeed world bundles.
#
# Answers "where does the RTF go?" with one session and four variants, so world
# tuning (tune_world_bundle.sh) is applied on evidence, not guesses:
#   baseline      bundle as-is, robot spawned          (the number users see)
#   world-only    same bundle, NO robot                (isolates gpu_lidar+cameras)
#   noshadow      shadows+sky off, robot spawned       (isolates the render fruit)
#   nolabels      Label plugins stripped, world-only   (isolates per-entity plugins)
#
# Usage:
#   ./scripts/bench_rtf.sh [--world-only] [bundle]     # default bundle: wildseed_42
#
# --world-only runs just the two robot-less variants (useful for pathological
# bundles like wildseed_forest where bring-up itself crawls).
#
# Each variant is a throwaway worlds_external/_bench_* copy (models hardlinked,
# so ~no disk); everything is cleaned up on exit. Requires the sim image built
# (deploy.sh build) and no other sim running.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT/.env"; [ -f "$ENV_FILE" ] || ENV_FILE="$ROOT/.env.example"
DC=(docker compose --env-file "$ENV_FILE" -f "$ROOT/docker/compose.yaml")

WORLD_ONLY=0
[ "${1:-}" = "--world-only" ] && { WORLD_ONLY=1; shift; }
SRC="${1:-wildseed_42}"
SRCDIR="$ROOT/worlds_external/$SRC"
[ -f "$SRCDIR/world.sdf" ] || { echo "no such bundle: $SRCDIR" >&2; exit 1; }

SETTLE="${SETTLE:-45}"     # wall seconds to let the sim settle before sampling
NSAMP="${NSAMP:-10}"       # stats messages to average

RESULTS=()

cleanup() {
  echo "== teardown =="
  "${DC[@]}" --profile compute --profile render down >/dev/null 2>&1
  rm -rf "$ROOT/worlds_external/_bench_"*
}
trap cleanup EXIT

mk_variant() { # mk_variant <name> <edit: none|noshadow|nolabels>
  local name="$1" edit="$2" bdir="$ROOT/worlds_external/_bench_$1"
  rm -rf "$bdir"; mkdir -p "$bdir"
  cp -al "$SRCDIR/models" "$bdir/models" 2>/dev/null || cp -a "$SRCDIR/models" "$bdir/models"
  cp "$SRCDIR/world.sdf" "$bdir/"; [ -f "$SRCDIR/spawn.json" ] && cp "$SRCDIR/spawn.json" "$bdir/"
  case "$edit" in
    noshadow)
      sed -i 's|<cast_shadows>true</cast_shadows>|<cast_shadows>false</cast_shadows>|g' "$bdir/world.sdf"
      sed -i '/<sky *\/>/d' "$bdir/world.sdf" ;;
    nolabels)
      python3 - "$bdir/world.sdf" <<'EOF'
import re, sys
p = sys.argv[1]
s = open(p).read()
s2 = re.sub(r'\s*<plugin filename="gz-sim-label-system"[^>]*>.*?</plugin>', '', s, flags=re.S)
open(p, 'w').write(s2)
print(f"  stripped {s.count('gz-sim-label-system') - s2.count('gz-sim-label-system')} Label plugins")
EOF
      ;;
  esac
}

world_name() { grep -oE '<world name="[^"]+"' "$1" | head -1 | cut -d'"' -f2; }

sample_rtf() { # sample_rtf <exec-cmd-prefix...> -- <world-name>
  # prints "mean sd" of real_time_factor over NSAMP stats messages.
  # gz is VENDORED under the ROS prefix in this image — must source first.
  local pre=() w
  while [ "$1" != "--" ]; do pre+=("$1"); shift; done; shift; w="$1"
  "${pre[@]}" "source /opt/ros/jazzy/setup.bash; gz topic -e -n $NSAMP -t /world/$w/stats 2>/dev/null | grep -oE 'real_time_factor: [0-9.eE+-]+' | awk '{s+=\$2; q+=\$2*\$2; c++} END {if (c) printf \"%.3f %.3f\", s/c, sqrt(q/c - (s/c)^2); else printf \"nan nan\"}'"
}

run_robot() { # run_robot <variant-name>
  local name="$1" bdir="_bench_$1" w rtf
  w=$(world_name "$ROOT/worlds_external/_bench_$1/world.sdf")
  echo "== [$name] robot-spawned, world '$w' =="
  SIM_WORLD="$bdir" SIM_WORLD_NAME="$w" "${DC[@]}" --profile compute up -d husky >/dev/null 2>&1
  # condition: world created (mirrors husky_sim.launch.py's wait), then settle
  local i ok=0
  for i in $(seq 1 150); do
    "${DC[@]}" exec -T husky bash -lc "source /opt/ros/jazzy/setup.bash; gz service -l 2>/dev/null | grep -q '/world/$w/create'" && { ok=1; break; }
    sleep 2
  done
  [ "$ok" = 1 ] || { echo "  WORLD NEVER CAME UP — skipping"; RESULTS+=("$name|$w|no-world"); "${DC[@]}" --profile compute down >/dev/null 2>&1; return; }
  echo "  world up after ~$((i * 2))s; settling ${SETTLE}s..."
  sleep "$SETTLE"
  rtf=$(sample_rtf "${DC[@]}" exec -T husky bash -lc -- "$w")
  echo "  RTF = $rtf (mean sd)"
  RESULTS+=("$name|robot|$rtf")
  "${DC[@]}" --profile compute down >/dev/null 2>&1
}

run_worldonly() { # run_worldonly <variant-name>
  local name="$1" bdir="_bench_$1" w rtf
  w=$(world_name "$ROOT/worlds_external/_bench_$1/world.sdf")
  echo "== [$name] world-only (no robot), world '$w' =="
  rtf=$("${DC[@]}" --profile render run --rm -T \
      -v "$ROOT/worlds_external:/worlds_external:ro" \
      -e GZ_SIM_RESOURCE_PATH="/worlds_external/$bdir/models" \
      sim bash -lc "
        source /opt/ros/jazzy/setup.bash
        gz sim -s -r -v1 --headless-rendering /worlds_external/$bdir/world.sdf >/dev/null 2>&1 &
        GZPID=\$!
        sleep $SETTLE
        gz topic -e -n $NSAMP -t /world/$w/stats 2>/dev/null | grep -oE 'real_time_factor: [0-9.eE+-]+' | awk '{s+=\$2; q+=\$2*\$2; c++} END {if (c) printf \"%.3f %.3f\", s/c, sqrt(q/c - (s/c)^2); else printf \"nan nan\"}'
        kill \$GZPID 2>/dev/null" 2>/dev/null)
  echo "  RTF = $rtf (mean sd)"
  RESULTS+=("$name|world-only|$rtf")
}

echo "=== bench_rtf: source bundle '$SRC' (settle ${SETTLE}s, $NSAMP samples/variant) ==="
"${DC[@]}" --profile compute --profile render down >/dev/null 2>&1   # clean slate

mk_variant baseline  none
mk_variant noshadow  noshadow
mk_variant nolabels  nolabels

if [ "$WORLD_ONLY" = 1 ]; then
  run_worldonly baseline
  run_worldonly noshadow
  run_worldonly nolabels
else
  run_robot     baseline
  run_worldonly baseline
  run_robot     noshadow
  run_worldonly nolabels
fi

echo
echo "=== RTF results: bundle '$SRC' ==="
printf '%-12s %-11s %-8s %-6s\n' variant mode "RTF" "±sd"
for r in "${RESULTS[@]}"; do
  IFS='|' read -r n m v s <<< "$r"
  printf '%-12s %-11s %-8s %-6s\n' "$n" "$m" "$v" "${s:-}"
done
echo "(interpretation: world-only ≪ baseline ⇒ robot's render sensors dominate;"
echo " noshadow ≫ baseline ⇒ shadows dominate; nolabels ≫ world-only ⇒ Label plugins matter)"
