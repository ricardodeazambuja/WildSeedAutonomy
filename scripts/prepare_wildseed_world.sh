#!/usr/bin/env bash
# prepare_wildseed_world.sh — package a WildSeed world into a self-contained
# bundle the Husky sim can load (see husky_sim.launch.py SIM_WORLD support).
#
#   ./scripts/prepare_wildseed_world.sh <wildseed-dir> <world-file> [bundle-name]
#       [--spawn "x,y[,yaw]"]
#
#   <wildseed-dir>   WildSeed checkout (needs src/wildseed importable + the
#                    models/ that <world-file> was generated against — run this
#                    RIGHT AFTER generating the world: `wildseed scenario`
#                    overwrites models/ground per run, so world+models must be
#                    snapshotted together or the terrain won't match).
#   <world-file>     the generated .world/.sdf (absolute or relative to cwd)
#   [bundle-name]    output name (default: world file stem)
#   --spawn          robot spawn x,y[,yaw] in world metres (default 0,0,0)
#   --step S         physics max_step_size (default 0.002 — see below)
#   --keep-labels / --no-shadows / --no-sky   forwarded to the RTF tuner
#                    (tune_world_bundle.sh; labels are stripped by default,
#                    shadows/sky kept — those measured as RTF no-ops)
#
# Produces worlds_external/<bundle-name>/
#   world.sdf    copy of the world, world-shell injected (--shell-only):
#                Sensors/Imu/NavSat/AirPressure/Magnetometer systems +
#                <spherical_coordinates> — WITHOUT WildSeed's flying rig
#   models/      snapshot of every model the world references
#   spawn.json   {x, y, z, yaw, world_name} — z sampled from the terrain mesh
#                (wildseed height) + clearance, so the Husky spawns ON terrain
#
# Then:  ./scripts/deploy.sh world <bundle-name>  &&  ./scripts/deploy.sh restart
#
# Host-side only. Needs the WildSeed python deps (numpy-stl, scipy, click,
# pydantic — the `condalocal` env has them); the sim container needs nothing new.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_ROOT="$ROOT/worlds_external"

usage() { sed -n '2,26p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }

[ $# -ge 2 ] || usage
WILDSEED="$(cd "$1" && pwd)"; shift
WORLD_FILE="$(readlink -f "$1")"; shift
BUNDLE=""
SPAWN_XY="0,0"
TUNE_ARGS=()
STEP_SET=0
while [ $# -gt 0 ]; do
  case "$1" in
    --spawn) SPAWN_XY="$2"; shift 2 ;;
    # forwarded to tune_world_bundle.sh (RTF tuning runs by default)
    --keep-labels|--no-shadows|--no-sky) TUNE_ARGS+=("$1"); shift ;;
    --step) TUNE_ARGS+=("$1" "$2"); STEP_SET=1; shift 2 ;;
    -h|--help) usage ;;
    *) [ -z "$BUNDLE" ] && BUNDLE="$1" && shift || usage ;;
  esac
done
# Default physics step 2 ms: dense WildSeed worlds are PHYSICS-STEP-bound
# (bench_rtf.sh: forest RTF 0.034 @1ms -> 0.149 @4ms, linear). 2 ms doubles
# dense-world RTF while staying conservative for wheel contacts (gated by
# deploy.sh m3-smoke). Pass an explicit `--step 0.001` to keep gz's default.
[ "$STEP_SET" = 1 ] || TUNE_ARGS+=(--step 0.002)
[ -f "$WORLD_FILE" ] || { echo "world file not found: $WORLD_FILE" >&2; exit 1; }
[ -d "$WILDSEED/src/wildseed" ] || { echo "not a WildSeed checkout: $WILDSEED" >&2; exit 1; }
[ -d "$WILDSEED/models/ground" ] || { echo "no models/ground in $WILDSEED — generate the world first" >&2; exit 1; }
BUNDLE="${BUNDLE:-$(basename "$WORLD_FILE" | sed 's/\.[^.]*$//')}"
BDIR="$OUT_ROOT/$BUNDLE"

IFS=',' read -r SX SY SYAW <<< "$SPAWN_XY"
SYAW="${SYAW:-0}"

echo "── bundle: $BDIR"
mkdir -p "$BDIR/models"

# 1. snapshot ONLY the model categories the world references (same instant as
#    the world — `wildseed scenario` overwrites models/ground per run), and
#    never the flying sensor_rig (the Husky is the robot here). Hardlink when
#    source and bundle share a filesystem (models are ~1 GB); silent fallback
#    to a real copy otherwise.
CATS="$(grep -o 'model://[^<"]*' "$WORLD_FILE" | cut -d/ -f3 | sort -u | grep -v '^sensor_rig$')"
echo "   models/  <- $WILDSEED/models  [$(echo $CATS | tr '\n' ' ')]"
for cat in $CATS; do
  [ -d "$WILDSEED/models/$cat" ] || { echo "world references model://$cat but $WILDSEED/models/$cat is missing" >&2; exit 1; }
  rm -rf "$BDIR/models/$cat"
  cp -al "$WILDSEED/models/$cat" "$BDIR/models/$cat" 2>/dev/null \
    || cp -a "$WILDSEED/models/$cat" "$BDIR/models/$cat"
done

# 2. copy world; drop any sensor_rig include (worlds built with --rig carry
#    one); inject the world-shell (--shell-only)
cp "$WORLD_FILE" "$BDIR/world.sdf"
python3 - "$BDIR/world.sdf" <<'PY'
import sys
import xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1])
world = tree.getroot().find("world")
dropped = 0
for inc in list(world.findall("include")):
    if (inc.findtext("uri") or "") == "model://sensor_rig":
        world.remove(inc); dropped += 1
if dropped:
    tree.write(sys.argv[1], encoding="utf-8", xml_declaration=True)
    print(f"   stripped {dropped} sensor_rig include(s)")
PY
# --no-labels: nothing in the Husky stack consumes segmentation labels (no
# segmentation camera in robot.yaml), so don't inject one Label system per
# include. Falls back to labelled injection on WildSeed checkouts older than
# the flag; the tuner below strips labels in that case.
PYTHONPATH="$WILDSEED/src" python3 -m wildseed rig \
    --inject "$BDIR/world.sdf" --shell-only --no-labels --models "$BDIR/models" >/dev/null \
  || PYTHONPATH="$WILDSEED/src" python3 -m wildseed rig \
    --inject "$BDIR/world.sdf" --shell-only --models "$BDIR/models" >/dev/null
echo "   world.sdf shell-injected (sensor systems + spherical_coordinates, no rig, no labels)"

# 2b. RTF tuning (shadows/sky/labels off — evidence: scripts/bench_rtf.sh).
#     Opt out per-item with --keep-shadows/--keep-sky/--keep-labels; opt into a
#     larger physics step with --step S. Idempotent; re-run any time via
#     scripts/tune_world_bundle.sh (also how PRE-EXISTING bundles migrate).
"$ROOT/scripts/tune_world_bundle.sh" "$BDIR/world.sdf" ${TUNE_ARGS[@]+"${TUNE_ARGS[@]}"}

# 3. spawn z from the bundle's own terrain mesh + wheel clearance
HJSON="$(PYTHONPATH="$WILDSEED/src" python3 -m wildseed height \
          -x "$SX" -y "$SY" --json -b "$BDIR")"
python3 - "$BDIR" "$SX" "$SY" "$SYAW" "$BDIR/world.sdf" "$HJSON" <<'PY'
import json, re, sys
bdir, sx, sy, syaw, world_sdf = sys.argv[1:6]
h = json.loads(sys.argv[6])
name = re.search(r'<world\s+name=["\']([^"\']+)["\']',
                 open(world_sdf, encoding="utf-8").read(65536)).group(1)
spawn = {"x": float(sx), "y": float(sy),
         # Clearpath's flat-world default is z=0.15 above ground; add a bit
         # more so mesh-interpolation error can't start the chassis intersecting
         "z": round(h["z"] + 0.3, 4),
         "yaw": float(syaw), "world_name": name,
         "terrain_z": h["z"], "bounds": h["bounds"]}
json.dump(spawn, open(f"{bdir}/spawn.json", "w"), indent=2)
print(f"   spawn.json: ({spawn['x']:g}, {spawn['y']:g}) ground z={h['z']:g} "
      f"-> spawn z={spawn['z']:g}, world '{name}'")
PY

echo "── done. activate with:"
echo "   ./scripts/deploy.sh world $BUNDLE   &&   ./scripts/deploy.sh restart compute"
