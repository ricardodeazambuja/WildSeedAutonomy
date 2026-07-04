#!/usr/bin/env bash
# tune_world_bundle.sh — idempotent RTF tuning of a prepared world bundle.
#
#   ./scripts/tune_world_bundle.sh <bundle|world.sdf-path>
#        [--keep-labels] [--no-shadows] [--no-sky] [--step S]
#
# Edits the bundle's world.sdf in place. Defaults follow the MEASURED evidence
# (scripts/bench_rtf.sh, RTX 2070 Max-Q — see docs/wildseed-worlds.md):
#
#   labels    STRIPPED by default — one gz::sim::systems::Label per include,
#             nothing consumes them (no segmentation camera in robot.yaml).
#             Measured RTF effect ~0; removed because it's free and semantic.
#             --keep-labels opts out (e.g. adding a segmentation camera).
#   --step S  THE real dense-world lever: forest (2,849 includes) world-only
#             RTF 0.034 -> 0.149 at step 0.004 (linear — physics-step-bound).
#             Changes physics integration + sensor stamp granularity, so the
#             tool only applies it when asked; prepare_wildseed_world.sh
#             passes --step 0.002 by default (gate: deploy.sh m3-smoke).
#   --no-shadows / --no-sky   available but NOT defaults: measured no-ops for
#             RTF here (sun shadows 0.310 vs 0.280 baseline, within noise;
#             forest identical to 3 decimals) and they change camera imagery.
#             May still help much weaker GPUs — that's why the flags exist.
#
# Called by prepare_wildseed_world.sh after shell-injection; run directly to
# migrate bundles prepared before this existed. Idempotent — safe to re-run.
# Host-side, python3 stdlib only (defusedxml used when present).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() { sed -n '2,26p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }

[ $# -ge 1 ] || usage
TARGET="$1"; shift
KEEP_LABELS=0 NO_SHADOWS=0 NO_SKY=0 STEP=""
while [ $# -gt 0 ]; do
  case "$1" in
    --keep-labels)  KEEP_LABELS=1; shift ;;
    --no-shadows)   NO_SHADOWS=1; shift ;;
    --no-sky)       NO_SKY=1; shift ;;
    --step)         STEP="$2"; shift 2 ;;
    -h|--help)      usage ;;
    *) echo "unknown flag: $1" >&2; usage ;;
  esac
done

if [ -f "$TARGET" ]; then SDF="$TARGET"
elif [ -f "$ROOT/worlds_external/$TARGET/world.sdf" ]; then SDF="$ROOT/worlds_external/$TARGET/world.sdf"
else echo "no such bundle or file: $TARGET" >&2; exit 1; fi

python3 - "$SDF" "$KEEP_LABELS" "$NO_SHADOWS" "$NO_SKY" "$STEP" <<'PY'
import sys
import xml.etree.ElementTree as ET

try:  # world bundles may come from other people — parse defensively (XXE)
    from defusedxml.ElementTree import parse as safe_parse
except ImportError:  # stdlib fallback: fine for self-generated bundles
    safe_parse = ET.parse

sdf, keep_labels, no_shadows, no_sky, step = sys.argv[1:6]
tree = safe_parse(sdf)
world = tree.getroot().find("world")
changed, summary = False, []

if keep_labels != "1":
    n = 0
    for parent in world.iter():
        for plug in list(parent.findall("plugin")):
            if plug.get("name") == "gz::sim::systems::Label":
                parent.remove(plug); n += 1
    if n:
        changed = True; summary.append(f"labels: {n} plugin(s) stripped")
    else:
        summary.append("labels: already absent")

if no_shadows == "1":
    n = 0
    for cs in world.iter("cast_shadows"):
        if (cs.text or "").strip() == "true":
            cs.text = "false"; n += 1
    scene = world.find("scene")
    if scene is not None and scene.find("shadows") is None:
        ET.SubElement(scene, "shadows").text = "false"; n += 1
    if n:
        changed = True; summary.append(f"shadows: {n} element(s) disabled")
    else:
        summary.append("shadows: already off")

if no_sky == "1":
    scene = world.find("scene")
    sky = scene.find("sky") if scene is not None else None
    if sky is not None:
        scene.remove(sky); changed = True; summary.append("sky: removed")
    else:
        summary.append("sky: already absent")

if step:
    ms = world.find("physics/max_step_size")
    if ms is not None and ms.text.strip() != step:
        summary.append(f"max_step_size: {ms.text.strip()} -> {step}")
        ms.text = step; changed = True
    else:
        summary.append(f"max_step_size: already {step}" if ms is not None
                       else "max_step_size: element not found (skipped)")

if changed:
    tree.write(sdf, encoding="utf-8", xml_declaration=True)
for s in summary:
    print(f"   {s}")
print(f"   -> {sdf}" + ("" if changed else "  (no changes needed)"))
PY
