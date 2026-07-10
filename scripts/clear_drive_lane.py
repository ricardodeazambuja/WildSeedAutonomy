#!/usr/bin/env python3
"""Clear a drive lane through a WildSeed bundle (UGV adaptation).

WildSeed's vio_lio recipe steers scatter INTO the corridor band — designed for
its *flying* rig (z=2 m clears everything). On the ground the multi-metre
`rock_moss_set` collision meshes (reach 4–10 m at recipe scales) seal the
centreline: measured 2026-07-09, the Husky spawns wedged under a set-rock —
wheels airborne, wheel odometry integrating while gz truth is frozen (the
"wheels spin / robot frozen" signature). Until WildSeed grows a
footprint-aware corridor keep-out, this post-bundle step deletes the includes
whose conservative collision footprint intersects a lane rectangle, and
records exactly what it removed in the bundle's provenance.json (applied
identically to both texture variants of a seed, so A/B pairs stay paired).

Footprint = horizontal reach of the asset's collision GLB bounds × scale;
trees use 25 % of reach (trunk blocks, canopy doesn't; floor 0.6 m). Bushes
and grass are passable (`collide_without_contact`) and are never removed.

Usage:
  clear_drive_lane.py <bundle-dir> [--lane X0,X1,HALFW] [--arc V,WZ,SECS,MARGIN]
                      [--dry-run]
Default lane: x in [-2, 28], |y| <= 2.5.
--arc clears around the ACTUAL m3/m4 demo trajectory instead — a unicycle arc
from the spawn (v=0.5, wz=0.1, 45 s => a 5 m-radius circle curling left; the
straight --lane misses its upper half, measured: seed-107 drive stopped at
8 m of 22.5).
"""
import json
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def glb_bounds(path):
    with open(path, "rb") as f:
        assert f.read(4) == b"glTF", f"not a GLB: {path}"
        f.read(8)
        clen, _ = struct.unpack("<I4s", f.read(8))
        g = json.loads(f.read(clen))
    mins = [a["min"] for a in g.get("accessors", [])
            if a.get("type") == "VEC3" and "min" in a]
    maxs = [a["max"] for a in g.get("accessors", [])
            if a.get("type") == "VEC3" and "max" in a]
    lo = [min(v[i] for v in mins) for i in range(3)]
    hi = [max(v[i] for v in maxs) for i in range(3)]
    return lo, hi


def main():
    bdir = Path(sys.argv[1])
    lane = (-2.0, 28.0, 2.5)
    arc = None
    dry = "--dry-run" in sys.argv
    for i, a in enumerate(sys.argv):
        if a == "--lane":
            lane = tuple(float(v) for v in sys.argv[i + 1].split(","))
        if a == "--arc":
            arc = tuple(float(v) for v in sys.argv[i + 1].split(","))
    x0, x1, halfw = lane
    pts = None
    if arc:
        v, wz, secs, margin = arc
        # unicycle from (0,0,yaw 0), sampled every 0.2 s; prepend the
        # jerk-start straight (~1 m of +x)
        pts, th, px, py = [(-1.0, 0.0), (0.0, 0.0)], 0.0, 0.0, 0.0
        t, dt = 0.0, 0.2
        while t < secs:
            px += v * math.cos(th) * dt
            py += v * math.sin(th) * dt
            th += wz * dt
            pts.append((px, py))
            t += dt
        halfw = margin

    sdf = bdir / "world.sdf"
    tree = ET.parse(sdf)
    world = tree.getroot().find("world")
    removed = []
    for inc in list(world.findall("include")):
        uri = inc.findtext("uri") or ""
        if "/rock/" not in uri and "/tree/" not in uri:
            continue                      # bushes/grass are passable
        pose = [float(v) for v in (inc.findtext("pose") or "0 0 0").split()]
        scale = float((inc.findtext("scale") or "1").split()[0])
        cat, name = uri.split("/")[-2], uri.split("/")[-1]
        glb = bdir / "models" / cat / name / "mesh" / f"{name}_collision.glb"
        if not glb.exists():
            continue
        lo, hi = glb_bounds(glb)
        reach = max(abs(lo[0]), abs(hi[0]), abs(lo[1]), abs(hi[1])) * scale
        if "/tree/" in uri:
            reach = max(reach * 0.25, 0.6)
        # circle (pose, reach) vs lane rectangle or arc polyline
        if pts is not None:
            hit = any(math.hypot(pose[0] - ax, pose[1] - ay) <= reach + halfw
                      for ax, ay in pts)
        else:
            cx = min(max(pose[0], x0), x1)
            cy = min(max(pose[1], -halfw), halfw)
            hit = math.hypot(pose[0] - cx, pose[1] - cy) <= reach
        if hit:
            removed.append({"name": inc.findtext("name"), "asset": name,
                            "x": round(pose[0], 2), "y": round(pose[1], 2),
                            "scale": round(scale, 3),
                            "reach_m": round(reach, 2)})
            if not dry:
                world.remove(inc)

    action = "would remove" if dry else "removed"
    where = (f"arc v={arc[0]} wz={arc[1]} {arc[2]}s margin {arc[3]}m"
             if arc else f"lane x[{x0},{x1}] |y|<={halfw}")
    print(f"{bdir.name}: {action} {len(removed)} include(s) from {where}:")
    for r in removed:
        print(f"   {r['name']:>10}  {r['asset']:<28} at ({r['x']},{r['y']}) "
              f"scale {r['scale']} reach {r['reach_m']} m")
    if dry:
        return
    tree.write(sdf, encoding="utf-8", xml_declaration=True)
    prov_p = bdir / "provenance.json"
    prov = json.loads(prov_p.read_text()) if prov_p.exists() else {}
    spec = ({"arc": list(arc)} if arc
            else {"lane_x": [x0, x1], "lane_half_width": halfw})
    prev = prov.get("lane_cleared", {}).get("removed", [])
    spec["removed"] = prev + [r for r in removed if r not in prev]
    prov["lane_cleared"] = spec
    prov_p.write_text(json.dumps(prov, indent=2))
    print(f"   world.sdf rewritten; provenance.json updated")


if __name__ == "__main__":
    main()
