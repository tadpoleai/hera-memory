"""
Dump every leaf-Xform instance in KitchenRoom.usd with position + bbox size,
grouped by normalized name-root -- same reconnaissance step done locally for
scene_04.usd (see PLAN.md Section 1.5), just against KitchenRoom this time
now that its payloads resolve under Isaac Sim (see probe results). Used to
build the label/mobility table before writing a real extraction script.

Usage: ~/isaacsim/python.sh scripts/p5sim_dump_kitchenroom.py
"""
import re
import collections
import json

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

STAGE_PATH = "/home/admin/hera-memory/Locomotion/KitchenRoom/KitchenRoom.usd"
OUT_PATH = "/tmp/kitchenroom_dump.json"

stage = Usd.Stage.Open(STAGE_PATH)
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

rows = []
for p in stage.Traverse():
    if p.GetTypeName() != "Xform":
        continue
    if any(c.GetTypeName() == "Xform" for c in p.GetChildren()):
        continue
    rng = cache.ComputeWorldBound(p).ComputeAlignedRange()
    if rng.IsEmpty():
        continue
    xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = xf.ExtractTranslation()
    name = p.GetName()
    root = re.sub(r"_?\d+$", "", name)
    mn, mx = rng.GetMin(), rng.GetMax()
    rows.append({
        "path": str(p.GetPath()), "name": name, "root": root,
        "x": round(pos[0], 3), "y": round(pos[1], 3), "z": round(pos[2], 3),
        "size": [round(mx[i] - mn[i], 3) for i in range(3)],
    })

with open(OUT_PATH, "w") as f:
    json.dump(rows, f, indent=2)

with open("/tmp/kitchenroom_dump_summary.txt", "w") as f:
    f.write(f"total instances: {len(rows)}\n")
    by_root = collections.defaultdict(list)
    for r in rows:
        by_root[r["root"]].append(r)
    for root, members in sorted(by_root.items(), key=lambda kv: -len(kv[1])):
        cx = sum(m["x"] for m in members) / len(members)
        cy = sum(m["y"] for m in members) / len(members)
        cz = sum(m["z"] for m in members) / len(members)
        avg_size = [sum(m["size"][i] for m in members) / len(members) for i in range(3)]
        f.write(f"{root:25s} n={len(members):3d} centroid=({cx:7.2f},{cy:7.2f},{cz:6.2f}) "
                f"avg_size=({avg_size[0]:.2f},{avg_size[1]:.2f},{avg_size[2]:.2f})\n")

print(f"wrote {OUT_PATH} and summary")
app.close()
