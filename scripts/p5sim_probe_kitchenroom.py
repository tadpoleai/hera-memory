"""
One-shot probe: does Isaac Sim's own asset resolver (unlike plain pip
usd-core, tested earlier on Windows -- see PLAN.md Section 0.1) actually
resolve KitchenRoom.usd's payload references to shared prop assets (e.g.
/root/Table049 -> Table049/Table049.usd)?

Only KitchenRoom.usd + the Table049/ prop folder are uploaded so far (not
the full ~1.7GB directory) -- this is a cheap yes/no check before deciding
whether to upload everything and build a full extraction pipeline for this
scene.

Usage (on the cloud desktop): ~/isaacsim/python.sh scripts/p5sim_probe_kitchenroom.py
"""
import sys

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402


def p(*a):
    # plain print()'s stdout doesn't reliably show up through Kit's own
    # logger when run over a piped/non-tty SSH session (confirmed twice
    # now) -- write straight to a file as well as stdout so results survive
    # regardless of Kit's stdout handling.
    print(*a)
    sys.stdout.flush()
    with open("/tmp/probe_kitchen_results.txt", "a") as f:
        f.write(" ".join(str(x) for x in a) + "\n")


STAGE_PATH = "/home/admin/hera-memory/Locomotion/KitchenRoom/KitchenRoom.usd"

stage = Usd.Stage.Open(STAGE_PATH)
p("up axis:", UsdGeom.GetStageUpAxis(stage))
p("metersPerUnit:", UsdGeom.GetStageMetersPerUnit(stage))

prim = stage.GetPrimAtPath("/root/Table049")
p("Table049 prim valid:", prim.IsValid())
children = list(prim.GetChildren())
p("Table049 children:", [str(c.GetPath()) for c in children])

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
p("Table049 bbox min/max:", rng.GetMin(), rng.GetMax(), "empty:", rng.IsEmpty())

# broader check: how many prims total, how many Xform leaves have real bboxes
n_prims = 0
n_leaf_xform = 0
n_leaf_nonempty = 0
for pr in stage.Traverse():
    n_prims += 1
    if pr.GetTypeName() == "Xform" and not any(
            c.GetTypeName() == "Xform" for c in pr.GetChildren()):
        n_leaf_xform += 1
        r = cache.ComputeWorldBound(pr).ComputeAlignedRange()
        if not r.IsEmpty():
            n_leaf_nonempty += 1

p(f"total prims: {n_prims}")
p(f"leaf Xform: {n_leaf_xform}, with non-empty bbox: {n_leaf_nonempty}")

app.close()
