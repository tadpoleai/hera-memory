"""
Phase5 sim validation, step 1a/1b -- ground truth extraction from a Lightwheel
SimReady USD scene via pxr (headless, no Isaac Sim render loop needed for this
particular scene -- see work/p5_sim_validation/PLAN.md Section 0 for why this
does NOT generalize to every Lightwheel scene: KitchenRoom.usd uses payload
references to shared prop assets that fail to resolve under plain pip
usd-core + is missing an Omniverse-only `metricsAssembler` sublayer; scene_04
is a baked self-contained export with everything inlined, which is why it
works here with zero Isaac Sim dependency).

No Semantics API schema found anywhere in this scene (pxr.Semantics /
pxr.UsdSemantics don't even exist in the pip usd-core build, and no attribute
name containing "semantic"/"label"/"class" was found on any prim) -- this is
not a fallback path, it's the only path. Labels come from the prim name root
(strip trailing "_NN" / "_NN_NN" instance-index suffixes), normalized through
LABEL_TABLE below. LABEL_TABLE was built by manually opening the per-prop
Props/SM_*.usd files and checking mesh bbox size + bound material name (see
PLAN.md Section 1.5) -- SM_kitchen resolved confidently (cabinet front/
countertop panels, material names contain "kitchen_front"/"stone_kitchen").
SM_bar did NOT resolve to anything confident (thin 0.67x0.68x0.04m disc,
material "MI_plastic_bar_01" -- not shaped like a railing baluster or a bar
counter) -- kept as a raw-ish placeholder label with label_confidence="low"
rather than inventing a guess, per this project's "don't fake success" rule.

Place split (apartment_unit vs courtyard) is NOT k-means -- the scene has an
obvious, clean two-cluster structure by inspection (see PLAN.md Section 1.5):
every category except tree/building/fence sits tightly inside a small
interior-unit footprint near the origin; tree/building/fence sprawl across a
much larger area at a different Z band entirely. No third zone was forced.

Usage (must run with the project's dedicated venv -- has usd-core installed,
the main project Python does not):
    .venv_p5sim\\Scripts\\python.exe scripts\\p5sim_extract_gt.py
"""
import json
import math
import re
from pathlib import Path

from pxr import Usd, UsdGeom

ROOT = Path(__file__).resolve().parent.parent
STAGE_PATH = (r"C:\Users\EDY\Downloads\Lightwheel_OpenSource\Lightwheel_OpenSource"
              r"\Locomotion\Apartment\scene_04.usd")
OUT_PATH = ROOT / "work/p5_sim_validation/gt_instances.json"

# root (post regex strip) -> (class_label, mobility, place_id, attributes)
LABEL_TABLE = {
    "SM_tree":     ("tree",            "static",      "courtyard",      {}),
    "SM_wall":     ("wall",            "static",      "apartment_unit", {}),
    "SM_bar":      ("bar_panel",       "static",      "apartment_unit",
                     {"label_confidence": "low"}),
    "SM_window":   ("window",          "static",      "apartment_unit", {}),
    "SM_building": ("building",        "static",      "courtyard",      {}),
    "SM_chair":    ("chair",           "semi_static", "apartment_unit", {}),
    "SM_kitchen":  ("kitchen_cabinet", "static",      "apartment_unit", {}),
    "SM_lamp":     ("lamp",            "semi_static", "apartment_unit", {}),
    "SM_pillow":   ("pillow",          "semi_static", "apartment_unit", {}),
    "SM_door":     ("door",            "static",      "apartment_unit", {}),
    "SM_table":    ("table",           "semi_static", "apartment_unit", {}),
    "SM_wardrobe": ("wardrobe",        "static",      "apartment_unit", {}),
    "SM_carpetl":  ("carpet",          "semi_static", "apartment_unit", {}),
    "SM_fence":    ("fence",           "static",      "courtyard",      {}),
    "SM_floor":    ("floor",           "static",      "apartment_unit", {}),
    "SM_flower":   ("plant",           "semi_static", "apartment_unit", {}),
    "SM_sofa":     ("sofa",            "semi_static", "apartment_unit", {}),
    "SM_tiles":    ("floor_tile",      "static",      "apartment_unit", {}),
}
EXCLUDED_ROOTS = {"Root"}

# Place AABBs: measured extent (see PLAN.md 1.5) + padding, same convention
# as p5_ingest_4dkankan.py's zone bbox = members + 1.5m pad.
PLACES = {
    "apartment_unit": {
        "name": "Apartment interior unit",
        "bounds": (-6.96, -5.16, -1.0, 8.76, 8.39, 3.5),
    },
    "courtyard": {
        "name": "Courtyard / exterior",
        "bounds": (-37.2, -23.09, -11.0, 46.22, 36.04, 1.0),
    },
}


def strip_instance_suffix(name: str) -> str:
    return re.sub(r"_\d+(_\d+)?$", "", name)


def yaw_from_quat(quat) -> float:
    qw = quat.GetReal()
    qx, qy, qz = quat.GetImaginary()
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))


def extract():
    stage = Usd.Stage.Open(STAGE_PATH)
    up_axis = str(UsdGeom.GetStageUpAxis(stage))
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    assert up_axis == "Z", f"expected Z-up, got {up_axis} -- see plan §2.1, need transform"
    assert abs(mpu - 1.0) < 1e-6, f"expected metersPerUnit=1.0, got {mpu} -- need unit scale"

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    instances = []
    unknown_roots = set()
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Xform":
            continue
        if any(c.GetTypeName() == "Xform" for c in prim.GetChildren()):
            continue  # not a leaf -- it's a grouping node
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            continue  # unresolved payload or genuinely empty -- skip, don't fake a pose

        root = strip_instance_suffix(prim.GetName())
        if root in EXCLUDED_ROOTS:
            continue
        if root not in LABEL_TABLE:
            unknown_roots.add(root)
            continue

        class_label, mobility, place_id, extra_attrs = LABEL_TABLE[root]
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = xf.ExtractTranslation()
        yaw = yaw_from_quat(xf.ExtractRotationQuat())
        mn, mx = rng.GetMin(), rng.GetMax()

        instances.append({
            "prim_path": str(prim.GetPath()),
            "class_label": class_label,
            "mobility": mobility,
            "place_id": place_id,
            "pose": {"x": round(pos[0], 4), "y": round(pos[1], 4),
                     "z": round(pos[2], 4), "yaw": round(yaw, 6)},
            "aabb": {"min_x": round(mn[0], 4), "min_y": round(mn[1], 4), "min_z": round(mn[2], 4),
                     "max_x": round(mx[0], 4), "max_y": round(mx[1], 4), "max_z": round(mx[2], 4)},
            "attributes": dict(extra_attrs),
        })

    if unknown_roots:
        raise RuntimeError(f"LABEL_TABLE missing entries for: {sorted(unknown_roots)} "
                            f"-- extend the table, don't silently drop instances")

    return {
        "scene": STAGE_PATH,
        "up_axis": up_axis,
        "meters_per_unit": mpu,
        "places": PLACES,
        "instances": instances,
    }


def main():
    result = extract()
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    n = len(result["instances"])
    print(f"{n} instances extracted -> {OUT_PATH}")
    from collections import Counter
    by_label = Counter(i["class_label"] for i in result["instances"])
    by_place = Counter(i["place_id"] for i in result["instances"])
    print("by class_label:", dict(by_label))
    print("by place_id:", dict(by_place))
    assert n == 103, f"expected 103 instances (see PLAN.md §1.5), got {n}"
    print("count check OK (103)")


if __name__ == "__main__":
    main()
