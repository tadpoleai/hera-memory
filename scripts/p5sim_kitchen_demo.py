"""
KitchenRoom.usd demo ingest -- NOT part of the formal 1a/1b validation
pipeline (that's scene_04.usd only, see work/p5_sim_validation/report.md).
This is scoped narrowly to answer one question: "does a ground-truth box
actually sit on a recognizable real kitchen object", which scene_04's
building-exterior geometry couldn't demonstrate (see PLAN.md's route A/B
discussion). Hand-curated allowlist of ~25 exact prim paths, picked from
scripts/p5sim_dump_kitchenroom.py's instance dump -- KitchenRoom's leaf
Xforms are mostly SUB-PARTS of composite objects (a cabinet's body + each
door as separate leaves, a toaster's body/knob/lever as separate leaves),
so unlike scene_04 this needed manual picking of one representative leaf
per real object rather than a blanket "every leaf Xform" pass. Also skips
~23 near-zero-size "Visuls"/"Visuals" helper Xforms (interaction anchors,
not real geometry) that a blanket pass would have picked up as junk.

Refrigerator001 has no usable leaf-Xform bbox (only a "Visuls" helper child
with a near-zero size, its actual body mesh is elsewhere in the hierarchy
not covered by this dump's leaf-Xform-only traversal) -- skipped rather
than mislabeled with a wrong tiny box.

Usage: ~/isaacsim/python.sh scripts/p5sim_kitchen_demo.py
"""
import json
import math
import sys

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

sys.path.insert(0, "/home/admin/hera-memory/spatial-memory-m0/spatial-memory")
from spatial_memory import build_system  # noqa: E402
from spatial_memory.schema import (AABB, Detection, Mobility, ObservationEvent,  # noqa: E402
                                    Place, Pose, Submap)

STAGE_PATH = "/home/admin/hera-memory/Locomotion/KitchenRoom/KitchenRoom.usd"
DB_PATH = "/home/admin/hera-memory/work/p5_sim_validation/kitchen_demo.db"
GT_PATH = "/home/admin/hera-memory/work/p5_sim_validation/kitchen_demo_gt.json"
SUBMAP_ID = "kitchen_demo_submap"
PLACE_ID = "kitchen"

# path -> (class_label, mobility)
ALLOWLIST = {
    "/root/Kitchen_InsularShelf_01/Kitchen_InsularShelf": ("kitchen_island", Mobility.STATIC),
    "/root/Kitchen_Cabinet001_01/Kitchen_Cabinet001": ("cabinet", Mobility.STATIC),
    "/root/Kitchen_TopCabinet_01/Kitchen_TopCabinet_Body_01": ("upper_cabinet", Mobility.STATIC),
    "/root/Kitchen_Cabinet002/Kitchen_Cabinet002": ("cabinet", Mobility.STATIC),
    "/root/Table049/Table049": ("table", Mobility.SEMI_STATIC),
    "/root/WallStackOven004_01": ("oven", Mobility.STATIC),
    "/root/Stovetop012_01": ("stovetop", Mobility.STATIC),
    "/root/RangeHood015": ("range_hood", Mobility.STATIC),
    "/root/Microwave017": ("microwave", Mobility.SEMI_STATIC),
    "/root/Dishwasher054_01/Dishwasher054_Body001": ("dishwasher", Mobility.STATIC),
    "/root/Toaster003/Toaster003_Body001": ("toaster", Mobility.SEMI_STATIC),
    "/root/Sink054_01/Sink054_spout": ("sink", Mobility.STATIC),
    "/root/Pot057": ("pot", Mobility.SEMI_STATIC),
    "/root/Kitchen_Flowers001/Kitchen_Flowers001": ("plant", Mobility.SEMI_STATIC),
    "/root/Kitchen_Flowers002/Kitchen_Flowers002_01": ("plant", Mobility.SEMI_STATIC),
    "/root/Kitchen_KnifeHolders001/Kitchen_KnifeHolders001": ("knife_holder", Mobility.SEMI_STATIC),
    "/root/Kitchen_Basket": ("basket", Mobility.SEMI_STATIC),
    "/root/Kitchen_Hookrack001": ("hook_rack", Mobility.STATIC),
    "/root/Kitchen_Shelf001": ("shelf", Mobility.STATIC),
    "/root/Kitchen_Paper": ("paper_roll", Mobility.SEMI_STATIC),
    "/root/Kitchen_Box": ("box", Mobility.SEMI_STATIC),
    "/root/SM_P_Choppingboard_01/Xform/SM_P_Choppingboard_01": ("chopping_board", Mobility.SEMI_STATIC),
    "/root/Kitchen_Bottle": ("bottle", Mobility.SEMI_STATIC),
    "/root/Kitchen_bottle002/Kitchen_bottle002_01": ("bottle", Mobility.SEMI_STATIC),
    "/root/Kitchen_Orange001": ("fruit", Mobility.SEMI_STATIC),
    "/root/Kitchen_Disk001": ("plate", Mobility.SEMI_STATIC),
}


def yaw_from_quat(quat):
    qw = quat.GetReal()
    qx, qy, qz = quat.GetImaginary()
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))


def main():
    stage = Usd.Stage.Open(STAGE_PATH)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    instances = []
    for path, (label, mobility) in ALLOWLIST.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"[skip] invalid prim: {path}")
            continue
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            print(f"[skip] empty bbox: {path}")
            continue
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = xf.ExtractTranslation()
        yaw = yaw_from_quat(xf.ExtractRotationQuat())
        mn, mx = rng.GetMin(), rng.GetMax()
        instances.append({
            "prim_path": path, "class_label": label, "mobility": mobility.value,
            "pose": {"x": pos[0], "y": pos[1], "z": pos[2], "yaw": yaw},
            "aabb": {"min_x": mn[0], "min_y": mn[1], "min_z": mn[2],
                     "max_x": mx[0], "max_y": mx[1], "max_z": mx[2]},
        })

    print(f"{len(instances)}/{len(ALLOWLIST)} allowlisted instances resolved")

    all_x = [i["pose"]["x"] for i in instances]
    all_y = [i["pose"]["y"] for i in instances]
    all_z = [i["pose"]["z"] for i in instances]
    bounds = (min(all_x) - 1, min(all_y) - 1, min(all_z) - 1,
              max(all_x) + 1, max(all_y) + 1, max(all_z) + 1)

    sys_ = build_system(DB_PATH)
    sys_.entities.upsert_submap(Submap(
        submap_id=SUBMAP_ID, anchor_pose_world=Pose(0, 0, 0, 0), bounds=AABB(*bounds)))
    sys_.entities.upsert_place(Place(place_id=PLACE_ID, name="Kitchen", floor_id="F0",
                                      bounds=AABB(*bounds)))

    dets = [Detection(class_label=i["class_label"], pose=Pose(**i["pose"]),
                       embedding=None, attributes={}, score=1.0) for i in instances]
    cx, cy = sum(all_x) / len(all_x), sum(all_y) / len(all_y)
    radius = max(math.hypot(x - cx, y - cy) for x, y in zip(all_x, all_y)) + 1.0
    events = sys_.consolidator.ingest(ObservationEvent(
        submap_id=SUBMAP_ID, place_id=PLACE_ID, robot_id="kitchen_demo",
        detections=dets, view_center=Pose(cx, cy, 0, 0), view_radius=radius))

    for ev, i in zip(events, instances):
        e = sys_.entities.get_object(ev.entity_uuid)
        e.mobility = Mobility(i["mobility"])
        sys_.entities.upsert_object(e)

    print(f"ingested {len(events)} entities -> {DB_PATH}")

    with open(GT_PATH, "w") as f:
        json.dump({"instances": [
            {"class_label": i["class_label"], "pose": i["pose"],
             "aabb": i["aabb"], "place_id": PLACE_ID} for i in instances
        ]}, f, indent=2)
    print(f"wrote {GT_PATH}")

    app.close()


if __name__ == "__main__":
    main()
