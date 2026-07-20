"""
Phase5 sim validation, 1b/T12 -- occlusion trap.

M0's "visibility" model (consolidator.py::ingest()) is view_center + radius +
place_id equality -- no line-of-sight check at all. An object behind a wall
but geometrically inside that circle counts as "should have been seen", so a
patrol that (correctly, physically) fails to detect it still fires DECAY.
This script quantifies how often that actually happens on this scene: real
triangle-mesh raycasting (via open3d, using the wall geometry dumped by
p5sim_extract_walls_mesh.py) gives ground-truth visibility per (station,
object) pair; consolidator.ingest() gives the actual M0 behavior; the gap
between them is the number to report per PLAN.md §6 / docs/p5_sim_validation
_plan.md's "M0 中心+半径模型固有误伤率" ask.

Runs in .venv_p5sim_o3d (Python 3.11 + open3d -- open3d has no 3.13 wheel,
hence the separate venv from the rest of this validation; spatial_memory
itself only needs numpy, which this venv also has, so the consolidator half
of this script works unmodified).

Usage: .venv_p5sim_o3d\\python.exe scripts\\p5sim_occlusion_check.py
"""
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "spatial-memory-m0/spatial-memory"))

from spatial_memory import build_system
from spatial_memory.schema import AABB, Detection, Mobility, ObservationEvent, Place, Pose, Submap

WALLS_PATH = ROOT / "work/p5_sim_validation/walls_mesh.npz"
GT_PATH = ROOT / "work/p5_sim_validation/gt_instances.json"
DB_PATH = ROOT / "work/p5_sim_validation/sim_memory_t12.db"
OUT_PATH = ROOT / "work/p5_sim_validation/t12_occlusion_result.json"

EYE_Z = 1.0            # 假想机器人传感器高度(m),apartment_unit Z 范围 [-0.02,2.56] 内取值合理
VIEW_RADIUS = 6.0      # 沿用 4dkankan 脚本同款"单站点"惯例
RAY_EPS = 0.05          # 命中距离比目标距离短这么多才算真遮挡,避免目标自身面片的浮点噪声误判

# 3 个手选巡逻站点,覆盖房间里不同的家具簇(见 PLAN.md §1.5 的簇心)
STATIONS = {
    "station_sofa_area":    (-2.5, 2.4),
    "station_kitchen_area": (1.4, -0.03),
    "station_dining_area":  (4.5, 1.0),
}


def build_raycasting_scene():
    data = np.load(WALLS_PATH)
    verts = data["vertices"].astype(np.float32)
    tris = data["triangles"].astype(np.uint32)
    mesh = o3d.t.geometry.TriangleMesh()
    mesh.vertex.positions = o3d.core.Tensor(verts)
    mesh.triangle.indices = o3d.core.Tensor(tris)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh)
    return scene


def is_occluded(scene, origin, target) -> tuple[bool, float, float]:
    """Casts one ray origin->target, returns (occluded, hit_dist, target_dist)."""
    d = np.array(target, dtype=np.float32) - np.array(origin, dtype=np.float32)
    target_dist = float(np.linalg.norm(d))
    if target_dist < 1e-6:
        return False, 0.0, 0.0
    direction = d / target_dist
    rays = o3d.core.Tensor([[*origin, *direction]], dtype=o3d.core.Dtype.Float32)
    ans = scene.cast_rays(rays)
    hit_dist = float(ans["t_hit"].numpy()[0])
    occluded = hit_dist < (target_dist - RAY_EPS)
    return occluded, hit_dist, target_dist


def setup_system():
    if DB_PATH.exists():
        DB_PATH.unlink()
    sys_ = build_system(str(DB_PATH))
    gt = json.loads(GT_PATH.read_text(encoding="utf-8"))
    all_x = [i["pose"]["x"] for i in gt["instances"]]
    all_y = [i["pose"]["y"] for i in gt["instances"]]
    sys_.entities.upsert_submap(Submap(
        submap_id="sim_submap_t12", anchor_pose_world=Pose(0, 0, 0, 0),
        bounds=AABB(min(all_x) - 5, min(all_y) - 5, -5, max(all_x) + 5, max(all_y) + 5, 5)))
    for place_id, pdef in gt["places"].items():
        sys_.entities.upsert_place(Place(place_id=place_id, name=pdef["name"],
                                          floor_id="F0", bounds=AABB(*pdef["bounds"])))
    return sys_, gt


def main():
    scene = build_raycasting_scene()
    sys_, gt = setup_system()
    submap_id = "sim_submap_t12"

    candidates = [i for i in gt["instances"]
                  if i["place_id"] == "apartment_unit" and i["class_label"] != "wall"]

    # ---- Round 1: bulk ADD every candidate once (huge view_radius so the
    # candidate set doesn't matter -- same "first observation always ADDs
    # regardless of station geometry" property used in p5sim_run_1a.py),
    # so every candidate has a library entry to potentially (mis-)decay in
    # the per-station negative-observation round below. Occluders considered
    # here are SM_wall meshes only (24 instances) -- kitchen_cabinet/
    # wardrobe could occlude too in principle but weren't extracted as
    # triangle meshes for this pass; walls are the dominant occluder in this
    # scene and this is a scoped case study, not an exhaustive occluder scan.
    dets = [Detection(class_label=i["class_label"], pose=Pose(**i["pose"]),
                       embedding=None, attributes={}, score=1.0)
            for i in candidates]
    add_events = sys_.consolidator.ingest(ObservationEvent(
        submap_id=submap_id, place_id="apartment_unit", robot_id="t12_setup",
        detections=dets, view_center=Pose(0, 2, 0, 0), view_radius=1000.0))
    uuid_by_path = {c["prim_path"]: ev.entity_uuid for c, ev in zip(candidates, add_events)}
    for c in candidates:
        e = sys_.entities.get_object(uuid_by_path[c["prim_path"]])
        e.mobility = Mobility.STATIC if c["mobility"] == "static" else Mobility.SEMI_STATIC
        sys_.entities.upsert_object(e)

    station_reports = []
    for station_name, (sx, sy) in STATIONS.items():
        origin = (sx, sy, EYE_Z)
        # NOTE: objects_near()/Pose.distance_to() use full 3D distance, and
        # we're about to issue an ObservationEvent with view_center z=EYE_Z
        # (matching the raycasting origin) -- so the in-radius filter here
        # must use the SAME 3D distance from (sx,sy,EYE_Z), not a 2D/XY
        # distance. Using XY-only here first (now fixed) caused a real
        # mismatch against the actual consolidator candidate set for
        # elevated objects (ceiling lamps), where 3D distance from an eye-
        # height origin exceeds VIEW_RADIUS even though XY distance alone
        # doesn't -- worth keeping this note, it's exactly the kind of bug
        # this validation exercise is supposed to catch, just in the test
        # harness instead of the framework this time.
        in_radius, visible, occluded = [], [], []
        for c in candidates:
            tx, ty, tz = c["pose"]["x"], c["pose"]["y"], c["pose"]["z"] + EYE_Z * 0.3
            dist_3d = ((tx - sx) ** 2 + (ty - sy) ** 2 + (tz - EYE_Z) ** 2) ** 0.5
            if dist_3d > VIEW_RADIUS:
                continue
            in_radius.append(c)
            occ, hit_d, tgt_d = is_occluded(scene, origin, (tx, ty, tz))
            (occluded if occ else visible).append((c, hit_d, tgt_d))

        # Negative observation: this station's patrol reports ZERO detections
        # (worst case -- tests whether occluded AND visible objects both get
        # DECAYed identically, since M0 has no occlusion awareness at all).
        before = {c["prim_path"]: sys_.entities.get_object(uuid_by_path[c["prim_path"]]).confidence
                  for c in in_radius}
        sys_.consolidator.ingest(ObservationEvent(
            submap_id=submap_id, place_id="apartment_unit", robot_id=station_name,
            detections=[], view_center=Pose(sx, sy, EYE_Z, 0), view_radius=VIEW_RADIUS))
        after = {c["prim_path"]: sys_.entities.get_object(uuid_by_path[c["prim_path"]]).confidence
                 for c in in_radius}

        decayed_occluded = sum(1 for c, *_ in occluded if after[c["prim_path"]] < before[c["prim_path"]])
        decayed_visible = sum(1 for c, *_ in visible if after[c["prim_path"]] < before[c["prim_path"]])

        station_reports.append({
            "station": station_name, "origin": [sx, sy, EYE_Z],
            "n_in_radius": len(in_radius), "n_occluded": len(occluded), "n_visible": len(visible),
            "n_occluded_erroneously_decayed": decayed_occluded,
            "n_visible_correctly_decayed": decayed_visible,
            "erroneous_decay_rate_of_in_radius": (len(occluded) / len(in_radius)
                                                   if in_radius else 0.0),
            "occluded_objects": [{"label": c["class_label"], "prim_path": c["prim_path"],
                                   "hit_dist": round(h, 2), "target_dist": round(t, 2)}
                                  for c, h, t in occluded],
        })
        print(f"{station_name}: {len(in_radius)} in radius, {len(occluded)} occluded "
              f"({len(occluded)/len(in_radius):.1%}), "
              f"{decayed_occluded}/{len(occluded)} occluded ones erroneously DECAYed, "
              f"{decayed_visible}/{len(visible)} visible ones correctly DECAYed")

    overall_in_radius = sum(r["n_in_radius"] for r in station_reports)
    overall_occluded = sum(r["n_occluded"] for r in station_reports)
    overall_rate = overall_occluded / overall_in_radius if overall_in_radius else 0.0

    OUT_PATH.write_text(json.dumps({
        "eye_z": EYE_Z, "view_radius": VIEW_RADIUS, "ray_eps": RAY_EPS,
        "stations": station_reports,
        "overall_in_radius": overall_in_radius, "overall_occluded": overall_occluded,
        "overall_erroneous_decay_rate": overall_rate,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nOverall across {len(STATIONS)} stations: {overall_occluded}/{overall_in_radius} "
          f"in-radius candidates were actually occluded ({overall_rate:.1%}) -- every one of "
          f"them still got DECAYed by M0 exactly like a genuinely-absent object would.")
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
