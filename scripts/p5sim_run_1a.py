"""
Phase5 sim validation, step 1a -- static direct-inject, same-process
ingest + 4-layer audit, mirroring scripts/p5_ingest_4dkankan.py's structure
(and its documented pitfalls -- see docs/phase5_schema.md §4.4/§4.3, both
apply here too):

  1. NumpyVectorIndex is in-memory only, never persisted to SQLite -- any
     script that ingests AND queries must do both in the same process. This
     script does both in main(), no cross-process split.
  2. consolidator._do_add() does not transfer `mobility` from Detection (the
     dataclass has no such field) -- new entities all land on the
     ObjectInstance default (SEMI_STATIC) regardless of true class. Patched
     here via a second pass after ingest, same pattern as the 4dkankan
     script's embedding_model patch. This one actually matters here (unlike
     4dkankan, which never exercised DECAY) because 1b's T4/T5/T6/T7 all
     depend on mobility-driven decay rates.

Ground truth = gt_instances.json itself (p5sim_extract_gt.py's own output) --
per docs/p5_sim_validation_plan.md §3, this is intentional: the point of 1a
is to isolate memory-layer bugs from data-quality issues, not to grade
extraction accuracy. Expected result is near-perfect on all three metrics;
any real gap is a memory-layer bug, not sensor/perception noise.

Run 1 (this script, default): embedding=None for every Detection -- matching
falls back to exact label match (consolidator._similarity's no-embedding
branch). Isolates consolidator mechanics from CLIP quality entirely, per
plan §2.4.

Usage: .venv_p5sim\\Scripts\\python.exe scripts\\p5sim_run_1a.py
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "spatial-memory-m0/spatial-memory"))

from spatial_memory import build_system
from spatial_memory.eval import EntityValidator, EvalConfig, GroundTruthEntity
from spatial_memory.schema import AABB, Detection, Mobility, ObservationEvent, Place, Pose, Submap

GT_PATH = ROOT / "work/p5_sim_validation/gt_instances.json"
DB_PATH = ROOT / "work/p5_sim_validation/sim_memory.db"
SUBMAP_ID = "sim_submap_scene04"

CLASS_TO_MOBILITY = {
    "tree": Mobility.STATIC, "wall": Mobility.STATIC, "bar_panel": Mobility.STATIC,
    "window": Mobility.STATIC, "building": Mobility.STATIC, "kitchen_cabinet": Mobility.STATIC,
    "door": Mobility.STATIC, "wardrobe": Mobility.STATIC, "fence": Mobility.STATIC,
    "floor": Mobility.STATIC, "floor_tile": Mobility.STATIC,
    "chair": Mobility.SEMI_STATIC, "lamp": Mobility.SEMI_STATIC, "pillow": Mobility.SEMI_STATIC,
    "table": Mobility.SEMI_STATIC, "carpet": Mobility.SEMI_STATIC, "plant": Mobility.SEMI_STATIC,
    "sofa": Mobility.SEMI_STATIC,
}


def load_gt():
    return json.loads(GT_PATH.read_text(encoding="utf-8"))


def place_view_station(bounds: tuple) -> tuple[Pose, float]:
    """One virtual robot station per place: centroid of the AABB, radius =
    half the AABB's horizontal diagonal + 1m margin, so a single
    ObservationEvent's view_radius covers every instance assigned to that
    place. This differs from 4dkankan's fixed 6.0m -- that script's zones
    (k-means, capped at ~7.2m span) all fit under one 6m-radius station;
    apartment_unit here is ~13m across and courtyard is ~80m across, a fixed
    6m would leave real instances outside view_radius. Doesn't affect 1a's
    ADD-only pass (view_radius only gates the *candidate set for matching
    existing entities*, and the DB starts empty), but matters for 1b's decay
    tests, so getting it geometrically honest now avoids a silent bug later.
    """
    min_x, min_y, _, max_x, max_y, _ = bounds
    cx, cy = (min_x + max_x) / 2, (min_y + max_y) / 2
    half_diag = math.hypot(max_x - min_x, max_y - min_y) / 2
    return Pose(cx, cy, 0.0, 0.0), half_diag + 1.0


def ingest(db_path: str | None = None, fresh: bool = True):
    path = Path(db_path) if db_path else DB_PATH
    if fresh and path.exists():
        path.unlink()

    gt = load_gt()
    sys_ = build_system(str(path))

    all_x = [i["pose"]["x"] for i in gt["instances"]]
    all_y = [i["pose"]["y"] for i in gt["instances"]]
    all_z = [i["pose"]["z"] for i in gt["instances"]]
    submap = Submap(
        submap_id=SUBMAP_ID, anchor_pose_world=Pose(0, 0, 0, 0),
        bounds=AABB(min(all_x) - 2, min(all_y) - 2, min(all_z) - 2,
                    max(all_x) + 2, max(all_y) + 2, max(all_z) + 2))
    sys_.entities.upsert_submap(submap)

    for place_id, pdef in gt["places"].items():
        b = pdef["bounds"]
        place = Place(place_id=place_id, name=pdef["name"], floor_id="F0",
                      bounds=AABB(*b))
        sys_.entities.upsert_place(place)

    by_place: dict[str, list] = {}
    for inst in gt["instances"]:
        by_place.setdefault(inst["place_id"], []).append(inst)

    total_events = 0
    for place_id, insts in by_place.items():
        dets = [Detection(
            class_label=i["class_label"],
            pose=Pose(**i["pose"]),
            embedding=None,
            attributes=dict(i["attributes"]),
            score=1.0,
        ) for i in insts]
        view_center, view_radius = place_view_station(gt["places"][place_id]["bounds"])
        obs = ObservationEvent(
            submap_id=SUBMAP_ID, place_id=place_id, robot_id="sim_validation_1a",
            detections=dets, view_center=view_center, view_radius=view_radius)
        events = sys_.consolidator.ingest(obs)
        total_events += len(events)
        print(f"{place_id}: ingested {len(dets)} detections (radius={view_radius:.1f}m) "
              f"-> {len(events)} events")

    print(f"Total events: {total_events}")

    # consolidator._do_add() never sets mobility (Detection has no such
    # field) -- patch every entity's mobility from CLASS_TO_MOBILITY, same
    # "补丁式修正" pattern as p5_ingest_4dkankan.py's embedding_model fix.
    n_patched = 0
    for place in sys_.entities.all_places():
        for e in sys_.entities.objects_in_place(place.place_id):
            correct = CLASS_TO_MOBILITY.get(e.class_label)
            if correct is not None and e.mobility != correct:
                e.mobility = correct
                sys_.entities.upsert_object(e)
                n_patched += 1
    print(f"mobility patched on {n_patched} entities")

    print(f"DB written to {path}")
    return sys_


def audit(sys_):
    gt = load_gt()
    gts = [GroundTruthEntity(label=i["class_label"], pose=Pose(**i["pose"]),
                              place_id=i["place_id"], attributes=dict(i["attributes"]))
           for i in gt["instances"]]

    # match_max_dist tight on purpose: this is perfect-ground-truth data, not
    # noisy real sensor output, so we should NOT need 4dkankan's loosened
    # 1.2m gate. Default EvalConfig (0.75m) should already be generous given
    # poses round-trip through the DB unchanged in a 1a ADD-only pass.
    cfg = EvalConfig(match_max_dist=0.75, require_label=True)
    validator = EntityValidator(sys_.entities, sys_.query, cfg)
    report = validator.audit_snapshot(gts)
    return report


def main():
    sys_ = ingest(fresh=True)
    report = audit(sys_)
    print()
    print(report.pretty())

    # pass/fail per docs/p5_sim_validation_plan.md §0
    checks = [
        ("F1 >= 0.95", report.f1 >= 0.95, report.f1),
        ("pose_mae <= 0.05m", report.pose_mae <= 0.05, report.pose_mae),
        ("place_accuracy == 1.0", report.place_accuracy == 1.0, report.place_accuracy),
    ]
    print()
    for name, ok, val in checks:
        print(f"{'PASS' if ok else 'FAIL'}  {name}  (actual={val:.4f})")

    out = ROOT / "work/p5_sim_validation/run_1a_result.json"
    out.write_text(json.dumps({
        "n_gt": report.n_gt, "n_mem": report.n_mem, "tp": report.tp,
        "fp": report.fp, "fn": report.fn,
        "precision": report.precision, "recall": report.recall, "f1": report.f1,
        "pose_mae": report.pose_mae, "attr_accuracy": report.attr_accuracy,
        "place_accuracy": report.place_accuracy,
        "fp_labels": report.fp_labels, "fn_labels": report.fn_labels,
        "checks": [{"name": n, "pass": ok, "value": v} for n, ok, v in checks],
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
