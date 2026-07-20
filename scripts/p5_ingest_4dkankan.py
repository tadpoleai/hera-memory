"""
Phase 5 technical validation (using the 4dkankan mall data, per user's
explicit choice -- NOT the CLAUDE.md-literal home dataset; see STATUS.md for
why: our own home capture only has 6 usable instances, too thin for
meaningful P/R/F1, while the 4dkankan run produced 139 instances across a
real multi-zone building. This validates the spatial_memory framework
mechanics -- ingestion, matching/consolidation, CLIP-based semantic search,
4-layer audit -- decoupled from our own capture pipeline's data-quality
issues, continuing the same isolation strategy used for the Phase4
validation).

Steps:
  1. Places: K-means (k=8) on the 26 valid capture points' horizontal
     position (excludes batch4's 7 points -- that batch's ConceptGraphs run
     produced one catastrophically over-merged "lamp" instance spanning the
     whole corridor, no usable per-object data, see STATUS.md). Max
     resulting zone extent is 7.2m, room-scale.
  2. Single submap anchored at world origin (matches CLAUDE.md's home-data
     approach: this whole scene is one already-globally-consistent mesh,
     no separate submap boundaries from our own pipeline to respect).
  3. Y-up (4dkankan) -> Z-up (framework convention, matches SyntheticWorld's
     AABB(0,0,0,30,12,3) where Z is the small "height" axis) via the same
     handedness-preserving transform used for the WebGL viewer:
     (x,y,z) -> (x, z, -y).
  4. Each of the 139 instances (batch4 excluded) is assigned to its nearest
     zone by horizontal distance to zone center, becomes one Detection.
     One ObservationEvent per zone (all its instances at once), embedding =
     the instance's own 512-dim CLIP image feature (already computed by
     ConceptGraphs, same model text queries will be embedded with later --
     see p5_query_demo.py).

Usage: python3 p5_ingest_4dkankan.py
"""
import csv
import json
from pathlib import Path

import numpy as np
from sklearn.cluster import KMeans

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent /
                       "spatial-memory-m0/spatial-memory"))
from spatial_memory import build_system
from spatial_memory.schema import AABB, Detection, ObservationEvent, Place, Pose, Submap

ROOT = Path(__file__).resolve().parent.parent
SRC_POSES = Path("/home/fred/Code/mvp2/spatialAI/capture/MTVUOIzO6U/poses.csv")
DB_PATH = ROOT / "work/phase5_4dkankan_validation/mall_memory.db"
SUBMAP_ID = "mall_submap_MTVUOIzO6U"
N_ZONES = 8

# batch4 excluded: catastrophic merge failure (929 detections -> 1 instance,
# 11m bbox), see STATUS.md "batch4 发现一个严重的合并失败案例"
VALID_POINTS = ["0", "1", "2", "3", "5", "11", "15", "16", "17", "18",
                 "19", "20", "21", "22", "29", "30", "31", "32",
                 "33", "34", "35", "36", "37", "38", "39", "40"]
BATCH_FILES = [
    ROOT / "work/phase4_realsee_validation/instances_result/instances.json",  # points 0,1
    ROOT / "work/phase4_realsee_validation/batch1/instances_result/instances.json",
    ROOT / "work/phase4_realsee_validation/batch2/instances_result/instances.json",
    ROOT / "work/phase4_realsee_validation/batch3/instances_result/instances.json",
]


def yup_to_zup(v):
    """(x,y,z) Y-up -> (x,-z,y) Z-up. Handedness-preserving (proper +90deg
    rotation about X) -- maps Y-up's "up" (0,1,0) to Z-up's "up" (0,0,1).

    FIXED 2026-07: the original formula here was (x,z,-y), a *different*
    proper rotation (-90deg about X) that sends Y-up's up to Z-up's DOWN
    (and flips one horizontal axis too, i.e. the whole scene was rotated
    180deg about the Z-up X axis relative to correct). This was invisible
    in the 3D orbit viewer (a symmetric-ish indoor point cloud viewed from
    outside doesn't obviously look "wrong side up") but became obvious the
    moment a first-person panorama mode was added -- ceiling/floor swapped,
    and one horizontal pair of cubemap faces also came out wrong. Note this
    bug does NOT change any Phase 5 audit numbers (F1/pose-MAE/recall etc.):
    those all reduce to pairwise Euclidean distances or AABB-containment
    tests computed AFTER a consistent transform of both memory and ground
    truth, and a proper rotation preserves distances -- only the human-facing
    visualization was affected.
    """
    return np.array([v[0], -v[2], v[1]])


def build_places():
    rows = {r["point_id"]: r for r in csv.DictReader(open(SRC_POSES))}
    pos_yup = np.array([[float(rows[p]["x"]), float(rows[p]["y"]), float(rows[p]["z"])]
                         for p in VALID_POINTS])
    horiz = pos_yup[:, [0, 2]]  # X,Z horizontal plane in Y-up
    km = KMeans(n_clusters=N_ZONES, n_init=10, random_state=0).fit(horiz)
    labels = km.labels_
    centers_zup = np.array([yup_to_zup([c[0], 0.0, c[1]]) for c in km.cluster_centers_])
    return labels, centers_zup


def load_all_instances():
    all_insts = []
    for f in BATCH_FILES:
        d = json.loads(f.read_text())
        all_insts.extend(d["instances"])
    return all_insts


def ingest(db_path: str | None = None, fresh: bool = True):
    """Runs the full ingest and returns the live SpatialMemorySystem object.

    IMPORTANT: NumpyVectorIndex (embeddings) is pure in-memory, never written
    to the SQLite DB (schema.py's to_row()/store.py's _obj_to_json() both
    explicitly drop the embedding field -- "向量单独存索引,不进快照行").
    A fresh `build_system()` call in a NEW process starts with an EMPTY
    vindex even if the DB file already has 139 entities from a prior run --
    semantic_search would silently return nothing (this is exactly what
    happened the first time p5_audit_4dkankan.py ran standalone). Any script
    that needs both persisted entities AND working retrieval must call this
    function directly (not just point build_system() at the existing db_path)
    so ingestion happens in the same process/vindex instance that will later
    query it. `fresh=True` (default) deletes any existing DB file first, so
    repeated runs don't pile up duplicate ADD events (uuids are fresh every
    call, upsert_object can't dedupe them against old rows by content).
    """
    path = Path(db_path) if db_path else DB_PATH
    if fresh and path.exists():
        path.unlink()

    labels, centers_zup = build_places()
    instances = load_all_instances()
    print(f"{len(instances)} instances (batch4 excluded), {N_ZONES} zones")

    sys_ = build_system(str(path))

    # ---- submap (single, world-anchored) ----
    all_pos_zup = np.array([yup_to_zup(o["position"]) for o in instances])
    submap_bounds = AABB(
        float(all_pos_zup[:, 0].min()) - 2, float(all_pos_zup[:, 1].min()) - 2, -6.0,
        float(all_pos_zup[:, 0].max()) + 2, float(all_pos_zup[:, 1].max()) + 2, 2.0)
    submap = Submap(submap_id=SUBMAP_ID, anchor_pose_world=Pose(0, 0, 0, 0),
                     bounds=submap_bounds)
    sys_.entities.upsert_submap(submap)

    # ---- assign each instance to nearest zone center (horizontal) ----
    zone_ids = [f"zone_{i}" for i in range(N_ZONES)]
    inst_positions_zup = all_pos_zup
    dists = np.linalg.norm(
        inst_positions_zup[:, None, :2] - centers_zup[None, :, :2], axis=2)
    assigned_zone = np.argmin(dists, axis=1)

    # ---- Place AABBs: bbox of assigned instances + 1.5m padding, generous
    # vertical range shared across zones (whole scene's floor-to-ceiling) ----
    z_all = inst_positions_zup[:, 2]
    z_lo, z_hi = float(z_all.min()) - 1.0, float(z_all.max()) + 1.0
    for zi, zone_id in enumerate(zone_ids):
        members = inst_positions_zup[assigned_zone == zi]
        if len(members) == 0:
            continue
        pad = 1.5
        bounds = AABB(
            float(members[:, 0].min()) - pad, float(members[:, 1].min()) - pad, z_lo,
            float(members[:, 0].max()) + pad, float(members[:, 1].max()) + pad, z_hi)
        place = Place(place_id=zone_id, name=f"Zone {zi}", floor_id="F0", bounds=bounds)
        sys_.entities.upsert_place(place)
        print(f"{zone_id}: n_instances={len(members)} bounds=({bounds.min_x:.1f},{bounds.min_y:.1f})"
              f"-({bounds.max_x:.1f},{bounds.max_y:.1f})")

    # ---- ingest: one ObservationEvent per zone, all its instances at once ----
    total_events = 0
    for zi, zone_id in enumerate(zone_ids):
        idxs = np.where(assigned_zone == zi)[0]
        if len(idxs) == 0:
            continue
        dets = []
        for i in idxs:
            o = instances[i]
            pos_zup = inst_positions_zup[i]
            dets.append(Detection(
                class_label=o["category"],
                pose=Pose(float(pos_zup[0]), float(pos_zup[1]), float(pos_zup[2]), 0.0),
                embedding=np.array(o["clip_feature"], dtype=np.float32),
                attributes={},
                score=float(o["confidence"]),
            ))
        center = centers_zup[zi]
        view_center = Pose(float(center[0]), float(center[1]), 0.0, 0.0)
        obs = ObservationEvent(
            submap_id=SUBMAP_ID, place_id=zone_id, robot_id="4dkankan_validation",
            detections=dets, view_center=view_center, view_radius=6.0)
        events = sys_.consolidator.ingest(obs)
        total_events += len(events)
        print(f"{zone_id}: ingested {len(dets)} detections -> {len(events)} events")

    print(f"\nTotal events: {total_events}")

    # consolidator._do_add() doesn't set embedding_model (Detection has no
    # such field) -- leaves the ObjectInstance default "toy-trigram@v0",
    # which would be a lie now that embeddings are real CLIP vectors.
    # Patch every entity's embedding_model field to match reality.
    n_active = 0
    for zone_id in zone_ids:
        for e in sys_.entities.objects_in_place(zone_id):
            e.embedding_model = "ViT-B-32/laion2b_s34b_b79k"
            sys_.entities.upsert_object(e)
            n_active += 1

    print(f"DB written to {path}")
    print(f"Active entities across all zones: {n_active}")
    return sys_


if __name__ == "__main__":
    ingest()
