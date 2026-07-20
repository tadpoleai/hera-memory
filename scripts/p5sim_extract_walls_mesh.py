"""
Phase5 sim validation, step 1b/T12 -- extracts real triangle-mesh geometry
(not just AABBs) for every SM_wall instance in scene_04.usd, baked into world
space. T12 needs actual line-of-sight raycasting against wall geometry, which
open3d does -- but open3d has no Python 3.13 wheel (this project's main venv
is 3.13), so this runs in the pxr venv (.venv_p5sim, Python 3.13) and dumps a
plain .npz that a SEPARATE Python 3.11 + open3d venv (.venv_p5sim_o3d) reads
in p5sim_occlusion_check.py. Two venvs, one per hard-to-combine dependency,
rather than fighting for one environment with both -- see PLAN.md §4 for why.

Confirmed by direct inspection (see PLAN.md): each SM_wall_NN_0 Xform has one
child Mesh with faceVertexCounts all == 3 (already triangulated, no fan
triangulation needed).

Usage: .venv_p5sim\\Scripts\\python.exe scripts\\p5sim_extract_walls_mesh.py
"""
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

ROOT = Path(__file__).resolve().parent.parent
STAGE_PATH = (r"C:\Users\EDY\Downloads\Lightwheel_OpenSource\Lightwheel_OpenSource"
              r"\Locomotion\Apartment\scene_04.usd")
OUT_PATH = ROOT / "work/p5_sim_validation/walls_mesh.npz"


def main():
    stage = Usd.Stage.Open(STAGE_PATH)
    all_verts = []
    all_tris = []
    n_wall_prims = 0

    for prim in stage.Traverse():
        if prim.GetTypeName() != "Xform":
            continue
        if not prim.GetName().startswith("SM_wall"):
            continue
        mesh_children = [c for c in prim.GetChildren() if c.GetTypeName() == "Mesh"]
        if not mesh_children:
            continue
        n_wall_prims += 1
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for mesh_prim in mesh_children:
            mesh = UsdGeom.Mesh(mesh_prim)
            pts = mesh.GetPointsAttr().Get()
            fvc = mesh.GetFaceVertexCountsAttr().Get()
            fvi = mesh.GetFaceVertexIndicesAttr().Get()
            if not pts or not fvc:
                continue
            assert all(c == 3 for c in fvc), \
                f"{mesh_prim.GetPath()} has non-triangle faces, need fan triangulation"

            # mesh points are in the Mesh's own local space; the child Mesh
            # prim may carry no extra transform of its own here (confirmed
            # single-level Xform->Mesh with no intermediate xformOp on the
            # Mesh itself), so bake with the parent Xform's world transform.
            mesh_xf = UsdGeom.Xformable(mesh_prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            base_idx = len(all_verts)
            for p in pts:
                world_p = mesh_xf.Transform(p)
                all_verts.append([world_p[0], world_p[1], world_p[2]])
            idx = np.array(fvi, dtype=np.int64).reshape(-1, 3)
            all_tris.append(idx + base_idx)

    verts = np.array(all_verts, dtype=np.float32)
    tris = np.concatenate(all_tris, axis=0).astype(np.int64)
    print(f"{n_wall_prims} SM_wall prims -> {len(verts)} verts, {len(tris)} triangles")

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    np.savez(OUT_PATH, vertices=verts, triangles=tris)
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
