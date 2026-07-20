"""
Phase 4 validation: adapt an already-captured 4dkankan (四维看看) scene
(decoded per ~/.claude/skills/4dkankan-pipeline.md) into the same
rgb/depth/poses.json/intrinsics.json bundle format p4_prepare.py produces,
so the existing ConceptGraphs FC GPU endpoint can be validated against
clean, dense, vendor-quality data -- decoupled from our own capture
pipeline's known depth-sparsity / extrinsic-calibration issues.

Source data (already decoded, not modified): capture/MTVUOIzO6U/
  sim/scene_geo.obj -- triangulated world mesh, Y-up, meters (96561 verts /
                  65990 tris). Depth is rendered via Open3D raycasting
                  against this mesh directly (not the decimated scene.ply
                  point sample -- that gave <15% pixel coverage per face,
                  the mesh gives ~98%, since it has real triangle
                  connectivity instead of a sparse vertex sample).
  poses.csv    -- point_id,uuid,floor_id,x,y,z,qx,qy,qz,qw (Y-up meters;
                  quaternion NOT used here -- skybox faces are axis-aligned
                  in world space, not rotated by this per-point quaternion,
                  per the viewer.html reference loader)
  sim/skybox/{uuid_nohyphen}_skybox{0-5}.jpg  -- 512x512 cubemap faces

Physical face directions (determined from face CONTENT, not the Three.js
CubeTexture *slot* assignment in the skill doc -- that table describes a
GPU-texture-sampling convention with an extra vertical flip baked in, which
is irrelevant here since we render depth directly against each face's real
capture direction):
  skybox0 -> physically "up"   (content = ceiling, confirmed by inspection)
  skybox5 -> physically "down" (content = floor, confirmed by inspection)
  skybox1..4 -> the horizontal ring, 90deg apart in yaw, in order (sign of
                the yaw step is arbitrary/self-consistent here since there's
                no external compass reference -- verified after the fact by
                checking the rendered depth's structure against the RGB).

Camera convention matches what run_extraction.py expects (pose_to_matrix +
build_cam_K): standard CV camera axes (x=right, y=down, z=forward), i.e.
T_world_camera columns = [right_world, down_world, forward_world].

Usage:
  python3 p4_adapt_4dkankan.py                              # default: points 0,1
  python3 p4_adapt_4dkankan.py <comma_sep_point_ids> <bundle_subdir>
  e.g. python3 p4_adapt_4dkankan.py 2,3,5,11,15,16,17 batch1
       -> writes work/phase4_realsee_validation/batch1/upload_bundle/
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image
from scipy.spatial.transform import Rotation

SRC = Path("/home/fred/Code/mvp2/spatialAI/capture/MTVUOIzO6U")
ROOT = Path(__file__).resolve().parent.parent

if len(sys.argv) >= 3:
    POINT_IDS = sys.argv[1].split(",")
    BUNDLE = ROOT / "work/phase4_realsee_validation" / sys.argv[2] / "upload_bundle"
else:
    POINT_IDS = ["0", "1"]  # default: two adjacent points, 3.93m apart
    BUNDLE = ROOT / "work/phase4_realsee_validation/upload_bundle"

FACE_SIZE = 512
FOV_DEG = 90.0

# face index -> (tag, physical look direction in the point's LOCAL horizontal
# ring frame, before applying the point's own yaw offset -- see build below)
HORIZONTAL_FACES = ["skybox1", "skybox2", "skybox3", "skybox4"]  # 90deg steps
UP_FACE = "skybox0"


def build_raycasting_scene(obj_path):
    mesh = o3d.io.read_triangle_mesh(str(obj_path))
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)
    return scene


def cv_basis(forward, world_up=np.array([0.0, 1.0, 0.0])):
    """Right-handed CV camera basis (right, down, forward) as columns of R,
    such that right x down = forward. Falls back to world +Z as the
    reference axis when forward is (near-)parallel to world_up (the "up"
    face)."""
    forward = forward / np.linalg.norm(forward)
    ref = world_up
    if abs(np.dot(forward, world_up)) > 0.99:
        ref = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, ref)
    right = right / np.linalg.norm(right)
    down = np.cross(forward, right)
    R = np.stack([right, down, forward], axis=1)  # columns
    return R


def render_face_depth(scene, cam_pos, R_world_cam, size, fov_deg):
    """R_world_cam: 3x3, columns = [right_world, down_world, forward_world]
    (camera->world). Open3D wants the world->camera extrinsic (4x4)."""
    f = (size / 2) / np.tan(np.radians(fov_deg) / 2)
    cx = cy = size / 2
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)

    Rt = R_world_cam.T
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = Rt
    extrinsic[:3, 3] = -Rt @ cam_pos

    rays = scene.create_rays_pinhole(intrinsic_matrix=o3d.core.Tensor(K),
                                      extrinsic_matrix=o3d.core.Tensor(extrinsic),
                                      width_px=size, height_px=size)
    depth = scene.cast_rays(rays)["t_hit"].numpy()

    hole_before = (~np.isfinite(depth)).mean()
    filled = depth.copy()
    for _ in range(2):
        pad = np.pad(filled, 1, mode="edge")
        stack = np.stack([pad[i:i + size, j:j + size] for i in range(3) for j in range(3)], axis=0)
        finite = np.isfinite(stack)
        neighbor_mean = np.where(finite, stack, 0).sum(axis=0) / np.maximum(finite.sum(axis=0), 1)
        still_hole = ~np.isfinite(filled)
        filled = np.where(still_hole & (finite.sum(axis=0) > 0), neighbor_mean, filled)
    hole_after = (~np.isfinite(filled)).mean()
    filled[~np.isfinite(filled)] = 0.0
    return filled.astype(np.float32), hole_before, hole_after


def main():
    scene = build_raycasting_scene(SRC / "sim/scene_geo.obj")
    rows = {r["point_id"]: r for r in csv.DictReader(open(SRC / "poses.csv"))}

    (BUNDLE / "rgb").mkdir(parents=True, exist_ok=True)
    (BUNDLE / "depth").mkdir(parents=True, exist_ok=True)

    poses = {}
    f_val = (FACE_SIZE / 2) / np.tan(np.radians(FOV_DEG) / 2)
    cx_val = cy_val = FACE_SIZE / 2
    intrinsics = {"_all_faces": {"fx": f_val, "fy": f_val, "cx": cx_val, "cy": cy_val,
                                  "width": FACE_SIZE, "height": FACE_SIZE, "fov_deg": FOV_DEG}}

    hole_stats = []
    for pid in POINT_IDS:
        row = rows[pid]
        uuid_nohyphen = row["uuid"].replace("-", "")
        cam_pos = np.array([float(row["x"]), float(row["y"]), float(row["z"])])

        # horizontal ring: skybox1 = arbitrary reference "+Z", 90deg yaw steps
        for k, face in enumerate(HORIZONTAL_FACES):
            theta = -90.0 * k  # sign is arbitrary/self-consistent, see module docstring
            forward = Rotation.from_euler("y", theta, degrees=True).apply([0.0, 0.0, 1.0])
            R = cv_basis(np.array(forward))
            _emit(pid, face, uuid_nohyphen, cam_pos, R, scene, poses, hole_stats)

        # up face: physically points at world +Y
        R = cv_basis(np.array([0.0, 1.0, 0.0]))
        _emit(pid, UP_FACE, uuid_nohyphen, cam_pos, R, scene, poses, hole_stats)

    (BUNDLE / "poses.json").write_text(json.dumps(poses, indent=2, ensure_ascii=False))
    (BUNDLE / "intrinsics.json").write_text(json.dumps(intrinsics, indent=2, ensure_ascii=False))

    n_degraded = sum(1 for *_, d in hole_stats if d)
    print(f"\n{len(hole_stats)} face renders, {n_degraded} flagged degraded (>20% holes after fill)")
    print(f"Bundle written to {BUNDLE}")


def _emit(pid, face, uuid_nohyphen, cam_pos, R, scene, poses, hole_stats):
    src_jpg = SRC / "sim/skybox" / f"{uuid_nohyphen}_{face}.jpg"
    tag = f"pt{pid}_{face}"
    Image.open(src_jpg).convert("RGB").save(BUNDLE / "rgb" / f"{tag}.jpg", quality=92)

    depth, hole_before, hole_after = render_face_depth(scene, cam_pos, R, FACE_SIZE, FOV_DEG)
    np.save(BUNDLE / "depth" / f"{tag}.npy", depth)

    quat_xyzw = Rotation.from_matrix(R).as_quat()
    poses[tag] = {"position": cam_pos.tolist(), "quat_xyzw": quat_xyzw.tolist(),
                  "frame": f"pt{pid}", "face": face}
    degraded = hole_after > 0.20
    hole_stats.append((tag, hole_before, hole_after, degraded))
    print(f"{tag}: hole_rate before={hole_before:.1%} after_fill={hole_after:.1%}"
          f"{'  [DEGRADED >20%]' if degraded else ''}")


if __name__ == "__main__":
    main()
