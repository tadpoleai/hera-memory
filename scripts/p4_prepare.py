"""
Phase 4 step 1 (local preprocessing, CLAUDE.md Sec.6):
For each pose-bound ERP frame, slice into cubemap perspective views (skip the
down face -- avoids the operator's body/rig visible in the ERP frames), and
render a matching depth map from the GLIM point cloud via z-buffer point
splatting + 3x3 hole fill. Package rgb/depth/poses/intrinsics for a GPU
instance-extraction step (ConceptGraphs or similar) -- GPU deployment is a
separate, later step; this script only produces the upload bundle.

Usage: python3 p4_prepare.py
"""
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
PLY = ROOT / "deliverables/map_20260712212422_fred_calib.ply"
FRAMES = ROOT / "work/phase3/frames_with_pose.json"
BUNDLE = ROOT / "work/phase4/upload_bundle"
FACE_SIZE = 640
FOV_DEG = 90.0

# Skip "down" -- avoids the operator's body/selfie-stick rig visible in the
# lower hemisphere of every ERP frame (per CLAUDE.md Sec.6 step 1).
FACES = {
    "front": Rotation.identity(),
    "right": Rotation.from_euler("z", -90, degrees=True),
    "back":  Rotation.from_euler("z", 180, degrees=True),
    "left":  Rotation.from_euler("z", 90, degrees=True),
    "up":    Rotation.from_euler("y", -90, degrees=True),
}


def load_binary_ply_xyzi(path):
    with open(path, "rb") as f:
        line = f.readline()
        assert line.strip() == b"ply"
        n_vertex = None
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("element vertex"):
                n_vertex = int(line.split()[-1])
            if line == "end_header":
                break
        data = np.frombuffer(f.read(n_vertex * 16), dtype="<f4").reshape(n_vertex, 4)
    return data


def equirect_sample(img_arr, dirs_main):
    """Sample an equirect RGB image array at given main-camera-frame direction
    vectors (N,3). Returns (N,3) uint8 colors."""
    h, w = img_arr.shape[:2]
    x, y, z = dirs_main[:, 0], dirs_main[:, 1], dirs_main[:, 2]
    r = np.sqrt(x * x + y * y + z * z)
    lon = np.arctan2(y, x)
    lat = np.arcsin(np.clip(z / np.maximum(r, 1e-9), -1, 1))
    u = ((0.5 - lon / (2 * np.pi)) * w).astype(np.int32) % w
    v = np.clip(((0.5 - lat / np.pi) * h).astype(np.int32), 0, h - 1)
    return img_arr[v, u]


def render_face_rgb(erp_arr, face_rot: Rotation, size: int, fov_deg: float):
    f = (size / 2) / np.tan(np.radians(fov_deg) / 2)
    cx = cy = size / 2
    px, py = np.meshgrid(np.arange(size), np.arange(size))
    x_cv = (px - cx) / f
    y_cv = (py - cy) / f
    z_cv = np.ones_like(x_cv)
    # CV (x=right,y=down,z=fwd) -> robotics (x=fwd,y=left,z=up)
    dir_local = np.stack([z_cv, -x_cv, -y_cv], axis=-1)
    dir_local = dir_local / np.linalg.norm(dir_local, axis=-1, keepdims=True)
    dir_main = face_rot.apply(dir_local.reshape(-1, 3)).reshape(size, size, 3)
    rgb = equirect_sample(erp_arr, dir_main.reshape(-1, 3)).reshape(size, size, 3)
    return rgb.astype(np.uint8), f, cx, cy


def render_face_depth(points_cam_frame_all, face_rot: Rotation, size: int, fov_deg: float):
    """z-buffer point splat into this face's pinhole camera + 3x3 hole fill.
    points_cam_frame_all: (N,3) points already expressed in the MAIN camera frame."""
    f = (size / 2) / np.tan(np.radians(fov_deg) / 2)
    cx = cy = size / 2

    pts_face = face_rot.inv().apply(points_cam_frame_all)  # main-cam frame -> face-local (robotics)
    x_rob, y_rob, z_rob = pts_face[:, 0], pts_face[:, 1], pts_face[:, 2]
    # robotics (x=fwd,y=left,z=up) -> CV (x=right,y=down,z=fwd)
    z_cv = x_rob
    x_cv = -y_rob
    y_cv = -z_rob

    in_front = z_cv > 0.05
    x_cv, y_cv, z_cv = x_cv[in_front], y_cv[in_front], z_cv[in_front]
    u = (x_cv / z_cv * f + cx).astype(np.int32)
    v = (y_cv / z_cv * f + cy).astype(np.int32)
    in_bounds = (u >= 0) & (u < size) & (v >= 0) & (v < size)
    u, v, z_cv = u[in_bounds], v[in_bounds], z_cv[in_bounds]

    depth = np.full((size, size), np.inf, dtype=np.float32)
    # z-buffer: keep nearest depth per pixel (sort far->near, write near last)
    order = np.argsort(-z_cv)
    depth[v[order], u[order]] = z_cv[order]

    hole_mask = ~np.isfinite(depth)
    hole_rate_before = hole_mask.mean()

    # 3x3 hole fill: repeatedly replace holes with the mean of finite neighbors
    filled = depth.copy()
    for _ in range(2):
        pad = np.pad(filled, 1, mode="edge")
        stack = np.stack([pad[i:i + size, j:j + size]
                           for i in range(3) for j in range(3)], axis=0)
        finite = np.isfinite(stack)
        neighbor_mean = np.where(finite, stack, 0).sum(axis=0) / np.maximum(finite.sum(axis=0), 1)
        still_hole = ~np.isfinite(filled)
        filled = np.where(still_hole & (finite.sum(axis=0) > 0), neighbor_mean, filled)

    hole_rate_after = (~np.isfinite(filled)).mean()
    filled[~np.isfinite(filled)] = 0.0
    return filled.astype(np.float32), hole_rate_before, hole_rate_after


def main():
    pts = load_binary_ply_xyzi(PLY)
    xyz_world = pts[:, :3].astype(np.float64)
    frames = json.loads(FRAMES.read_text())

    (BUNDLE / "rgb").mkdir(parents=True, exist_ok=True)
    (BUNDLE / "depth").mkdir(parents=True, exist_ok=True)

    poses = {}
    intrinsics = {}
    hole_stats = []

    f_val = (FACE_SIZE / 2) / np.tan(np.radians(FOV_DEG) / 2)
    cx_val = cy_val = FACE_SIZE / 2
    intrinsics["_all_faces"] = {"fx": f_val, "fy": f_val, "cx": cx_val, "cy": cy_val,
                                 "width": FACE_SIZE, "height": FACE_SIZE, "fov_deg": FOV_DEG}

    for frame in frames:
        name = frame["name"]
        erp_img = Image.open(ROOT / frame["jpg"]).convert("RGB")
        erp_arr = np.array(erp_img)

        cam_pos = np.array(frame["camera_pos"])
        cam_rot = Rotation.from_quat(frame["camera_quat_xyzw"])
        pts_in_main_cam = cam_rot.inv().apply(xyz_world - cam_pos)

        for face_name, face_rot in FACES.items():
            rgb, f, cx, cy = render_face_rgb(erp_arr, face_rot, FACE_SIZE, FOV_DEG)
            depth, hole_before, hole_after = render_face_depth(
                pts_in_main_cam, face_rot, FACE_SIZE, FOV_DEG)

            tag = f"{name}_{face_name}"
            Image.fromarray(rgb).save(BUNDLE / "rgb" / f"{tag}.jpg", quality=90)
            np.save(BUNDLE / "depth" / f"{tag}.npy", depth)

            face_world_rot = cam_rot * face_rot
            poses[tag] = {
                "position": cam_pos.tolist(),
                "quat_xyzw": face_world_rot.as_quat().tolist(),
                "frame": name,
                "face": face_name,
            }
            degraded = hole_after > 0.20
            hole_stats.append((tag, hole_before, hole_after, degraded))
            print(f"{tag}: hole_rate before={hole_before:.1%} after_fill={hole_after:.1%}"
                  f"{'  [DEGRADED >20%]' if degraded else ''}")

    (BUNDLE / "poses.json").write_text(json.dumps(poses, indent=2, ensure_ascii=False))
    (BUNDLE / "intrinsics.json").write_text(json.dumps(intrinsics, indent=2, ensure_ascii=False))

    n_degraded = sum(1 for *_, d in hole_stats if d)
    print(f"\n{len(hole_stats)} face renders, {n_degraded} flagged degraded (>20% holes after fill)")
    print(f"Bundle written to {BUNDLE}")


if __name__ == "__main__":
    main()
