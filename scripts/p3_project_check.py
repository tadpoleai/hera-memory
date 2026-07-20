"""
Phase 3: project the GLIM world-frame point cloud (depth-colored) onto each
pose-bound ERP frame, for visual extrinsic-calibration confirmation (P3).

Camera-frame axis convention assumed (subject to correction via the P3
iteration loop): +X forward, +Y left, +Z up. Equirectangular longitude
measured from +X (forward, image horizontal center), latitude from the
XY-plane (+Z = up = top of image).

Usage: python3 p3_project_check.py
"""
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
PLY = ROOT / "deliverables/map_20260712212422_fred_calib.ply"
FRAMES = ROOT / "work/phase3/frames_with_pose.json"
OUT_DIR = ROOT / "work/phase3"


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


def project_equirect(pts_cam, width, height):
    x, y, z = pts_cam[:, 0], pts_cam[:, 1], pts_cam[:, 2]
    r = np.sqrt(x * x + y * y + z * z)
    lon = np.arctan2(y, x)          # +X fwd (lon=0), +Y left (lon=+90deg)
    lat = np.arcsin(np.clip(z / np.maximum(r, 1e-9), -1, 1))  # +Z up

    u = (0.5 - lon / (2 * np.pi)) * width
    v = (0.5 - lat / np.pi) * height
    return u, v, r


def main():
    pts = load_binary_ply_xyzi(PLY)
    xyz_world = pts[:, :3].astype(np.float64)

    frames = json.loads(FRAMES.read_text())
    for frame in frames:
        jpg_path = ROOT / frame["jpg"]
        img = Image.open(jpg_path).convert("RGB")
        width, height = img.size

        cam_pos = np.array(frame["camera_pos"])
        cam_rot = Rotation.from_quat(frame["camera_quat_xyzw"])

        rel = xyz_world - cam_pos
        pts_cam = cam_rot.inv().apply(rel)  # world -> camera frame

        u, v, r = project_equirect(pts_cam, width, height)

        # keep points reasonably in front / within sane range for a room-scale
        # capture (avoid a handful of far outliers dominating the color scale)
        mask = (r > 0.05) & (r < 15) & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v, r = u[mask], v[mask], r[mask]

        arr = np.array(img)
        # color by depth: near=red, far=blue (simple HSV-ish ramp via matplotlib colormap)
        import matplotlib.cm as cm
        r_norm = np.clip(r / 8.0, 0, 1)
        colors = (cm.get_cmap("turbo")(1 - r_norm)[:, :3] * 255).astype(np.uint8)

        ui = u.astype(np.int32)
        vi = v.astype(np.int32)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                uu = np.clip(ui + dx, 0, width - 1)
                vv = np.clip(vi + dy, 0, height - 1)
                arr[vv, uu] = colors

        out_path = OUT_DIR / f"overlay_{frame['name']}.png"
        Image.fromarray(arr).save(out_path)
        print(f"{frame['name']}: {mask.sum()} points projected -> {out_path.name}")


if __name__ == "__main__":
    main()
