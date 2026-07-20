"""
Phase 2 automated acceptance checks (CLAUDE.md §4).

Usage: python3 p2_checks.py <ply_path> <trajectory_csv>
No open3d/plyfile available in this environment -- binary PLY (x,y,z,intensity
float32, as written by hera-desktop's export_map_pcd.py) is parsed manually.
"""
import csv
import sys

import numpy as np


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
    return data  # columns: x, y, z, intensity


def load_traj(path):
    ts, xyz = [], []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            ts.append(float(row[0]))
            xyz.append([float(row[1]), float(row[2]), float(row[3])])
    return np.array(ts), np.array(xyz)


def ransac_plane(points, n_iter=1000, thresh=0.03, rng=None):
    rng = rng or np.random.default_rng(0)
    best_inliers = 0
    best_plane = None
    n = len(points)
    if n < 3:
        return None, 0
    for _ in range(n_iter):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]
        v1, v2 = p1 - p0, p2 - p0
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -np.dot(normal, p0)
        dist = np.abs(points @ normal + d)
        inliers = np.sum(dist < thresh)
        if inliers > best_inliers:
            best_inliers = inliers
            best_plane = (normal, d, dist < thresh)
    return best_plane, best_inliers


def main():
    ply_path, traj_path = sys.argv[1], sys.argv[2]
    pts = load_binary_ply_xyzi(ply_path)
    xyz = pts[:, :3]
    ts, traj_xyz = load_traj(traj_path)

    results = []

    def record(name, ok, detail):
        results.append((name, ok))
        print(f"[{'OK' if ok else 'FAIL'}] {name}: {detail}")

    # --- Check 1: Z-axis orientation (floor/ceiling histogram) ---
    z = xyz[:, 2]
    hist, edges = np.histogram(z, bins=200)
    peak_idx = np.argmax(hist)
    floor_z = (edges[peak_idx] + edges[peak_idx + 1]) / 2
    # look for a second peak (ceiling) among bins outside +-0.5m of the floor peak
    mask_far = np.abs((edges[:-1] + edges[1:]) / 2 - floor_z) > 0.5
    ceiling_z = None
    if np.any(mask_far):
        far_hist = hist.copy()
        far_hist[~mask_far] = 0
        ceil_idx = np.argmax(far_hist)
        if far_hist[ceil_idx] > 0:
            ceiling_z = (edges[ceil_idx] + edges[ceil_idx + 1]) / 2
    ok1 = (-0.3 <= floor_z <= 0.3) and (ceiling_z is not None and 2.2 <= ceiling_z <= 3.2)
    record("1 Z轴朝向", ok1,
           f"地面峰 z={floor_z:.3f}m (要求[-0.3,0.3]), 天花板峰 z={ceiling_z if ceiling_z is None else round(ceiling_z,3)}m (要求[2.2,3.2])")

    # --- Check 2: scale sanity ---
    n_pts = len(xyz)
    xy_extent_x = xyz[:, 0].max() - xyz[:, 0].min()
    xy_extent_y = xyz[:, 1].max() - xyz[:, 1].min()
    ok2 = (n_pts > 1e5) and (xy_extent_x < 25) and (xy_extent_y < 25)
    record("2 规模合理", ok2,
           f"总点数={n_pts} (要求>1e5), XY 包围盒 = {xy_extent_x:.2f}m x {xy_extent_y:.2f}m (每边要求<25m)")

    # --- Check 3: trajectory completeness ---
    traj_duration = ts[-1] - ts[0]
    dt = np.diff(ts)
    max_gap = dt.max() if len(dt) else 0
    ok3 = max_gap <= 0.5
    record("3 轨迹完整", ok3,
           f"轨迹时长={traj_duration:.2f}s, 最大位姿空洞={max_gap:.3f}s (要求<=0.5s), n_poses={len(ts)}")

    # --- Check 4: loop closure drift (first vs last position) ---
    drift = np.linalg.norm(traj_xyz[-1] - traj_xyz[0])
    ok4 = drift < 0.15
    record("4 回环漂移", ok4,
           f"首尾位置差={drift:.4f}m (要求<0.15m); 首={traj_xyz[0]}, 末={traj_xyz[-1]}")

    # --- Check 5: wall quality (RANSAC plane fit on largest vertical-ish cluster) ---
    # crude wall candidate: points with |z - mean| within human-height band, excluding floor/ceiling bins
    z_mid_mask = (z > floor_z + 0.4) & (z < (ceiling_z - 0.4 if ceiling_z else floor_z + 2.0))
    wall_candidates = xyz[z_mid_mask]
    plane, n_inliers = ransac_plane(wall_candidates, n_iter=500, thresh=0.03)
    if plane is not None and n_inliers > 100:
        normal, d, inlier_mask = plane
        inlier_pts = wall_candidates[inlier_mask]
        dist = np.abs(inlier_pts @ normal + d)
        rms = float(np.sqrt(np.mean(dist ** 2)))
        ok5 = rms < 0.03
        record("5 墙体质量", ok5,
               f"最大平面 RANSAC 内点数={n_inliers}/{len(wall_candidates)}, RMS={rms:.4f}m (要求<0.03m)")
    else:
        record("5 墙体质量", False, "RANSAC 未找到足够内点的平面")

    all_ok = all(r[1] for r in results)
    print(f"\n=== Phase 2 overall: {'PASS' if all_ok else 'FAIL'} ===")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
