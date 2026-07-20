"""
Phase 3: auto-search the LiDAR->camera extrinsic ROTATION (translation held
fixed at the hand-measured 16cm-up offset) by maximizing alignment between
the point cloud's projected depth-discontinuity edges and the panorama
image's visual edges.

Why: work/phase3/extrinsic.json currently hardcodes rotation="identity" --
that was never actually calibrated, just accepted early on when the sparse
Phase-2 point cloud was too noisy to tell good alignment from bad. Now that
the dense point cloud (deliverables/map_20260712212422_fred_calib.ply) is
much cleaner, the overlay shows a real misalignment, suspected to include a
~180 degree yaw (Z-axis) error -- plausible if the Insta360's "front" lens
direction doesn't match the LiDAR body frame's declared +X.

This script does NOT decide the final extrinsic. Per CLAUDE.md's P3 human
confirmation gate, it only proposes a short ranked list of candidates (with
rendered overlays) for visual confirmation / fine adjustment. It searches
rotation only, coordinate-descent style:
  1. Coarse grid: roll/pitch in {0, 180} deg (catches upside-down mounting),
     yaw in 10 degree steps over the full circle.
  2. Refine the best coarse candidate: yaw +-10 deg @ 1 deg steps, then
     roll +-10 deg @ 2 deg steps, then pitch +-10 deg @ 2 deg steps.

Score = mean(lidar_depth_edge_magnitude * image_edge_magnitude) over a
downsampled equirect grid, i.e. how well projected geometric edges (wall
corners, door frames -- real depth discontinuities) line up with visual
edges in the panorama. Both maps are non-negative, so this is a plain
cross-correlation: higher is better.

Usage: python3 scripts/p3_calibrate_extrinsic.py
Outputs:
  work/phase3/calib_search/scores.json       -- full ranked candidate list
  work/phase3/calib_search/report.md         -- human-readable summary
  work/phase3/calib_search/overlay_<cand>_<frame>.png  -- for top candidates
"""
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
PLY = ROOT / "deliverables/map_20260712212422_fred_calib.ply"
TRAJ_CSV = ROOT / "work/phase2/trajectory.csv"
MANIFEST = ROOT / "work/phase3/frames_manifest.json"
OUT_DIR = ROOT / "work/phase3/calib_search"
OUT_DIR.mkdir(parents=True, exist_ok=True)

T_TRANS = np.array([0.0, 0.0, 0.16])  # held fixed; only rotation is searched
GRID_DOWNSAMPLE = 8  # 1920x960 -> 240x120 scoring grid
RENDER_TOP_N = 3


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


def load_traj():
    ts, xyz, quat = [], [], []
    import csv
    with open(TRAJ_CSV) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            ts.append(float(row[0]))
            xyz.append([float(row[1]), float(row[2]), float(row[3])])
            quat.append([float(row[4]), float(row[5]), float(row[6]), float(row[7])])
    return np.array(ts), np.array(xyz), np.array(quat)


def interpolate_pose(ts, xyz, quat, t_query):
    from scipy.spatial.transform import Slerp
    if t_query <= ts[0] or t_query >= ts[-1]:
        raise ValueError(f"t_query={t_query} outside trajectory range")
    i = np.searchsorted(ts, t_query) - 1
    i = max(0, min(i, len(ts) - 2))
    t0, t1 = ts[i], ts[i + 1]
    alpha = (t_query - t0) / (t1 - t0)
    pos = xyz[i] * (1 - alpha) + xyz[i + 1] * alpha
    rots = Rotation.from_quat([quat[i], quat[i + 1]])
    rot = Slerp([t0, t1], rots)([t_query])[0]
    return pos, rot


def project_equirect(pts_cam, width, height):
    x, y, z = pts_cam[:, 0], pts_cam[:, 1], pts_cam[:, 2]
    r = np.sqrt(x * x + y * y + z * z)
    lon = np.arctan2(y, x)
    lat = np.arcsin(np.clip(z / np.maximum(r, 1e-9), -1, 1))
    u = (0.5 - lon / (2 * np.pi)) * width
    v = (0.5 - lat / np.pi) * height
    return u, v, r


def lidar_edge_grid(pts_lidarframe, r_cand_matrix, gw, gh):
    """Project points (already in lidar-body frame, camera-pos-relative) through
    a candidate rotation matrix and rasterize a min-depth grid -> edge magnitude."""
    pts_cam = pts_lidarframe @ r_cand_matrix
    u, v, r = project_equirect(pts_cam, gw, gh)
    mask = (r > 0.05) & (r < 15) & (u >= 0) & (u < gw) & (v >= 0) & (v < gh)
    u, v, r = u[mask], v[mask], r[mask]
    ui = u.astype(np.int64)
    vi = v.astype(np.int64)
    flat = vi * gw + ui

    depth = np.full(gw * gh, np.inf, dtype=np.float64)
    np.minimum.at(depth, flat, r)
    valid = np.isfinite(depth)
    depth_img = depth.reshape(gh, gw)
    valid_img = valid.reshape(gh, gw)

    gx = np.zeros((gh, gw))
    gy = np.zeros((gh, gw))
    vx = valid_img[:, 1:] & valid_img[:, :-1]
    gx[:, 1:][vx] = depth_img[:, 1:][vx] - depth_img[:, :-1][vx]
    vy = valid_img[1:, :] & valid_img[:-1, :]
    gy[1:, :][vy] = depth_img[1:, :][vy] - depth_img[:-1, :][vy]

    edge = np.clip(np.sqrt(gx * gx + gy * gy), 0, 3.0) / 3.0
    return edge, valid_img


ROW_BAND = (0.12, 0.78)  # exclude flat ceiling/light-fixture top band and
                          # photographer-body/tripod bottom band from scoring


def image_edge_grid(img, gw, gh):
    small = img.convert("L").resize((gw, gh), Image.BILINEAR)
    arr = np.asarray(small, dtype=np.float64)
    gx = np.zeros_like(arr)
    gy = np.zeros_like(arr)
    gx[:, 1:-1] = arr[:, 2:] - arr[:, :-2]
    gy[1:-1, :] = arr[2:, :] - arr[:-2, :]
    mag = np.sqrt(gx * gx + gy * gy)
    mag /= max(mag.max(), 1e-9)

    # Zero out near-saturated pixels (light fixtures, window blowouts, direct
    # reflections) -- these dominate raw Sobel magnitude without being real
    # architectural edges, and were confirmed (visually) to be pulling the
    # search toward bogus rotations.
    saturated = arr > 235
    mag[saturated] = 0.0

    row_mask = np.zeros(gh, dtype=bool)
    row_mask[int(gh * ROW_BAND[0]):int(gh * ROW_BAND[1])] = True
    mag[~row_mask, :] = 0.0
    return mag


def score_candidate(r_cand, frames_data, gw, gh):
    r_mat = r_cand.as_matrix()
    row_valid = np.zeros(gh, dtype=bool)
    row_valid[int(gh * ROW_BAND[0]):int(gh * ROW_BAND[1])] = True
    total, count = 0.0, 0
    for fd in frames_data:
        edge_l, valid = lidar_edge_grid(fd["pts_lidarframe"], r_mat, gw, gh)
        valid = valid & row_valid[:, None]
        prod = edge_l * fd["img_edge"]
        total += prod[valid].sum()
        count += valid.sum()
    return total / max(count, 1)


def render_overlay(name, r_cand, frames_data, tag):
    for fd in frames_data:
        pts_cam = fd["pts_lidarframe"] @ r_cand.as_matrix()
        width, height = fd["img"].size
        u, v, r = project_equirect(pts_cam, width, height)
        mask = (r > 0.05) & (r < 15) & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v, r = u[mask], v[mask], r[mask]

        arr = np.array(fd["img"].convert("RGB"))
        import matplotlib.cm as cm
        r_norm = np.clip(r / 8.0, 0, 1)
        colors = (cm.get_cmap("turbo")(1 - r_norm)[:, :3] * 255).astype(np.uint8)
        ui, vi = u.astype(np.int32), v.astype(np.int32)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                uu = np.clip(ui + dx, 0, width - 1)
                vv = np.clip(vi + dy, 0, height - 1)
                arr[vv, uu] = colors

        out_path = OUT_DIR / f"overlay_{tag}_{fd['name']}.png"
        Image.fromarray(arr).save(out_path)


def main():
    print("Loading point cloud...")
    pts = load_binary_ply_xyzi(PLY)
    xyz_world = pts[:, :3].astype(np.float64)

    ts, traj_xyz, traj_quat = load_traj()
    manifest = json.loads(MANIFEST.read_text())

    frames_data = []
    for frame in manifest:
        t_query = frame["abs_host_ns"] / 1e9
        try:
            lidar_pos, lidar_rot = interpolate_pose(ts, traj_xyz, traj_quat, t_query)
        except ValueError as e:
            print(f"{frame['name']}: SKIP ({e})")
            continue
        cam_pos = lidar_pos + lidar_rot.apply(T_TRANS)
        # pts_lidarframe is independent of the rotation candidate -- precompute once
        pts_lidarframe = lidar_rot.inv().apply(xyz_world - cam_pos)

        img = Image.open(ROOT / frame["jpg"])
        gw, gh = img.width // GRID_DOWNSAMPLE, img.height // GRID_DOWNSAMPLE
        img_edge = image_edge_grid(img, gw, gh)

        frames_data.append(dict(
            name=frame["name"], img=img, pts_lidarframe=pts_lidarframe, img_edge=img_edge,
        ))
        print(f"{frame['name']}: prepared ({len(xyz_world)} pts, grid {gw}x{gh})")

    if not frames_data:
        print("No frames with valid poses -- aborting.")
        return

    gw, gh = frames_data[0]["img_edge"].shape[1], frames_data[0]["img_edge"].shape[0]

    # ---- stage 1: coarse grid ----
    print("\nStage 1: coarse grid (roll,pitch in {0,180}, yaw step 10deg)...")
    results = []
    for roll in (0, 180):
        for pitch in (0, 180):
            for yaw in range(0, 360, 10):
                r_cand = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
                s = score_candidate(r_cand, frames_data, gw, gh)
                results.append((s, roll, pitch, yaw))
    results.sort(reverse=True)
    print("Top 5 coarse candidates:")
    for s, roll, pitch, yaw in results[:5]:
        print(f"  score={s:.5f}  roll={roll} pitch={pitch} yaw={yaw}")

    best_s, best_roll, best_pitch, best_yaw = results[0]

    # ---- stage 2: coordinate-descent refine ----
    print("\nStage 2: refine yaw (+-10deg @ 1deg)...")
    cand = [(score_candidate(Rotation.from_euler("xyz", [best_roll, best_pitch, y], degrees=True),
                              frames_data, gw, gh), y)
            for y in np.arange(best_yaw - 10, best_yaw + 10.01, 1)]
    cand.sort(reverse=True)
    best_yaw = cand[0][1]
    print(f"  refined yaw={best_yaw} score={cand[0][0]:.5f}")

    print("Refine roll (+-10deg @ 2deg)...")
    cand = [(score_candidate(Rotation.from_euler("xyz", [rr, best_pitch, best_yaw], degrees=True),
                              frames_data, gw, gh), rr)
            for rr in np.arange(best_roll - 10, best_roll + 10.01, 2)]
    cand.sort(reverse=True)
    best_roll = cand[0][1]
    print(f"  refined roll={best_roll} score={cand[0][0]:.5f}")

    print("Refine pitch (+-10deg @ 2deg)...")
    cand = [(score_candidate(Rotation.from_euler("xyz", [best_roll, pp, best_yaw], degrees=True),
                              frames_data, gw, gh), pp)
            for pp in np.arange(best_pitch - 10, best_pitch + 10.01, 2)]
    cand.sort(reverse=True)
    best_pitch = cand[0][1]
    final_score = cand[0][0]
    print(f"  refined pitch={best_pitch} score={final_score:.5f}")

    final_rot = Rotation.from_euler("xyz", [best_roll, best_pitch, best_yaw], degrees=True)
    identity_score = score_candidate(Rotation.identity(), frames_data, gw, gh)

    print(f"\nFinal candidate: roll={best_roll:.1f} pitch={best_pitch:.1f} yaw={best_yaw:.1f}"
          f"  score={final_score:.5f}  (identity score={identity_score:.5f})")

    # ---- render overlays: identity (baseline) + top-N coarse + final refined ----
    print(f"\nRendering overlays for identity, top-{RENDER_TOP_N} coarse candidates, and final refined...")
    render_overlay("identity", Rotation.identity(), frames_data, "identity")
    for i, (s, roll, pitch, yaw) in enumerate(results[:RENDER_TOP_N]):
        tag = f"coarse{i+1}_r{roll}_p{pitch}_y{yaw}"
        render_overlay(tag, Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True), frames_data, tag)
    render_overlay("final", final_rot, frames_data, f"final_r{best_roll:.1f}_p{best_pitch:.1f}_y{best_yaw:.1f}")

    # ---- write outputs ----
    scores_out = {
        "identity_score": identity_score,
        "coarse_top10": [{"score": s, "roll": r, "pitch": p, "yaw": y} for s, r, p, y in results[:10]],
        "final_candidate": {"roll": best_roll, "pitch": best_pitch, "yaw": best_yaw, "score": final_score},
        "translation_lidar_to_camera_m": T_TRANS.tolist(),
        "note": "Rotation only; translation held fixed at the hand-measured value. "
                "Requires human visual confirmation (CLAUDE.md P3 gate) before writing to "
                "work/phase3/extrinsic.json -- see overlay_final_*.png and overlay_identity_*.png "
                "for before/after comparison.",
    }
    (OUT_DIR / "scores.json").write_text(json.dumps(scores_out, indent=2))

    report = [
        "# Phase 3 extrinsic rotation auto-search report\n",
        f"Identity (previously accepted) score: **{identity_score:.5f}**\n",
        f"Best found: roll={best_roll:.1f} pitch={best_pitch:.1f} yaw={best_yaw:.1f}"
        f" -> score **{final_score:.5f}** ({final_score/max(identity_score,1e-9):.1f}x)\n",
        "\n## Coarse top 10\n",
        "| score | roll | pitch | yaw |",
        "|---|---|---|---|",
    ]
    for s, roll, pitch, yaw in results[:10]:
        report.append(f"| {s:.5f} | {roll} | {pitch} | {yaw} |")
    report.append("\nOverlay images for visual confirmation: `overlay_identity_*.png` (baseline) "
                   "vs `overlay_final_*.png` (best found). Coarse runner-ups: `overlay_coarse{1,2,3}_*.png`.\n"
                   "This is a proposal only -- not yet written to work/phase3/extrinsic.json.")
    (OUT_DIR / "report.md").write_text("\n".join(report))
    print(f"\nWrote {OUT_DIR / 'scores.json'} and {OUT_DIR / 'report.md'}")


if __name__ == "__main__":
    main()
