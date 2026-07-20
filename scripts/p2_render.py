"""
Phase 2 human-confirmation renders (CLAUDE.md §4 人工确认点 P2):
top-down projection, side view, trajectory-overlaid top-down.

Usage: python3 p2_render.py <ply_path> <trajectory_csv> <out_dir>
"""
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


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


def load_traj(path):
    xyz = []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            xyz.append([float(row[1]), float(row[2]), float(row[3])])
    return np.array(xyz)


def main():
    ply_path, traj_path, out_dir = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    pts = load_binary_ply_xyzi(ply_path)
    xyz = pts[:, :3]
    traj = load_traj(traj_path)

    # 1. Top-down projection (XY), colored by Z
    fig, ax = plt.subplots(figsize=(9, 8))
    sc = ax.scatter(xyz[:, 0], xyz[:, 1], c=xyz[:, 2], s=0.5, cmap="viridis")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down projection (colored by Z)")
    ax.set_aspect("equal")
    plt.colorbar(sc, ax=ax, label="z (m)")
    fig.tight_layout()
    fig.savefig(out_dir / "topdown.png", dpi=150)
    plt.close(fig)

    # 2. Side view (XZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.scatter(xyz[:, 0], xyz[:, 2], s=0.3, alpha=0.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("Side view (X-Z), floor/ceiling should be two horizontal bands")
    fig.tight_layout()
    fig.savefig(out_dir / "sideview.png", dpi=150)
    plt.close(fig)

    # 3. Trajectory overlay on top-down
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.scatter(xyz[:, 0], xyz[:, 1], s=0.3, c="lightgray", alpha=0.6)
    ax.plot(traj[:, 0], traj[:, 1], c="red", lw=1.2, label="trajectory")
    ax.scatter([traj[0, 0]], [traj[0, 1]], c="green", s=80, marker="o", label="start", zorder=5)
    ax.scatter([traj[-1, 0]], [traj[-1, 1]], c="blue", s=80, marker="x", label="end", zorder=5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Trajectory overlaid on top-down point cloud")
    ax.set_aspect("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "trajectory_overlay.png", dpi=150)
    plt.close(fig)

    print(f"Wrote topdown.png, sideview.png, trajectory_overlay.png -> {out_dir}")


if __name__ == "__main__":
    main()
