"""
Phase 2: load GLIM's per-run trajectory into a canonical CSV.

CLAUDE.md originally assumed a cloud-workflow map package with N numbered
submap directories each containing data.txt (T_world_origin), requiring this
script to stitch them into a global trajectory. The local hera-desktop/GLIM
pipeline already writes a single, globally-optimized trajectory directly at
the map root (traj_lidar.txt, TUM format: t x y z qx qy qz qw, loop-closed),
so no stitching is needed -- we just reformat it to CSV.

Usage: python3 p2_load_traj.py <map_dir> <out_trajectory.csv>
"""
import csv
import sys
from pathlib import Path


def main():
    map_dir, out_csv = Path(sys.argv[1]), Path(sys.argv[2])
    traj_path = map_dir / "traj_lidar.txt"

    n = 0
    with open(traj_path) as fin, open(out_csv, "w", newline="") as fout:
        w = csv.writer(fout)
        w.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for line in fin:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            w.writerow(parts[:8])
            n += 1
    print(f"Wrote {n} poses -> {out_csv}")


if __name__ == "__main__":
    main()
