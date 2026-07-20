"""
Phase 3: bind each extracted ERP frame to a world pose via spherical/linear
interpolation on the GLIM trajectory (position: linear, attitude: slerp).

Usage: python3 p3_bind_pose.py
"""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parent.parent
TRAJ_CSV = ROOT / "work/phase2/trajectory.csv"
MANIFEST = ROOT / "work/phase3/frames_manifest.json"
OUT = ROOT / "work/phase3/frames_with_pose.json"

# Camera extrinsic relative to LiDAR -- see work/phase3/extrinsic.json for the
# full derivation/confirmation log. User-confirmed 2026-07-14: yaw=+90deg,
# ONLY valid for panoramas stitched with flowstate=false (see extrinsic.json
# "requires" -- flowstate=true bakes a per-frame gyro reorientation into the
# panorama that no fixed extrinsic can compensate for).
T_LIDAR_CAMERA_TRANS = np.array([0.0, 0.0, 0.16])
T_LIDAR_CAMERA_ROT = Rotation.from_euler("xyz", [0.0, 0.0, 90.0], degrees=True)


def load_traj():
    ts, xyz, quat = [], [], []
    with open(TRAJ_CSV) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            ts.append(float(row[0]))
            xyz.append([float(row[1]), float(row[2]), float(row[3])])
            quat.append([float(row[4]), float(row[5]), float(row[6]), float(row[7])])  # xyzw
    return np.array(ts), np.array(xyz), np.array(quat)


def interpolate_pose(ts, xyz, quat, t_query):
    if t_query <= ts[0] or t_query >= ts[-1]:
        raise ValueError(f"t_query={t_query} outside trajectory range [{ts[0]}, {ts[-1]}]")
    i = np.searchsorted(ts, t_query) - 1
    i = max(0, min(i, len(ts) - 2))
    t0, t1 = ts[i], ts[i + 1]
    alpha = (t_query - t0) / (t1 - t0)
    pos = xyz[i] * (1 - alpha) + xyz[i + 1] * alpha
    rots = Rotation.from_quat([quat[i], quat[i + 1]])
    slerp = Slerp([t0, t1], rots)
    rot = slerp([t_query])[0]
    return pos, rot


def main():
    ts, xyz, quat = load_traj()
    manifest = json.loads(MANIFEST.read_text())

    out = []
    for frame in manifest:
        t_query = frame["abs_host_ns"] / 1e9
        try:
            lidar_pos, lidar_rot = interpolate_pose(ts, xyz, quat, t_query)
        except ValueError as e:
            print(f"{frame['name']}: SKIP ({e})")
            continue

        cam_rot = lidar_rot * T_LIDAR_CAMERA_ROT
        cam_pos = lidar_pos + lidar_rot.apply(T_LIDAR_CAMERA_TRANS)

        entry = dict(frame)
        entry["lidar_pos"] = lidar_pos.tolist()
        entry["lidar_quat_xyzw"] = lidar_rot.as_quat().tolist()
        entry["camera_pos"] = cam_pos.tolist()
        entry["camera_quat_xyzw"] = cam_rot.as_quat().tolist()
        out.append(entry)
        print(f"{frame['name']}: t={t_query:.3f} lidar_pos={lidar_pos} cam_pos={cam_pos}")

    OUT.write_text(json.dumps(out, indent=2, ensure_ascii=False))
    print(f"\nWrote {OUT} ({len(out)} frames)")


if __name__ == "__main__":
    main()
