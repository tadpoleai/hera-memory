"""
Phase 1: extract Livox IMU samples from a .hera file into CSV.

Replaces the (nonexistent in this environment) `hera-storage-extract-mid360`
binary. The `hera` Python SDK (hera-sdk-python) does not yet expose a
livox_imu_frames() method, so we decode MSG_LIVOX_IMU packets directly here
using the wire format documented in hera-sdk-python/tools/hera_to_ros2bag.py
(_LIVOX_IMU_HDR = "<QQIBBffffffI": ts_device, ts_host, handle, dev_type,
data_type, gx, gy, gz, ax, ay, az, payload_size).

Usage: python3 p1_extract_imu.py <recording.hera> <out_imu.csv> <out_points_sample.csv>
"""
import csv
import struct
import sys

sys.path.insert(0, "/home/fred/Code/hera-sdk-python")

from hera import HeraFile
from hera._format import MSG_LIVOX_IMU, MSG_LIVOX_PACKET

_LIVOX_IMU_HDR = struct.Struct("<QQIBBffffffI")

POINTS_SAMPLE_STRIDE = 50  # keep every Nth livox frame's points to bound file size


def main():
    hera_path, imu_csv, points_csv = sys.argv[1], sys.argv[2], sys.argv[3]

    n_imu = 0
    with open(imu_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp_device_ns", "timestamp_host_ns", "gx", "gy", "gz", "ax", "ay", "az"])
        with HeraFile(hera_path) as f:
            for pkt in f.packets(msg_type=MSG_LIVOX_IMU):
                b = pkt.plugin_bytes
                if len(b) < _LIVOX_IMU_HDR.size:
                    continue
                ts_dev, ts_host, handle, dev_type, data_type, gx, gy, gz, ax, ay, az, psz = \
                    _LIVOX_IMU_HDR.unpack_from(b)
                w.writerow([ts_dev, ts_host, gx, gy, gz, ax, ay, az])
                n_imu += 1
    print(f"IMU samples written: {n_imu} -> {imu_csv}")

    n_pts_frames = 0
    n_pts_total = 0
    with open(points_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp_host_ns", "x", "y", "z", "reflectivity"])
        with HeraFile(hera_path) as f:
            frame_idx = 0
            for pkt in f.packets(msg_type=MSG_LIVOX_PACKET):
                frame_idx += 1
                if frame_idx % POINTS_SAMPLE_STRIDE != 0:
                    continue
                from hera.decoders.livox import decode_packet
                frame = decode_packet(pkt)
                if frame is None:
                    continue
                n_pts_frames += 1
                for row in frame.points:
                    w.writerow([frame.timestamp_host_ns, row[0], row[1], row[2], row[3]])
                    n_pts_total += 1
    print(f"Point-cloud sample: {n_pts_frames} frames, {n_pts_total} points -> {points_csv}")


if __name__ == "__main__":
    main()
