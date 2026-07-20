"""
Phase 3: extract one representative ERP JPEG per confirmed static segment,
at the segment's midpoint (video-relative time).

Video/IMU are on the same host clock (SyncLocalTime=true) but their capture
windows don't start/end at exactly the same instant (Phase1 check #5) --
multi_source_synchronizer isn't available in this environment, so we convert
IMU-relative segment times to video-relative times using the two absolute
start timestamps directly (degraded mode, per CLAUDE.md Sec.5 step 3).

Usage: python3 p3_extract_frames.py
"""
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PANO = ROOT / "work/phase3/pano.mp4"
SESSION = ROOT / "data/raw/20260712212422_fred_calib.session.json"
OUT_DIR = ROOT / "work/phase3/frames"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# From Phase1 p1_checks.py output (IMU-relative seconds) on
# 20260712212422_fred_calib, user-confirmed to correspond, in order, to the
# walk's stop points (room A / room B / room C(longest) / final stop, cut
# short by recording end).
SEGMENTS_IMU_REL = [
    ("stop1", 10.0, 18.7),
    ("stop2", 28.1, 35.4),
    ("stop3", 38.6, 54.3),
    ("stop4", 63.2, 70.2),
]

IMU_ABS_START = 1783862664.509177056  # ts_host[0] from work/phase1/imu.csv
VIDEO_ABS_START = json.loads(SESSION.read_text())["record_start_host_ns"] / 1e9


def main():
    offset = IMU_ABS_START - VIDEO_ABS_START
    print(f"video/IMU start offset: {offset:.3f}s (IMU starts this much after video)")

    manifest = []
    for name, t0, t1 in SEGMENTS_IMU_REL:
        mid_imu_rel = (t0 + t1) / 2
        mid_video_rel = mid_imu_rel + offset
        mid_abs_host_ns = int((IMU_ABS_START + (mid_imu_rel - 0)) * 1e9)
        # (mid_imu_rel is already IMU-relative, so absolute = IMU_ABS_START + mid_imu_rel)
        out_path = OUT_DIR / f"{name}.jpg"
        cmd = ["ffmpeg", "-y", "-ss", f"{mid_video_rel:.3f}", "-i", str(PANO),
               "-frames:v", "1", str(out_path)]
        subprocess.run(cmd, capture_output=True)
        ok = out_path.exists() and out_path.stat().st_size > 0
        print(f"{name}: imu_rel_mid={mid_imu_rel:.2f}s video_rel={mid_video_rel:.2f}s "
              f"abs_host_ns={mid_abs_host_ns} -> {out_path.name} {'OK' if ok else 'FAILED'}")
        manifest.append({
            "name": name,
            "imu_rel_mid_s": mid_imu_rel,
            "video_rel_s": mid_video_rel,
            "abs_host_ns": mid_abs_host_ns,
            "jpg": str(out_path.relative_to(ROOT)),
        })

    (ROOT / "work/phase3/frames_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False))
    print(f"\nWrote work/phase3/frames_manifest.json ({len(manifest)} frames)")


if __name__ == "__main__":
    main()
