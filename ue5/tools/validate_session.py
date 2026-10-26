"""Check one VisionRecorder session before it is handed to a SLAM system.

Usage:
    python ue5/tools/validate_session.py <session-dir> [--stride 10]

The point is to fail loudly on data that would make SLAM results meaningless:
missing frames, duplicate or non-monotonic timestamps, bad intrinsics, jumps in
the ground-truth pose, or blurry images.
"""

import argparse
import json
import math
import statistics
import sys
from pathlib import Path


def load_jsonl(path):
    rows = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def quat_angle_deg(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--stride", type=int, default=10,
                        help="inspect every Nth image for sharpness and size")
    parser.add_argument("--jump-meters", type=float, default=2.0)
    parser.add_argument("--jump-degrees", type=float, default=60.0)
    args = parser.parse_args()

    session = Path(args.session)
    problems = []
    warnings = []

    meta_path = session / "meta.json"
    poses_path = session / "poses.jsonl"
    frames_dir = session / "frames"
    for required in (meta_path, poses_path, frames_dir):
        if not required.exists():
            print(f"FAIL missing {required}")
            return 1

    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    poses = load_jsonl(poses_path)
    frames = sorted(frames_dir.glob("frame_*.png"))

    width = int(meta["width"])
    height = int(meta["height"])
    hz = float(meta["target_hz"])

    if len(frames) != len(poses):
        problems.append(f"frames={len(frames)} but poses={len(poses)}")

    indices = [int(p["i"]) for p in poses]
    if indices != list(range(len(indices))):
        problems.append("pose indices are not 0..N-1 in order")

    for frame, pose in zip(frames, poses):
        expected = f"frame_{int(pose['i']):06d}.png"
        if frame.name != expected:
            problems.append(f"frame order mismatch: {frame.name} vs pose {pose['i']}")
            break

    # Timestamps must be strictly increasing; intervals should sit near 1/hz.
    intervals = [(b["t_wall"] - a["t_wall"]) * 1000.0 for a, b in zip(poses, poses[1:])]
    if any(dt <= 0 for dt in intervals):
        problems.append("non-monotonic wall timestamps")
    if intervals:
        target_ms = 1000.0 / hz
        outside = [dt for dt in intervals if abs(dt - target_ms) > 0.2 * target_ms]
        frac = len(outside) / len(intervals)
        stats = {
            "min": round(min(intervals), 2),
            "median": round(statistics.median(intervals), 2),
            "max": round(max(intervals), 2),
        }
        if frac > 0.05:
            warnings.append(f"{frac:.1%} of intervals differ from {target_ms:.0f} ms by more than 20%")
    else:
        stats = {}

    # Ground-truth pose must be continuous; a jump means a teleport or a bug.
    jumps = 0
    for a, b in zip(poses, poses[1:]):
        d = math.dist(a["loc"], b["loc"]) / 100.0  # cm -> m
        ang = quat_angle_deg(a["quat"], b["quat"])
        if d > args.jump_meters or ang > args.jump_degrees:
            jumps += 1
    if jumps:
        problems.append(f"{jumps} pose jumps above {args.jump_meters} m or {args.jump_degrees} deg")

    # Intrinsics recorded per frame must agree with field of view and resolution.
    worst_fx_error = 0.0
    for pose in poses[:50]:
        fx_expected = (pose["w"] * 0.5) / math.tan(math.radians(pose["fov"]) * 0.5)
        worst_fx_error = max(worst_fx_error, abs(pose["fx"] - fx_expected))
        if abs(pose["cx"] - pose["w"] * 0.5) > 0.51 or abs(pose["cy"] - pose["h"] * 0.5) > 0.51:
            problems.append("principal point is not at the image centre")
            break
    if worst_fx_error > 1e-3:
        problems.append(f"fx disagrees with fov by {worst_fx_error:.4f} px")

    # Image sanity: size, brightness, sharpness.
    try:
        import cv2
        import numpy as np
    except ImportError:
        warnings.append("opencv not installed; image checks skipped")
        cv2 = None

    if cv2 is not None and frames:
        variances = []
        for frame in frames[:: max(1, args.stride)]:
            image = cv2.imread(str(frame), cv2.IMREAD_COLOR)
            if image is None:
                problems.append(f"cannot read {frame.name}")
                break
            if image.shape[1] != width or image.shape[0] != height:
                problems.append(f"{frame.name} is {image.shape[1]}x{image.shape[0]}, expected {width}x{height}")
                break
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if float(gray.mean()) < 5.0:
                problems.append(f"{frame.name} is nearly black")
            variances.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
        if variances:
            stats["laplacian_median"] = round(statistics.median(variances), 1)
            if statistics.median(variances) < 30.0:
                warnings.append("median Laplacian variance below 30: images may be blurry or textureless")

    report = {
        "session": session.name,
        "map": meta.get("map"),
        "frames": len(frames),
        "poses": len(poses),
        "resolution": f"{width}x{height}",
        "interval_ms": stats,
        "duration_s": round(sum(intervals) / 1000.0, 1) if intervals else 0.0,
        "problems": problems,
        "warnings": warnings,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
