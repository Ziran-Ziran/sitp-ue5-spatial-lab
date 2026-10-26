"""Check that the recorded depth map really is metric depth from the camera.

    python ue5/tools/check_depth.py output/ue5-datasets/line-03

Method: take the known 3D positions of the synthetic labels, project them with the
recorded pose and intrinsics, read the depth map at that pixel, and compare. If the
depth convention (axis distance in centimetres) is right, the ratio is 1.

Prints the ratio against both candidate conventions, because "scene depth" can mean
distance along the optical axis or distance along the ray, and guessing which one
the engine wrote is exactly the kind of assumption that silently breaks a dataset.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def quat_to_matrix(quat):
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--stride", type=int, default=20, help="check every Nth frame")
    parser.add_argument("--flip", action="store_true", help="read the depth map bottom-up")
    args = parser.parse_args()

    session = Path(args.session)
    meta = json.loads((session / "meta.json").read_text(encoding="utf-8"))
    if not meta.get("depth", {}).get("enabled"):
        print("this session has no depth")
        return 1

    landmarks = np.array(
        json.loads((session / "landmarks.json").read_text(encoding="utf-8"))["points"], dtype=float)
    poses = [json.loads(line) for line in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]

    axis_ratios, ray_ratios = [], []
    abs_errors_cm = []
    checked = 0

    for pose in poses[:: max(1, args.stride)]:
        depth_path = session / "depth" / f"frame_{pose['i']:06d}.png"
        if not depth_path.exists():
            continue
        raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if raw is None or raw.dtype != np.uint16:
            print(f"unexpected depth encoding: {depth_path.name} dtype={None if raw is None else raw.dtype}")
            return 2
        depth_cm = raw.astype(np.float32) / 10.0
        if args.flip:
            depth_cm = depth_cm[::-1, :]

        rotation = quat_to_matrix(pose["quat"])
        centre = np.array(pose["loc"], dtype=float)

        for point in landmarks:
            delta = point - centre
            camera = rotation.T @ delta
            if camera[0] <= 1.0:
                continue
            u = pose["cx"] + pose["fx"] * camera[1] / camera[0]
            v = pose["cy"] - pose["fy"] * camera[2] / camera[0]
            ui, vi = int(round(u)), int(round(v))
            if not (2 <= ui < pose["w"] - 2 and 2 <= vi < pose["h"] - 2):
                continue

            # 3x3 median: a single pixel can land on a glyph gap.
            patch = depth_cm[vi - 1:vi + 2, ui - 1:ui + 2]
            measured = float(np.median(patch))
            if measured <= 0.0 or measured >= 6500.0:
                continue

            axis = float(camera[0])
            ray = float(np.linalg.norm(camera))
            axis_ratios.append(measured / axis)
            ray_ratios.append(measured / ray)
            abs_errors_cm.append(measured - axis)
            checked += 1

    if checked < 20:
        print(f"only {checked} landmark samples; not enough to judge")
        return 2

    axis = np.array(axis_ratios)
    ray = np.array(ray_ratios)
    report = {
        "session": session.name,
        "samples": checked,
        "axis_ratio_median": round(float(np.median(axis)), 4),
        "axis_ratio_p10_p90": [round(float(np.percentile(axis, 10)), 4), round(float(np.percentile(axis, 90)), 4)],
        "ray_ratio_median": round(float(np.median(ray)), 4),
        "axis_abs_error_cm_median": round(float(np.median(np.abs(abs_errors_cm))), 2),
        "axis_abs_error_cm_p90": round(float(np.percentile(np.abs(abs_errors_cm), 90)), 2),
    }
    print(json.dumps(report, indent=2))

    verdict = "axis distance (metric, centimetres)" if abs(report["axis_ratio_median"] - 1.0) < 0.02 else (
        "ray distance" if abs(report["ray_ratio_median"] - 1.0) < 0.02 else "unknown scaling")
    print(f"verdict: depth matches {verdict}")
    (session / "depth_check.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
