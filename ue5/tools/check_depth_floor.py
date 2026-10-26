"""Decide what the depth map measures, using the floor as a known plane.

    python ue5/tools/check_depth_floor.py output/ue5-datasets/line --frame 30

A camera at height h looking level sees the floor at axis-distance h / tan(theta) for
a ray theta below the optical axis, or h / sin(theta) if the stored value is ray
distance. Whichever formula returns a constant height down a column is the convention
the engine used. This needs no landmarks and no thin geometry.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--frame", type=int, default=30)
    parser.add_argument("--column-offset", type=int, default=0)
    args = parser.parse_args()

    session = Path(args.session)
    poses = {json.loads(l)["i"]: json.loads(l) for l in
             (session / "poses.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()}
    pose = poses[args.frame]

    depth = cv2.imread(str(session / "depth" / f"frame_{args.frame:06d}.png"), cv2.IMREAD_UNCHANGED)
    if depth is None:
        print("no depth frame")
        return 1
    depth_cm = depth.astype(np.float64) / 10.0

    column = int(pose["cx"]) + args.column_offset
    rows = np.arange(int(pose["cy"]) + 30, int(pose["h"]) - 3, 2)
    values = depth_cm[rows, column]
    keep = (values > 50.0) & (values < 6000.0)
    rows, values = rows[keep], values[keep]
    if len(rows) < 20:
        print("not enough floor pixels")
        return 2

    theta = np.arctan((rows - pose["cy"]) / pose["fy"])
    height_axis = values * np.tan(theta)
    height_ray = values * np.sin(theta)

    def describe(samples):
        return {
            "median_cm": round(float(np.median(samples)), 1),
            "iqr_cm": round(float(np.percentile(samples, 75) - np.percentile(samples, 25)), 1),
            "relative_spread": round(float(np.std(samples) / np.median(samples)), 3),
        }

    report = {
        "session": session.name,
        "frame": args.frame,
        "samples": int(len(rows)),
        "assume_axis_distance": describe(height_axis),
        "assume_ray_distance": describe(height_ray),
    }
    print(json.dumps(report, indent=2))

    better = "axis" if report["assume_axis_distance"]["relative_spread"] < report["assume_ray_distance"]["relative_spread"] else "ray"
    print(f"better fit: {better} distance (smaller relative spread down the column)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
