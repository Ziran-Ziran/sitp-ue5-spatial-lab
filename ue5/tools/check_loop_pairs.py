"""Measure whether loop closure is even possible in a session.

    python ue5/tools/check_loop_pairs.py output/ue5-datasets/city-04 output/ue5-datasets/weave

Loop closure needs two frames of the same place to share enough stable, correctly
matchable features. If a return frame only yields a dozen geometric inliers against
the start frame, no algorithm can close the loop reliably - the information is not
there, and tuning parameters just changes how the failure looks.

Prints, for a reference frame and a few increasingly distant frames: features,
ratio-test matches, and RANSAC inliers of the essential matrix.
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np


def load_pose(session, index):
    for line in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        pose = json.loads(line)
        if pose["i"] == index:
            return pose
    return None


def pair_report(session, first, second, sift):
    image_a = cv2.imread(str(session / "frames" / f"frame_{first:06d}.png"), cv2.IMREAD_GRAYSCALE)
    image_b = cv2.imread(str(session / "frames" / f"frame_{second:06d}.png"), cv2.IMREAD_GRAYSCALE)
    if image_a is None or image_b is None:
        return None

    keypoints_a, descriptors_a = sift.detectAndCompute(image_a, None)
    keypoints_b, descriptors_b = sift.detectAndCompute(image_b, None)
    if descriptors_a is None or descriptors_b is None:
        return None

    matcher = cv2.BFMatcher(cv2.NORM_L2)
    pairs = matcher.knnMatch(descriptors_a, descriptors_b, k=2)
    good = [m for pair in pairs if len(pair) == 2 for m, n in [pair] if m.distance < 0.75 * n.distance]
    if len(good) < 8:
        return {"pair": [first, second], "features": [len(keypoints_a), len(keypoints_b)],
                "matches": len(good), "inliers": 0}

    pose = load_pose(session, first)
    points_a = np.float32([keypoints_a[m.queryIdx].pt for m in good])
    points_b = np.float32([keypoints_b[m.trainIdx].pt for m in good])
    essential, mask = cv2.findEssentialMat(
        points_a, points_b, focal=pose["fx"], pp=(pose["cx"], pose["cy"]),
        method=cv2.RANSAC, prob=0.999, threshold=1.0)
    inliers = int(mask.sum()) if mask is not None else 0
    return {"pair": [first, second], "features": [len(keypoints_a), len(keypoints_b)],
            "matches": len(good), "inliers": inliers}


def main() -> int:
    sift = cv2.SIFT_create()
    results = {}
    for argument in sys.argv[1:]:
        session = Path(argument)
        poses = sorted(json.loads(l)["i"] for l in
                       (session / "poses.jsonl").read_text(encoding="utf-8").splitlines() if l.strip())
        first, last = poses[0], poses[-1]

        # Baseline: adjacent frames. Then progressively further away, ending at the
        # last frame, which for a closed loop is the same place as the first.
        targets = [first + 3, first + 30, (first + last) // 2, last - 30, last]
        rows = [pair_report(session, first, t, sift) for t in targets]
        rows = [row for row in rows if row]
        results[session.name] = rows

        print(f"== {session.name} (reference frame {first})")
        print(f"{'target':>8} {'matches':>8} {'inliers':>8}")
        for row in rows:
            print(f"{row['pair'][1]:>8} {row['matches']:>8} {row['inliers']:>8}")

    out = Path("output/ue5-eval/loop_feasibility.json")
    out.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(f"\nwritten to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
