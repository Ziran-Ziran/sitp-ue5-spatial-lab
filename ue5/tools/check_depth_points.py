"""Validate the recorded depth against the reconstruction's own 3D points.

    python ue5/tools/check_depth_points.py output/ue5-datasets/weave

Each reconstructed point knows which pixel it was observed at and which image that
was. Bring the point into the ground-truth frame with the trajectory Sim(3), compute
its distance along the ground-truth camera axis, and compare with the recorded depth
at that pixel. If the depth map means what the recorder claims -?metric distance
along the optical axis -?the two agree.

This is stronger than comparing whole point clouds: it compares the same surface at
the same pixel, so sampling density and untextured regions do not matter.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pycolmap

sys.path.insert(0, str(Path(__file__).parent))
from evaluate_slam import camera_center, umeyama  # noqa: E402


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
    parser.add_argument("--model", default=None)
    parser.add_argument("--max-samples", type=int, default=4000)
    args = parser.parse_args()

    session = Path(args.session).resolve()
    model_path = Path(args.model) if args.model else (
        Path("output/ue5-eval") / f"colmap-{session.name}" / "best")
    if not model_path.exists():
        print(f"missing model {model_path}")
        return 1

    poses = {}
    for line in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        pose = json.loads(line)
        poses[f"frame_{pose['i']:06d}.png"] = pose

    reconstruction = pycolmap.Reconstruction(str(model_path))

    # Sim(3) from the trajectory, so the reconstruction's arbitrary scale is resolved.
    truth = {}
    for name, pose in poses.items():
        x, y, z = pose["loc"]
        truth[name] = np.array([x, -y, z], dtype=float) / 100.0
    estimated, expected = [], []
    for image in reconstruction.images.values():
        if image.name in truth:
            estimated.append(camera_center(image))
            expected.append(truth[image.name])
    rotation, translation, scale = umeyama(np.asarray(estimated), np.asarray(expected))

    depth_cache = {}
    pairs = []
    for point in reconstruction.points3D.values():
        if len(pairs) >= args.max_samples:
            break
        world = scale * (rotation @ np.asarray(point.xyz, dtype=float)) + translation
        for element in point.track.elements:
            image = reconstruction.images[element.image_id] if element.image_id in reconstruction.images else None
            if image is None or image.name not in poses:
                continue
            pose = poses[image.name]
            key = image.name
            if key not in depth_cache:
                depth_cache[key] = cv2.imread(
                    str(session / "depth" / key), cv2.IMREAD_UNCHANGED)
            depth_cm = depth_cache[key]
            if depth_cm is None:
                continue
            point2d = image.points2D[element.point2D_idx]
            u, v = int(round(point2d.xy[0])), int(round(point2d.xy[1]))
            if not (2 <= u < pose["w"] - 2 and 2 <= v < pose["h"] - 2):
                continue
            measured_mm = float(np.median(depth_cm[v - 1:v + 2, u - 1:u + 2]))
            if measured_mm <= 0 or measured_mm >= 65000:
                continue

            # The cloud is in the right-handed evaluation frame, so the pose must be
            # converted too: mirror Y on both the rotation and the position.
            mirror = np.diag([1.0, -1.0, 1.0])
            rotation_rh = mirror @ quat_to_matrix(pose["quat"]) @ mirror
            centre_rh = np.array([pose["loc"][0], -pose["loc"][1], pose["loc"][2]]) / 100.0
            cam = rotation_rh.T @ (world - centre_rh)
            if cam[0] <= 0.2:
                continue
            pairs.append((cam[0], measured_mm / 1000.0, np.linalg.norm(cam)))
            break

    if len(pairs) < 100:
        print(f"only {len(pairs)} usable pairs")
        return 2

    predicted = np.array([p[0] for p in pairs])
    measured = np.array([p[1] for p in pairs])
    ray = np.array([p[2] for p in pairs])

    ratio_axis = measured / predicted
    ratio_ray = measured / ray
    residual = np.abs(measured - predicted)

    report = {
        "session": session.name,
        "samples": len(pairs),
        "measured_over_axis_median": round(float(np.median(ratio_axis)), 4),
        "measured_over_axis_p10_p90": [
            round(float(np.percentile(ratio_axis, 10)), 4),
            round(float(np.percentile(ratio_axis, 90)), 4),
        ],
        "measured_over_ray_median": round(float(np.median(ratio_ray)), 4),
        "abs_error_vs_axis_m": {
            "median": round(float(np.median(residual)), 3),
            "p90": round(float(np.percentile(residual, 90)), 3),
        },
        "median_depth_m": round(float(np.median(measured)), 2),
    }
    print(json.dumps(report, indent=2))

    out = model_path.parent / "depth_points_check.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
