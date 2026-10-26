"""Score the reconstruction's map against a reference cloud built from ground truth.

    python ue5/tools/evaluate_map.py output/ue5-datasets/line

The reference cloud is produced by back-projecting the recorded depth map with the
recorded pose and intrinsics: that is the true geometry of what the camera saw.
The reconstruction's points are brought into the same frame with the trajectory
Sim(3), then the two clouds are compared with a nearest-neighbour distance.

This is both a map metric and a depth check: if the depth map did not mean what the
recorder claims (metric distance along the camera axis), the two clouds would not
overlap even though the trajectory aligned.
"""

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


def reference_cloud(session, stride, pixel_stride):
    poses = [json.loads(line) for line in
             (session / "poses.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    points = []
    for pose in poses[::stride]:
        depth_path = session / "depth" / f"frame_{pose['i']:06d}.png"
        if not depth_path.exists():
            continue
        raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if raw is None:
            continue
        depth_m = raw.astype(np.float32) / 1000.0
        rotation = quat_to_matrix(pose["quat"])
        centre = np.array(pose["loc"], dtype=float) / 100.0

        rows = np.arange(0, pose["h"], pixel_stride)
        cols = np.arange(0, pose["w"], pixel_stride)
        grid_v, grid_u = np.meshgrid(rows, cols, indexing="ij")
        d = depth_m[grid_v, grid_u]
        valid = (d > 0.1) & (d < 60.0)
        if not valid.any():
            continue
        u = grid_u[valid].astype(np.float64)
        v = grid_v[valid].astype(np.float64)
        z = d[valid].astype(np.float64)

        camera = np.stack([
            z,
            (u - pose["cx"]) / pose["fx"] * z,
            -(v - pose["cy"]) / pose["fy"] * z,
        ], axis=1)
        world = camera @ rotation.T + centre
        # right-handed frame, same convention as the evaluation
        world[:, 1] *= -1.0
        points.append(world)

    if not points:
        return np.zeros((0, 3))
    return np.concatenate(points, axis=0)


def main() -> int:
    parser = argparse.ArgumentParser() if False else None
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--model", default=None)
    parser.add_argument("--stride", type=int, default=5, help="every Nth depth frame")
    parser.add_argument("--pixel-stride", type=int, default=6)
    parser.add_argument("--max-points", type=int, default=4000)
    args = parser.parse_args()

    session = Path(args.session).resolve()
    model_path = Path(args.model) if args.model else (
        Path("output/ue5-eval") / f"colmap-{session.name}" / "best")
    if not model_path.exists():
        print(f"missing model {model_path}")
        return 1

    truth = {}
    for line in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        pose = json.loads(line)
        x, y, z = pose["loc"]
        truth[f"frame_{pose['i']:06d}.png"] = np.array([x, -y, z], dtype=float) / 100.0

    reconstruction = pycolmap.Reconstruction(str(model_path))
    estimated, expected = [], []
    for image in reconstruction.images.values():
        if image.name in truth:
            estimated.append(camera_center(image))
            expected.append(truth[image.name])
    if len(estimated) < 10:
        print("not enough registered images")
        return 2

    rotation, translation, scale = umeyama(np.asarray(estimated), np.asarray(expected))

    cloud = np.array([np.asarray(p.xyz, dtype=float) for p in reconstruction.points3D.values()])
    aligned = (scale * (rotation @ cloud.T)).T + translation

    reference = reference_cloud(session, args.stride, args.pixel_stride)
    if len(reference) == 0:
        print("no depth frames found; nothing to compare")
        return 3

    rng = np.random.default_rng(0)
    def subsample(points):
        if len(points) <= args.max_points:
            return points
        index = rng.choice(len(points), args.max_points, replace=False)
        return points[index]

    a = subsample(aligned)
    b = subsample(reference)

    def nearest(source, target):
        distances = np.linalg.norm(source[:, None, :] - target[None, :, :], axis=2).min(axis=1)
        return distances

    map_to_ref = nearest(a, b)
    ref_to_map = nearest(b, a)

    report = {
        "session": session.name,
        "registered_images": len(estimated),
        "reconstruction_points": int(len(cloud)),
        "reference_points": int(len(reference)),
        "sampled": int(len(a)),
        "map_to_reference_m": {
            "median": round(float(np.median(map_to_ref)), 3),
            "mean": round(float(map_to_ref.mean()), 3),
            "p90": round(float(np.percentile(map_to_ref, 90)), 3),
        },
        "reference_to_map_m": {
            "median": round(float(np.median(ref_to_map)), 3),
            "mean": round(float(ref_to_map.mean()), 3),
            "p90": round(float(np.percentile(ref_to_map, 90)), 3),
        },
        "sim3_scale": round(float(scale), 4),
    }
    print(json.dumps(report, indent=2))

    out = model_path.parent / "map_eval.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
