"""Score the map with the synthetic labels as known 3D references.

    python ue5/tools/evaluate_landmarks.py output/ue5-datasets/line

Point-cloud-to-point-cloud comparison is a poor fit for this scene: the labels are
thin sheets floating in front of a wall, so a subsampled reference cloud almost never
lands on a label and every reconstructed point looks "1.5 m off" because its nearest
reference sample is the wall behind.

Instead: bring the reconstruction into the ground-truth frame with the trajectory
Sim(3), then ask, for the reconstructed points that sit near a label, how close they
are to that label's known centre. Plus the reverse question that matters for a map:
how many labels actually got reconstructed near them.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pycolmap

sys.path.insert(0, str(Path(__file__).parent))
from evaluate_slam import camera_center, umeyama  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--model", default=None)
    parser.add_argument("--radius", type=float, default=2.0, help="only score points this close to a label")
    args = parser.parse_args()

    session = Path(args.session).resolve()
    landmarks_path = session / "landmarks.json"
    if not landmarks_path.exists():
        print("session has no landmarks.json")
        return 1

    model_path = Path(args.model) if args.model else (
        Path("output/ue5-eval") / f"colmap-{session.name}" / "best")
    if not model_path.exists():
        print(f"missing model {model_path}")
        return 1

    landmarks = np.array(json.loads(landmarks_path.read_text(encoding="utf-8"))["points"], dtype=float)
    landmarks = np.stack([landmarks[:, 0], -landmarks[:, 1], landmarks[:, 2]], axis=1) / 100.0

    # Labels were placed facing the start point, so their normal points back at it.
    # Perpendicular distance to that plane is the part of the error the map can get
    # wrong; the rest is just where on the sheet a point landed.
    meta = json.loads((session / "meta.json").read_text(encoding="utf-8"))
    start = np.array(meta["start_pose"]["loc"], dtype=float)
    origin = np.array([start[0], -start[1], start[2]]) / 100.0
    normals = origin[None, :] - landmarks
    normals[:, 2] = 0.0
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.maximum(lengths, 1e-6)

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
    cloud = (scale * (rotation @ cloud.T)).T + translation

    distances = np.linalg.norm(cloud[:, None, :] - landmarks[None, :, :], axis=2)
    nearest = distances.min(axis=1)
    kept = nearest[nearest < args.radius]

    # Perpendicular distance to the nearest label plane, for points near a label.
    deltas = cloud[:, None, :] - landmarks[None, :, :]
    plane_distance = np.abs((deltas * normals[None, :, :]).sum(axis=2))
    nearest_index = distances.argmin(axis=1)
    perpendicular = plane_distance[np.arange(len(cloud)), nearest_index]
    perpendicular = perpendicular[nearest < args.radius]

    labels_with_points = (distances < 0.5).any(axis=0)

    report = {
        "session": session.name,
        "landmarks": int(len(landmarks)),
        "reconstruction_points": int(len(cloud)),
        "points_within_radius": round(float(len(kept) / max(1, len(cloud))), 3),
        "distance_to_nearest_label_m": {
            "median": round(float(np.median(kept)), 3) if len(kept) else None,
            "mean": round(float(kept.mean()), 3) if len(kept) else None,
            "p90": round(float(np.percentile(kept, 90)), 3) if len(kept) else None,
        },
        "perpendicular_to_label_plane_m": {
            "median": round(float(np.median(perpendicular)), 3) if len(perpendicular) else None,
            "p90": round(float(np.percentile(perpendicular, 90)), 3) if len(perpendicular) else None,
        },
        "labels_with_a_point_within_0_5_m": f"{int(labels_with_points.sum())}/{len(landmarks)}",
        "label_recall": round(float(labels_with_points.mean()), 3),
    }
    print(json.dumps(report, indent=2))

    out = model_path.parent / "landmark_eval.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
