"""Compare a reconstruction against the engine's ground-truth camera path.

    python ue5/tools/evaluate_slam.py output/ue5-datasets/circle

Ground truth is the recorder's own poses.jsonl. The estimate is a COLMAP binary
model. Monocular scale is unobservable, so the comparison uses a similarity
transform (Umeyama): rotation, translation and one scale factor. The reported
ATE is the RMS error after that alignment, in metres.

UE is left-handed (X forward, Y right, Z up, centimetres). Ground truth is
mirrored to Y-left, which is right-handed, before alignment; without that the
alignment would be forced to absorb a reflection it cannot represent.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pycolmap


def camera_center(image):
    pose = image.cam_from_world
    if callable(pose):
        pose = pose()
    for attribute in ("inverse",):
        if hasattr(pose, attribute):
            try:
                return np.array(getattr(pose, attribute)().translation, dtype=float)
            except Exception:
                pass
    matrix = np.array(pose.matrix(), dtype=float)  # 3x4 world->cam
    rotation, translation = matrix[:, :3], matrix[:, 3]
    return -rotation.T @ translation


def umeyama(source, target):
    """Least-squares similarity mapping source onto target. Returns (R, t, scale)."""
    mu_source = source.mean(axis=0)
    mu_target = target.mean(axis=0)
    source_centred = source - mu_source
    target_centred = target - mu_target

    covariance = target_centred.T @ source_centred / len(source)
    u, singular, vt = np.linalg.svd(covariance)

    correction = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[2, 2] = -1.0

    rotation = u @ correction @ vt
    variance = (source_centred ** 2).sum() / len(source)
    scale = float((singular * np.diag(correction)).sum() / variance)
    translation = mu_target - scale * rotation @ mu_source
    return rotation, translation, scale


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--model", default=None, help="default output/ue5-eval/colmap-<session>/best")
    parser.add_argument("--plot", default=None)
    args = parser.parse_args()

    session = Path(args.session).resolve()
    model_path = Path(args.model) if args.model else (
        Path("output/ue5-eval") / f"colmap-{session.name}" / "best")
    if not model_path.exists():
        print(f"missing model {model_path}; run run_colmap.py first")
        return 1

    truth = {}
    for line in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        pose = json.loads(line)
        x, y, z = pose["loc"]
        # centimetres -> metres, and mirror Y so the frame is right-handed
        truth[f"frame_{pose['i']:06d}.png"] = np.array([x, -y, z], dtype=float) / 100.0

    reconstruction = pycolmap.Reconstruction(str(model_path))
    estimated, expected, names = [], [], []
    for image in reconstruction.images.values():
        if image.name in truth:
            estimated.append(camera_center(image))
            expected.append(truth[image.name])
            names.append(image.name)

    if len(names) < 10:
        print(f"only {len(names)} images intersect ground truth; cannot evaluate")
        return 2

    # COLMAP iterates images in arbitrary order; path length and plots are only
    # meaningful in capture order.
    order = sorted(range(len(names)), key=lambda index: names[index])
    names = [names[index] for index in order]
    estimated = np.asarray([estimated[index] for index in order])
    expected = np.asarray([expected[index] for index in order])

    rotation, translation, scale = umeyama(estimated, expected)
    aligned = (scale * (rotation @ estimated.T)).T + translation
    errors = np.linalg.norm(aligned - expected, axis=1)

    def path_length(points):
        return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())

    coverage = len(names) / len(truth)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1.0) / 2.0))))

    report = {
        "session": session.name,
        "registered_of_truth": f"{len(names)}/{len(truth)}",
        "coverage": round(coverage, 3),
        "ate_rmse_m": round(float(np.sqrt((errors ** 2).mean())), 4),
        "ate_median_m": round(float(np.median(errors)), 4),
        "ate_max_m": round(float(errors.max()), 4),
        "ate_relative_percent": round(100.0 * float(np.sqrt((errors ** 2).mean())) / max(1e-9, path_length(expected)), 3),
        "sim3_scale": round(scale, 5),
        "sim3_rotation_deg": round(angle, 3),
        "gt_path_m": round(path_length(expected), 2),
        "est_path_m_after_scale": round(path_length(aligned), 2),
        "points3d": reconstruction.num_points3D(),
    }

    # The trajectory only says where the camera went. Landmarks are the only ground
    # truth for "did the map put structure where it belongs", so score them when the
    # session recorded their positions.
    plot_landmarks = None
    landmarks_path = session / "landmarks.json"
    if landmarks_path.exists():
        raw = json.loads(landmarks_path.read_text(encoding="utf-8"))["points"]
        landmarks = np.array([[x, -y, z] for x, y, z in raw], dtype=float) / 100.0
        plot_landmarks = landmarks
        cloud = np.array([np.asarray(point.xyz, dtype=float) for point in reconstruction.points3D.values()])
        aligned_cloud = (scale * (rotation @ cloud.T)).T + translation
        distances = np.linalg.norm(
            landmarks[:, None, :] - aligned_cloud[None, :, :], axis=2).min(axis=1)
        report["landmarks"] = {
            "count": int(len(landmarks)),
            "median_m": round(float(np.median(distances)), 3),
            "mean_m": round(float(distances.mean()), 3),
            "p90_m": round(float(np.percentile(distances, 90)), 3),
            "within_0_2_m": round(float((distances < 0.2).mean()), 3),
        }
        report["cloud_points"] = int(len(cloud))

    out_dir = model_path.parent
    (out_dir / "eval.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(1, 2, figsize=(12, 5))
        axes[0].plot(expected[:, 0], expected[:, 1], label="ground truth", linewidth=2)
        axes[0].plot(aligned[:, 0], aligned[:, 1], label="reconstruction", linestyle="--")
        if plot_landmarks is not None:
            axes[0].scatter(plot_landmarks[:, 0], plot_landmarks[:, 1],
                            marker="x", s=18, alpha=0.5, label="landmarks")
        axes[0].set_aspect("equal")
        axes[0].set_title("top view, Sim(3) aligned")
        axes[0].set_xlabel("x [m]")
        axes[0].set_ylabel("y [m]")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(errors)
        axes[1].set_title(f"per-frame error (RMSE {report['ate_rmse_m']} m)")
        axes[1].set_xlabel("frame")
        axes[1].set_ylabel("error [m]")
        axes[1].grid(True, alpha=0.3)

        figure.tight_layout()
        plot_path = Path(args.plot) if args.plot else out_dir / "trajectory.png"
        figure.savefig(plot_path, dpi=120)
        print(f"plot -> {plot_path}")
    except Exception as error:  # plotting is a convenience, not a result
        print(f"plot skipped: {error}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
