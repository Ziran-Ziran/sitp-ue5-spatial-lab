"""Frame-by-frame RGB-D visual odometry baseline.

    python ue5/tools/rgbd_vo.py output/ue5-datasets/weave

This is the online counterpart to the offline COLMAP reconstruction: it sees one
frame at a time, never revisits, never bundles, and therefore accumulates drift.
Reading the two side by side is the point - a 3 cm offline result and a drifting
online result are measuring different things.

Method per frame: ORB features, previous-frame 3D points from the depth map,
2D-3D correspondences on the current frame, PnP with RANSAC, compose the pose.
Depth makes the result metric, so the reported scale should be close to 1.

Camera convention here is OpenCV (x right, y down, z forward); the depth map is
metric distance along the optical axis, which is exactly z in this convention.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def load_poses(session):
    poses = {}
    for line in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        pose = json.loads(line)
        poses[pose["i"]] = pose
    return poses


def backproject(depth_m, pose, stride=2):
    """Return (N,3) points in the camera frame and their (u,v) pixels."""
    rows = np.arange(0, pose["h"], stride)
    cols = np.arange(0, pose["w"], stride)
    grid_v, grid_u = np.meshgrid(rows, cols, indexing="ij")
    d = depth_m[grid_v, grid_u]
    valid = (d > 0.2) & (d < 60.0)
    u = grid_u[valid].astype(np.float64)
    v = grid_v[valid].astype(np.float64)
    z = d[valid].astype(np.float64)
    points = np.stack([
        (u - pose["cx"]) / pose["fx"] * z,
        (v - pose["cy"]) / pose["fy"] * z,
        z,
    ], axis=1)
    return points, np.stack([u, v], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--out")
    parser.add_argument("--odometry-only", action="store_true", help="save native poses without opening ground truth")
    parser.add_argument("--stride", type=int, default=2, help="depth back-projection stride")
    args = parser.parse_args()

    session = Path(args.session)
    from colmap_depth_poses import load_calibration
    calibration = load_calibration(session)
    indices = sorted(int(path.stem.split("_")[-1]) for path in (session / "frames").glob("frame_*.png"))
    frames_total = len(indices)
    if not indices:
        raise ValueError("no RGB frames")
    poses = {i: calibration for i in indices}
    if args.max_frames:
        indices = indices[: args.max_frames]

    orb = cv2.ORB_create(nfeatures=3000, fastThreshold=15)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    # First camera defines the world frame of the estimate.
    trajectory = [np.eye(4)]
    failures = 0
    inliers_history = []
    pose_status = ["origin"]

    previous = None
    for step, index in enumerate(indices):
        pose = poses[index]
        image = cv2.imread(str(session / "frames" / f"frame_{index:06d}.png"), cv2.IMREAD_GRAYSCALE)
        depth_raw = cv2.imread(str(session / "depth" / f"frame_{index:06d}.png"), cv2.IMREAD_UNCHANGED)
        if image is None or depth_raw is None:
            print(f"frame {index} missing")
            return 1
        depth_m = depth_raw.astype(np.float32) / 1000.0

        keypoints, descriptors = orb.detectAndCompute(image, None)

        if previous is None:
            previous = (index, keypoints, descriptors, depth_m)
            continue

        prev_index, prev_keypoints, prev_descriptors, prev_depth = previous
        if descriptors is None or prev_descriptors is None or len(keypoints) < 10:
            failures += 1
            pose_status.append("held_after_pnp_failure")
            trajectory.append(trajectory[-1].copy())
            previous = (index, keypoints, descriptors, depth_m)
            continue

        matches = matcher.knnMatch(prev_descriptors, descriptors, k=2)
        good = [m for pair in matches if len(pair) == 2 for m, n in [pair] if m.distance < 0.8 * n.distance]

        object_points = []
        image_points = []
        for match in good:
            u, v = prev_keypoints[match.queryIdx].pt
            ui, vi = int(round(u)), int(round(v))
            if not (0 <= ui < pose["w"] and 0 <= vi < pose["h"]):
                continue
            z = float(prev_depth[vi, ui])
            if z < 0.2 or z > 60.0:
                continue
            object_points.append([
                (u - pose["cx"]) / pose["fx"] * z,
                (v - pose["cy"]) / pose["fy"] * z,
                z,
            ])
            image_points.append(keypoints[match.trainIdx].pt)

        if len(object_points) < 12:
            failures += 1
            pose_status.append("held_after_pnp_failure")
            trajectory.append(trajectory[-1].copy())
            previous = (index, keypoints, descriptors, depth_m)
            continue

        camera_matrix = np.array([[pose["fx"], 0, pose["cx"]], [0, pose["fy"], pose["cy"]], [0, 0, 1]])
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            np.asarray(object_points, dtype=np.float64),
            np.asarray(image_points, dtype=np.float64),
            camera_matrix, None,
            reprojectionError=3.0, iterationsCount=200,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inliers is None or len(inliers) < 10:
            failures += 1
            pose_status.append("held_after_pnp_failure")
            trajectory.append(trajectory[-1].copy())
            previous = (index, keypoints, descriptors, depth_m)
            continue

        rotation, _ = cv2.Rodrigues(rvec)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = tvec.ravel()
        # PnP gives current-from-previous; compose onto the running pose.
        trajectory.append(trajectory[-1] @ np.linalg.inv(transform))
        inliers_history.append(len(inliers))
        pose_status.append("pnp")
        previous = (index, keypoints, descriptors, depth_m)

    estimate = np.array([T[:3, 3] for T in trajectory])

    # Persist the algorithm output before opening any evaluation-only truth.
    out = Path(args.out) if args.out else Path("output/ue5-eval") / f"colmap-{session.name}"
    out.mkdir(parents=True, exist_ok=True)
    with (out / "vo_poses.jsonl").open("w", encoding="utf-8") as handle:
        for index, transform, status in zip(indices, trajectory, pose_status):
            handle.write(json.dumps(dict(i=int(index),
                T=[round(float(value), 9) for value in transform.ravel()],
                status=status)) + "\n")
    native = dict(session=session.name, frames=len(indices), frames_total=frames_total,
                  coverage=round(len(indices)/frames_total, 4), pnp_failures=failures,
                  held_frames=failures, estimated_pose_frames=len(indices)-failures,
                  mean_inliers=round(float(np.mean(inliers_history)), 1) if inliers_history else 0.,
                  ground_truth_used=False, calibration=calibration,
                  frame_convention="OpenCV camera to first camera, metres")
    (out / 'vo_native.json').write_text(json.dumps(native, indent=2), encoding='utf-8')
    if args.odometry_only:
        print(json.dumps(native, indent=2))
        return 0

    # Only the evaluation phase below opens poses.jsonl.
    poses = load_poses(session)
    if not all(i in poses for i in indices):
        raise ValueError('truth is missing captured frame indices; native output was saved')

    # Ground truth camera centres in the right-handed evaluation frame (metres).
    truth = []
    for index in indices:
        pose = poses[index]
        truth.append([pose["loc"][0], -pose["loc"][1], pose["loc"][2]])
    truth = np.asarray(truth, dtype=float) / 100.0

    # Similarity alignment, to separate drift from a fixed frame difference.
    mu_e = estimate.mean(axis=0)
    mu_t = truth.mean(axis=0)
    ec = estimate - mu_e
    tc = truth - mu_t
    covariance = tc.T @ ec / len(estimate)
    u, singular, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[2, 2] = -1.0
    rotation = u @ correction @ vt
    scale = float((singular * np.diag(correction)).sum() / ((ec ** 2).sum() / len(estimate)))
    translation = mu_t - scale * rotation @ mu_e
    aligned = (scale * (rotation @ estimate.T)).T + translation
    errors = np.linalg.norm(aligned - truth, axis=1)

    # Because the depth is metric, the estimate should need no scale at all. Re-fit
    # with scale fixed at 1 to report a purely rigid (metric) error as well.
    rigid_rotation = u @ correction @ vt
    rigid_translation = mu_t - rigid_rotation @ mu_e
    rigid = (rigid_rotation @ estimate.T).T + rigid_translation
    rigid_errors = np.linalg.norm(rigid - truth, axis=1)

    def path_length(points):
        return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())

    report = {
        "session": session.name,
        "frames": len(indices),
        "frames_of_truth": f"{len(indices)}/{len(poses)}",
        "coverage": round(len(indices)/len(poses), 4),
        "ground_truth_usage": "evaluation after native poses saved",
        "pnp_failures": failures,
        "mean_inliers": round(float(np.mean(inliers_history)), 1) if inliers_history else 0.0,
        "ate_rmse_m": round(float(np.sqrt((errors ** 2).mean())), 4),
        "ate_max_m": round(float(errors.max()), 4),
        "ate_rigid_rmse_m": round(float(np.sqrt((rigid_errors ** 2).mean())), 4),
        "ate_rigid_max_m": round(float(rigid_errors.max()), 4),
        "final_error_m": round(float(errors[-1]), 4),
        "gt_path_m": round(path_length(truth), 2),
        "est_path_m": round(path_length(estimate), 2),
        "aligned_path_m": round(path_length(aligned), 2),
        "scale": round(scale, 4),
        "drift_percent_of_path": round(100.0 * float(errors.max()) / max(1e-9, path_length(truth)), 2),
    }
    print(json.dumps(report, indent=2))

    (out / "vo_eval.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(1, 2, figsize=(12, 5))
        axes[0].plot(truth[:, 0], truth[:, 1], label="ground truth", linewidth=2)
        axes[0].plot(aligned[:, 0], aligned[:, 1], label="rgbd vo", linestyle="--")
        axes[0].set_aspect("equal")
        axes[0].set_title("top view, aligned")
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
        figure.savefig(out / "vo_trajectory.png", dpi=120)
        print(f"plot -> {out / 'vo_trajectory.png'}")
    except Exception as error:
        print(f"plot skipped: {error}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
