"""Fuse the recorded depth maps into a dense map, and score that map.

    python ue5/tools/dense_map.py output/ue5-datasets/city-lap2 --poses gt
    python ue5/tools/dense_map.py output/ue5-datasets/city-lap2 --poses vo
    python ue5/tools/dense_map.py output/ue5-datasets/city-lap2 --poses colmap

Everything before this produced a *sparse* map: COLMAP keeps only the pixels that
became triangulated landmarks (about 12k points for an 85 m walk). A depth map has a
value for every pixel, so fusing them gives a real surface -- and that is the thing a
robot would actually use.

Pose sources:

    --poses gt   the engine's own poses. This isolates the fusion itself: any error in
                 the map is my back-projection or my voxel grid, nothing else.
    --poses vo   the RGB-D odometry from rgbd_vo.py. Same fusion, drifting poses. The
                 difference between the two runs is what pose error does to a map.
    --poses colmap   metric scale from sparse-point/depth correspondences. Fusion
                    runs in native COLMAP coordinates without ground truth.

For estimated poses, all truth alignment happens AFTER native fusion is saved.
Sim(3) evaluation outputs never feed back into fusion. --fuse-only works without
poses.jsonl, landmarks.json or the meta.json start_pose. The gt branch is explicitly
an oracle reference, not an estimated map. --evaluate-only scores saved native data.

Scoring uses the same 53 unique text labels as the sparse evaluations, so the dense
and sparse numbers are comparable: how many labels have map points within 0.5 m, and
how far those points sit from the label's known plane. A third metric is the map's own
surface thickness (local PCA residual), which needs no ground truth and is what a
fused map visibly gets wrong when the poses jitter.

Conventions (validated by check_depth_points.py, not assumed):
    depth PNG     16-bit, millimetres, measured along the camera's optical axis
    camera axes   X forward, Y right, Z up (UE component axes)
    eval frame    (x, -y, z) / 100, right-handed, metres -- Y is mirrored
    rotation      R_eval = mirror @ R(quat) @ mirror, since the mirror must be applied
                  to both ends of the rotation
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree

MIRROR = np.diag([1.0, -1.0, 1.0])


def quat_to_matrix(quat):
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def load_ground_truth(session):
    return load_pose_records(session / "poses.jsonl")


def load_pose_records(path):
    poses = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            pose = json.loads(line)
            poses[pose["i"]] = pose
    return poses


def pose_transform(pose):
    """Decode loc/quat fields to a RH metric transform in their stated world frame."""
    rotation = MIRROR @ quat_to_matrix(pose["quat"]) @ MIRROR
    centre = np.array([pose["loc"][0], -pose["loc"][1], pose["loc"][2]], dtype=float) / 100.0
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = centre
    return transform


# Compatibility for callers of the original evaluation helper. The decoder itself
# uses no truth: COLMAP records have the same encoding in a different world frame.
ground_truth_pose = pose_transform


def load_vo_poses(session):
    """Odometry poses, already converted to the eval frame's camera axes."""
    path = Path("output/ue5-eval") / f"colmap-{session.name}" / "vo_poses.jsonl"
    if not path.exists():
        raise SystemExit(f"missing {path}; run rgbd_vo.py on this session first")
    # OpenCV camera axes (x right, y down, z forward) -> eval camera axes
    # (x forward, y left, z up). Both are right-handed, so this is a rotation.
    to_eval_axes = np.array([[0.0, 0.0, 1.0],
                             [-1.0, 0.0, 0.0],
                             [0.0, -1.0, 0.0]])
    poses = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        transform = np.array(record["T"], dtype=float).reshape(4, 4)
        transform[:3, :3] = transform[:3, :3] @ to_eval_axes.T
        poses[record["i"]] = transform
    return poses


def rigid_align(source, target):
    """Least-squares rotation+translation, scale fixed at 1. Metric depth needs no scale."""
    mu_source = source.mean(axis=0)
    mu_target = target.mean(axis=0)
    covariance = (target - mu_target).T @ (source - mu_source) / len(source)
    u, _, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[2, 2] = -1.0
    rotation = u @ correction @ vt
    return rotation, mu_target - rotation @ mu_source


def backproject(depth_mm, pose, stride):
    """Camera-frame points for the sampled pixels, plus their grayscale later."""
    depth = depth_mm.astype(np.float32) / 1000.0
    rows = np.arange(0, pose["h"], stride)
    cols = np.arange(0, pose["w"], stride)
    grid_v, grid_u = np.meshgrid(rows, cols, indexing="ij")
    z = depth[grid_v, grid_u]
    valid = (z > 0.2) & (z < 60.0)
    if not valid.any():
        return None, None
    u = grid_u[valid].astype(np.float32)
    v = grid_v[valid].astype(np.float32)
    d = z[valid]
    points = np.stack([
        d,
        -(u - pose["cx"]) / pose["fx"] * d,
        -(v - pose["cy"]) / pose["fy"] * d,
    ], axis=1)
    return points, (u.astype(np.int32), v.astype(np.int32))


def merge_voxels(keys_list, sums_list, counts_list):
    """Average all observations falling in the same voxel."""
    if not keys_list:
        return np.zeros((0, 3), np.int64), np.zeros((0, 3), np.float64), np.zeros(0, np.int64)
    keys = np.concatenate(keys_list)
    sums = np.concatenate(sums_list)
    counts = np.concatenate(counts_list)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    total = np.zeros((len(unique), 3), dtype=np.float64)
    np.add.at(total, inverse, sums)
    count = np.zeros(len(unique), dtype=np.int64)
    np.add.at(count, inverse, counts)
    return unique, total / count[:, None], count


def fuse(session, poses, transforms, stride, voxel, chunk):
    """Accumulate voxel means in chunks; a 500-frame walk does not fit in one sort."""
    indices = sorted(transforms)
    keys_chunks, sums_chunks, counts_chunks = [], [], []
    pending_keys, pending_sums, pending_counts = [], [], []
    chunks = []
    raw_total = 0
    used = []

    for step, index in enumerate(indices):
        pose = poses[index]
        depth_mm = cv2.imread(str(session / "depth" / f"frame_{index:06d}.png"), cv2.IMREAD_UNCHANGED)
        if depth_mm is None:
            continue
        points, _ = backproject(depth_mm, pose, stride)
        if points is None:
            continue
        transform = transforms[index]
        world = points @ transform[:3, :3].T + transform[:3, 3]
        raw_total += len(world)
        used.append(index)

        cell = np.floor(world / voxel).astype(np.int64)
        unique, inverse = np.unique(cell, axis=0, return_inverse=True)
        frame_sums = np.zeros((len(unique), 3), dtype=np.float64)
        np.add.at(frame_sums, inverse, world)
        frame_counts = np.bincount(inverse, minlength=len(unique)).astype(np.int64)
        pending_keys.append(unique)
        pending_sums.append(frame_sums)
        pending_counts.append(frame_counts)

        if (step + 1) % chunk == 0:
            chunks.append(merge_voxels(pending_keys, pending_sums, pending_counts))
            pending_keys, pending_sums, pending_counts = [], [], []

    if pending_keys:
        chunks.append(merge_voxels(pending_keys, pending_sums, pending_counts))
    if not chunks:
        return np.zeros((0, 3)), np.zeros(0, np.int64), 0, []

    # Merging chunk summaries keeps every sort bounded: 500 frames of raw depth would
    # be tens of millions of points, while the summaries are a few hundred thousand.
    while len(chunks) > 1:
        merged = []
        for group in range(0, len(chunks), 4):
            batch = chunks[group:group + 4]
            merged.append(merge_voxels(
                [item[0] for item in batch],
                [item[1] * item[2][:, None] for item in batch],
                [item[2] for item in batch]))
        chunks = merged
    _, points, counts = chunks[0]
    return points, counts, raw_total, used


def label_metrics(points, landmarks, normals, radius=0.5):
    tree = cKDTree(points)
    distances, _ = tree.query(landmarks, k=1)
    covered = distances < radius

    # Same nearest-label rule, without an N_voxels x N_labels x 3 allocation.
    label_distance, nearest = cKDTree(landmarks).query(points)
    near = label_distance < 2.0
    perpendicular = np.abs(((points[near] - landmarks[nearest[near]])
                            * normals[nearest[near]]).sum(axis=1))

    return {
        "label_recall_0_5_m": round(float(covered.mean()), 3),
        "labels_covered": f"{int(covered.sum())}/{len(landmarks)}",
        "label_distance_m": {
            "median": round(float(np.median(distances)), 3),
            "p90": round(float(np.percentile(distances, 90)), 3),
        },
        "perpendicular_to_label_plane_m": {
            "median": round(float(np.median(perpendicular)), 3) if len(perpendicular) else None,
            "p90": round(float(np.percentile(perpendicular, 90)), 3) if len(perpendicular) else None,
        },
    }


def surface_thickness(points, samples=4000, radius=0.3, seed=0):
    """Local PCA residual: how thick a fused surface is where the poses disagree."""
    if len(points) < 100:
        return None
    generator = np.random.default_rng(seed)
    picks = generator.choice(len(points), size=min(samples, len(points)), replace=False)
    tree = cKDTree(points)
    residuals = []
    for pick in picks:
        neighbours = tree.query_ball_point(points[pick], radius)
        if len(neighbours) < 8:
            continue
        patch = points[neighbours] - points[neighbours].mean(axis=0)
        eigenvalues = np.linalg.eigvalsh(patch.T @ patch / len(patch))
        residuals.append(float(np.sqrt(max(eigenvalues[0], 0.0))))
    if not residuals:
        return None
    residuals = np.asarray(residuals)
    return {
        "median": round(float(np.median(residuals)), 3),
        "p90": round(float(np.percentile(residuals, 90)), 3),
        "samples": int(len(residuals)),
    }


def write_ply(path, points, counts):
    """Binary PLY so the map opens in any viewer; intensity from fusion redundancy."""
    intensity = np.clip(counts / max(1.0, np.percentile(counts, 95)) * 255.0, 0, 255).astype(np.uint8)
    vertex = np.zeros(len(points), dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    vertex["x"], vertex["y"], vertex["z"] = points[:, 0], points[:, 1], points[:, 2]
    vertex["red"] = vertex["green"] = vertex["blue"] = intensity
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    with path.open("wb") as handle:
        handle.write(header.encode("ascii"))
        vertex.tofile(handle)


def plot(path, points, landmarks, truth, title, floor=0.15):
    """Top view of the structures, plus the height histogram.

    The floor is dropped from the top view on purpose: it holds most of the points,
    and a top view of a dense floor is a rectangle that hides everything else. The
    saved cloud keeps every voxel."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    above = points[points[:, 2] >= floor]
    generator = np.random.default_rng(1)
    picks = generator.choice(len(above), size=min(150000, len(above)), replace=False)
    sample = above[picks]
    figure, axes = plt.subplots(1, 2, figsize=(13, 6))
    axes[0].scatter(sample[:, 0], sample[:, 1], c=sample[:, 2], s=0.4, cmap="viridis", alpha=0.6)
    axes[0].plot(truth[:, 0], truth[:, 1], color="red", linewidth=1.2, label="ground truth path")
    axes[0].scatter(landmarks[:, 0], landmarks[:, 1], marker="x", s=16, c="black",
                    linewidths=0.6, label="labels")
    axes[0].set_aspect("equal")
    axes[0].set_title(f"{title}\ntop view, z >= {floor} m ({len(sample)} of {len(points)} voxels)")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].hist(points[:, 2], bins=100, color="steelblue")
    axes[1].set_yscale("log")
    axes[1].set_title("height distribution (all voxels)")
    axes[1].set_xlabel("z [m]")
    axes[1].set_ylabel("voxels")
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)


def evaluate_native(session, out, source, points, counts, transforms, native_report):
    """Read truth only after fusion; these transforms never return to the algorithm."""
    from evaluate_slam import umeyama
    from colmap_depth_poses import pose_record, CV_TO_CAMERA

    poses = load_ground_truth(session)
    shared = sorted(set(transforms) & set(poses))
    if len(shared) < 10:
        raise ValueError(f"only {len(shared)} frames intersect ground truth")
    estimate = np.array([transforms[i][:3, 3] for i in shared])
    target = np.array([ground_truth_pose(poses[i])[:3, 3] for i in shared])
    rigid_r, rigid_t = rigid_align(estimate, target)
    sim_r, sim_t, sim_s = umeyama(estimate, target)
    rigid_errors = np.linalg.norm(estimate @ rigid_r.T + rigid_t - target, axis=1)
    sim_errors = np.linalg.norm(sim_s * (estimate @ sim_r.T) + sim_t - target, axis=1)
    # Primary COLMAP scoring follows the monocular Sim(3) protocol. Also report
    # rigid-only scores so depth-derived scale errors remain visible.
    if source == "colmap":
        rotation, translation, scale = sim_r, sim_t, sim_s
    elif source == "gt":
        rotation, translation, scale = np.eye(3), np.zeros(3), 1.
    else:
        rotation, translation, scale = rigid_r, rigid_t, 1.
    evaluated = scale * (points @ rotation.T) + translation
    alignment = dict(
        kind="Sim(3), evaluation only" if source == "colmap" else "SE(3), evaluation only",
        frames=len(shared), coverage=round(len(shared) / len(poses), 4),
        rmse_m=round(float(np.sqrt(np.mean((sim_errors if source == "colmap" else rigid_errors)**2))), 4),
        max_m=round(float((sim_errors if source == "colmap" else rigid_errors).max()), 4),
        rigid_rmse_m=round(float(np.sqrt(np.mean(rigid_errors**2))), 4),
        sim3_rmse_m=round(float(np.sqrt(np.mean(sim_errors**2))), 4),
        sim3_scale=float(sim_s),
        applied_scale=float(scale), rotation=rotation.tolist(), translation=translation.tolist(),
        ground_truth_used_for_fusion=(source == "gt"), applied_after_fusion=True)

    # Requested intermediate pose format, strictly an EVALUATION artifact.
    if source == "colmap":
        records = []
        calibration = native_report['pose_preparation']['depth_intrinsics']
        for i in sorted(transforms):
            transform = transforms[i]
            records.append(pose_record(i, rotation @ transform[:3, :3] @ CV_TO_CAMERA,
                                       scale * (rotation @ transform[:3, 3]) + translation,
                                       calibration))
        (out / "colmap_poses_eval_only.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in records), encoding="utf-8")
        (out / "colmap_poses_eval_only.json").write_text(json.dumps(dict(
            purpose="evaluation only; must never be used by reconstruction or fusion",
            ground_truth_used=True, alignment=alignment), indent=2), encoding="utf-8")

    landmarks_path = session / "landmarks.json"
    landmarks = np.asarray(json.loads(landmarks_path.read_text(encoding="utf-8"))["points"], dtype=float)
    landmarks = landmarks @ MIRROR / 100.
    meta = json.loads((session / "meta.json").read_text(encoding="utf-8"))
    origin = np.asarray(meta["start_pose"]["loc"]) @ MIRROR / 100.
    normals = origin[None, :] - landmarks
    normals[:, 2] = 0.
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-6)
    truth = np.array([ground_truth_pose(poses[i])[:3, 3] for i in sorted(poses)])
    report = dict(native_report)
    report.update(
        frames_of_truth=f"{len(shared)}/{len(poses)}",
        coverage=round(len(shared) / len(poses), 4),
        pose_alignment=None if source == "gt" else alignment,
        evaluation_frame="UE mirrored Y, metres",
        evaluation_alignment="oracle reference" if source == "gt" else alignment['kind'],
        ground_truth_usage="oracle fusion reference" if source == "gt" else "post-fusion evaluation only",
        extent_m=[round(float(v), 2) for v in np.ptp(evaluated, axis=0)])
    report.update(label_metrics(evaluated, landmarks, normals))
    # Thickness is measured in native metric units BEFORE any truth scale correction.
    report["surface_thickness_m"] = surface_thickness(points)
    report['surface_thickness_frame'] = 'native metric map, before any evaluation alignment'
    if source == "colmap":
        rigid_points = points @ rigid_r.T + rigid_t
        report['rigid_only_label_metrics'] = label_metrics(rigid_points, landmarks, normals)
    np.savez_compressed(out / "dense_map.npz", points=evaluated.astype(np.float32),
                        counts=counts.astype(np.int32))
    write_ply(out / "dense_map.ply", evaluated, counts)
    (out / "dense_eval.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot(out / "dense_preview.png", evaluated, landmarks, truth,
         f"{session.name}: {source}, coverage {len(shared)}/{len(poses)}")
    print(json.dumps(report, indent=2), flush=True)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--poses", choices=["gt", "vo", "colmap"], default="gt")
    parser.add_argument("--model", help="COLMAP model; default output/ue5-eval/colmap-<session>/best")
    parser.add_argument("--voxel", type=float, default=0.05, help="voxel edge in metres")
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--chunk", type=int, default=40)
    parser.add_argument("--out")
    parser.add_argument("--frame-indices", help="oracle gt control only: use used_frame_indices from a native map report; requires --out")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--fuse-only", action="store_true", help="save native map without reading any evaluation truth")
    modes.add_argument("--evaluate-only", action="store_true", help="score previously saved native map")
    modes.add_argument("--replot", action="store_true")
    args = parser.parse_args()
    if args.voxel <= 0 or args.stride < 1 or args.chunk < 1:
        parser.error("voxel, stride and chunk must be positive")
    if args.frame_indices and (args.poses != 'gt' or not args.out or args.evaluate_only or args.replot):
        parser.error('--frame-indices is only for a new explicitly named oracle gt control')
    session = Path(args.session).resolve()
    out = Path(args.out) if args.out else Path("output/ue5-eval") / f"dense-{session.name}-{args.poses}"
    out.mkdir(parents=True, exist_ok=True)

    if args.replot:
        poses = load_ground_truth(session)
        truth = np.array([ground_truth_pose(poses[i])[:3, 3] for i in sorted(poses)])
        landmarks = np.array(json.loads((session / "landmarks.json").read_text(encoding="utf-8"))["points"]) @ MIRROR / 100.
        saved = np.load(out / "dense_map.npz")
        plot(out / "dense_preview.png", saved["points"], landmarks, truth, f"{session.name}: {args.poses}")
        return 0

    if args.evaluate_only:
        native_report = json.loads((out / 'dense_native.json').read_text(encoding='utf-8'))
        if native_report['session'] != session.name or native_report['pose_source'] != args.poses:
            raise ValueError('saved native map session/source does not match request')
        saved = np.load(out / 'dense_native.npz')
        points, counts = saved['points'].astype(float), saved['counts']
        records = [json.loads(line) for line in (out / 'native_transforms.jsonl').read_text(encoding='utf-8').splitlines()]
        transforms = {row['i']: np.asarray(row['T']) for row in records}
    else:
        preparation = None
        matched_control = None
        if args.poses == "gt":
            poses = load_ground_truth(session)
            transforms = {i: pose_transform(p) for i, p in poses.items()}
        elif args.poses == "vo":
            from colmap_depth_poses import load_calibration
            transforms = load_vo_poses(session)
            calibration = load_calibration(session)
            poses = {i: calibration for i in transforms}
        else:
            from colmap_depth_poses import export_colmap_poses
            model = Path(args.model) if args.model else Path('output/ue5-eval') / f'colmap-{session.name}' / 'best'
            path, preparation = export_colmap_poses(session, model, out)
            poses = load_pose_records(path)
            transforms = {i: pose_transform(p) for i, p in poses.items()}
            print(json.dumps(preparation, indent=2), flush=True)
        if args.frame_indices:
            reference_path = Path(args.frame_indices).resolve()
            reference = json.loads(reference_path.read_text(encoding='utf-8'))
            selected = reference['used_frame_indices']
            if (not isinstance(selected, list) or not selected
                    or any(type(i) is not int for i in selected)
                    or len(set(selected)) != len(selected)):
                raise ValueError('reference frame indices must be a nonempty list of unique integers')
            if not set(selected).issubset(transforms):
                raise ValueError('reference frames are absent from the oracle session')
            transforms = {i: transforms[i] for i in sorted(selected)}
            matched_control = dict(reference=str(reference_path),
                                   pose_source=reference.get('pose_source', 'estimate'),
                                   frames=len(selected))
        registered = len(transforms)
        print(f"fusing {registered} {args.poses} poses in native frame", flush=True)
        points, counts, raw_total, used = fuse(session, poses, transforms, args.stride, args.voxel, args.chunk)
        if not used:
            raise ValueError('no usable depth frames')
        transforms = {i: transforms[i] for i in used}
        total_depth = len(list((session / 'depth').glob('frame_*.png')))
        native_report = dict(
            session=session.name, pose_source=args.poses, registered_frames=registered,
            frames_used=len(used), depth_frames_total=total_depth,
            depth_coverage=round(len(used) / max(1, total_depth), 4), used_frame_indices=used,
            voxel_m=args.voxel, pixel_stride=args.stride, chunk_frames=args.chunk,
            observations=int(raw_total), voxels=len(points),
            observations_per_voxel=round(float(counts.mean()), 2),
            voxels_with_3plus=round(float((counts >= 3).mean()), 3),
            ground_truth_used_for_fusion=(args.poses == 'gt'),
            fusion_frame='UE evaluation frame (oracle)' if args.poses == 'gt' else 'native metric reconstruction frame',
            pose_preparation=preparation)
        if matched_control is not None:
            native_report['matched_frame_control'] = matched_control
        # Native map and poses are complete and durable BEFORE any evaluation data
        # is opened. Evaluation-only mode never refits or re-fuses this artifact.
        np.savez_compressed(out / 'dense_native.npz', points=points.astype(np.float32), counts=counts.astype(np.int32))
        (out / 'native_transforms.jsonl').write_text(''.join(
            json.dumps(dict(i=i, T=t.tolist())) + "\n" for i, t in sorted(transforms.items())), encoding='utf-8')
        (out / 'dense_native.json').write_text(json.dumps(native_report, indent=2), encoding='utf-8')
        print(f"native fusion saved: {out}; frames {len(used)}/{total_depth}", flush=True)
    if not args.fuse_only:
        evaluate_native(session, out, args.poses, points, counts, transforms, native_report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
