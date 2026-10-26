"""Metric COLMAP poses using sparse tracks and optical-axis depth, never truth.

The JSONL uses the recorder's loc (cm, mirrored Y) / quat (xyzw) encoding,
but its WORLD frame is the native COLMAP frame in metres, not the UE world.
The sidecar records that distinction. No evaluation alignment is imported here.
"""
import json
import re
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

MIRROR = np.diag([1., -1., 1.])
CV_TO_CAMERA = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def load_calibration(session):
    """Recorder calibration from capture metadata, with no pose/truth fields."""
    meta = json.loads((session / 'meta.json').read_text(encoding='utf-8'))
    if meta.get('calibration_constant') is False:
        raise ValueError('capture intrinsics changed; independent per-frame calibration is required')
    width, height = int(meta['width']), int(meta['height'])
    fov = float(meta['fov_degrees_initial'])
    if not (0 < fov < 180 and width > 0 and height > 0):
        raise ValueError('invalid capture calibration')
    if 'calibration' in meta:
        calibration = dict(meta['calibration'])
        required = ('fx', 'fy', 'cx', 'cy', 'w', 'h')
        if not all(k in calibration and np.isfinite(calibration[k]) for k in required):
            raise ValueError('invalid independent capture calibration')
        if (calibration['fx'] <= 0 or calibration['fy'] <= 0
                or calibration['w'] != width or calibration['h'] != height):
            raise ValueError('invalid calibration focal length or image dimensions')
        calibration['fov'] = fov
        return {k: calibration[k] for k in (*required, 'fov')}
    # Recorder uses square pixels and horizontal FOV; see VisionRecorderComponent.
    focal = width / (2 * np.tan(np.deg2rad(fov) / 2))
    return dict(fx=float(focal), fy=float(focal), cx=width / 2,
                cy=height / 2, w=width, h=height, fov=fov)


def fit_depth_scale(predicted_axis, measured_m):
    """Median ratio is robust to foreground/background depth mismatches.

    Samples must have positive camera Z in reconstruction units. Range clipping
    applies only to measured (metric) depth; COLMAP units are arbitrary.
    """
    predicted = np.asarray(predicted_axis, dtype=float)
    measured = np.asarray(measured_m, dtype=float)
    valid = (np.isfinite(predicted) & np.isfinite(measured)
             & (predicted > 1e-9) & (measured > .2) & (measured < 60))
    ratios = measured[valid] / predicted[valid]
    if len(ratios) < 30:
        raise ValueError(f'only {len(ratios)} usable depth correspondences for scale')
    scale = float(np.median(ratios))
    residual = np.abs(ratios / scale - 1)
    inliers = residual < .05
    return scale, dict(samples=int(len(ratios)), inlier_fraction=float(inliers.mean()),
                       relative_residual_median=float(np.median(residual)),
                       ratio_p10_p90=[float(v) for v in np.percentile(ratios, [10, 90])])


def pose_record(index, world_from_cv, centre, calibration):
    """Encode a right-handed native-world pose in the recorder field convention."""
    world_from_camera = world_from_cv @ CV_TO_CAMERA.T
    ue_rotation = MIRROR @ world_from_camera @ MIRROR
    return dict(i=index, loc=(MIRROR @ centre * 100).tolist(),
                quat=Rotation.from_matrix(ue_rotation).as_quat().tolist(), **calibration)


def export_colmap_poses(session, model_path, out):
    """Build and save metric poses. Only model, metadata and depth are read."""
    import pycolmap

    reconstruction = pycolmap.Reconstruction(str(model_path))
    calibration = load_calibration(session)
    predicted, measured, frame_medians, entries = [], [], [], []
    depth_frame_count = 0
    for image in sorted(reconstruction.images.values(), key=lambda im: im.name):
        match = re.fullmatch(r'frame_(\d+)\.png', image.name)
        if match is None:
            continue
        index = int(match.group(1))
        pose = image.cam_from_world
        pose = pose() if callable(pose) else pose
        matrix = np.asarray(pose.matrix(), dtype=float)
        world_from_cv = matrix[:, :3].T
        centre = -world_from_cv @ matrix[:, 3]
        entries.append((index, world_from_cv, centre))
        depth = cv2.imread(str(session / 'depth' / image.name), cv2.IMREAD_UNCHANGED)
        if depth is None:
            continue
        if depth.ndim != 2 or depth.shape != (calibration['h'], calibration['w']):
            raise ValueError(f'depth dimensions mismatch: {image.name}')
        depth_frame_count += 1
        points = [p for p in image.points2D if p.has_point3D()]
        # Equal cap per image, deterministic coverage of the feature list.
        picks = np.linspace(0, len(points) - 1, min(256, len(points)), dtype=int)
        frame_ratios = []
        for pick in picks:
            point2d = points[pick]
            u, v = np.rint(point2d.xy).astype(int)
            if not (1 <= u < depth.shape[1] - 1 and 1 <= v < depth.shape[0] - 1):
                continue
            patch = depth[v-1:v+2, u-1:u+2].astype(float) / 1000
            d = float(np.median(patch))
            # Skip depth discontinuities: the sparse feature may sit on either side.
            if d <= .2 or d >= 60 or np.ptp(patch) > max(.05, .02 * d):
                continue
            xyz = np.asarray(reconstruction.points3D[point2d.point3D_id].xyz)
            z = float((matrix[:, :3] @ xyz + matrix[:, 3])[2])
            if z <= 1e-9:
                continue
            predicted.append(z)
            measured.append(d)
            frame_ratios.append(d / z)
        if len(frame_ratios) >= 30:
            frame_medians.append(float(np.median(frame_ratios)))

    scale, stats = fit_depth_scale(predicted, measured)
    if len(entries) < 10 or len(frame_medians) < 10:
        raise ValueError('need at least 10 registered depth frames with scale support')
    # One vote per supported image avoids texture-rich frames dominating the result.
    scale = float(np.median(frame_medians))
    relative = np.abs(np.asarray(measured) / np.asarray(predicted) / scale - 1)
    stats.update(inlier_fraction=float((relative < .05).mean()),
                 relative_residual_median=float(np.median(relative)),
                 supported_frames=len(frame_medians), depth_frames=depth_frame_count)
    if stats['inlier_fraction'] < .5:
        raise ValueError(f'depth scale lacks majority support: {stats}')

    # Fix a native origin from reconstruction alone, to keep voxel coordinates small.
    origin = entries[0][2]
    records = [pose_record(i, r, scale * (c - origin), calibration)
               for i, r, c in entries]
    out.mkdir(parents=True, exist_ok=True)
    path = out / 'colmap_poses.jsonl'
    path.write_text(''.join(json.dumps(row) + '\n' for row in records), encoding='utf-8')
    report = dict(model=str(model_path.resolve()), pose_file=str(path.resolve()),
                  world_frame='native COLMAP, first registered camera centre as origin',
                  field_encoding='loc: mirrored Y centimetres; quat: mirror @ R @ mirror xyzw',
                  metric_scale_source='depth / sparse point camera Z, median of frame medians',
                  metres_per_colmap_unit=scale, scale_fit=stats,
                  native_origin_colmap_units=origin.tolist(),
                  registered_frames=len(records), ground_truth_used=False,
                  depth_intrinsics=calibration,
                  reconstruction_cameras={str(k): str(v) for k, v in reconstruction.cameras.items()})
    (out / 'colmap_poses.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return path, report
