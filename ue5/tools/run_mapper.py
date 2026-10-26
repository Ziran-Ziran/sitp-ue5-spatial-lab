"""Run only COLMAP's incremental mapper on an existing database.

    python ue5/tools/run_mapper.py output/ue5-eval/colmap-city-04/database.db \
        output/ue5-datasets/city-04/frames --out output/ue5-eval/map-city-04-base

Why this exists: feature extraction and matching for 260 frames take a few minutes
and are the same for every matching-graph experiment. Only the graph changes, so
only the mapper needs to rerun. Keeping this separate makes the comparisons in
`filter_match_graph.py` cheap and honest -- same features, same images, one
variable.

Writes <out>/best (binary model) and <out>/summary.json, the same layout the
evaluators expect, so `evaluate_slam.py` and `evaluate_landmarks.py` work unchanged.
"""

import argparse
import json
import shutil
import time
from pathlib import Path

import pycolmap


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("frames")
    parser.add_argument("--out", required=True)
    parser.add_argument("--label", default=None,
                        help="short name for the match graph, shown in the comparison table")
    parser.add_argument("--init-pair", type=int, nargs=2, default=None,
                        metavar=("IMG1", "IMG2"),
                        help="force the seed pair by image id; the default is COLMAP's automatic choice")
    parser.add_argument("--fixed-intrinsics", action="store_true",
                        help="keep input calibration fixed in both absolute-pose refinement and bundle adjustment")
    parser.add_argument("--random-seed", type=int, default=None,
                        help="seed both pipeline and mapper; omission preserves COLMAP defaults")
    args = parser.parse_args()

    database = Path(args.database).resolve()
    frames = Path(args.frames).resolve()
    out = Path(args.out).resolve()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    options = pycolmap.IncrementalPipelineOptions()
    options.num_threads = 8
    if args.random_seed is not None:
        options.random_seed = args.random_seed
        options.mapper.random_seed = args.random_seed
    if args.fixed_intrinsics:
        options.ba_refine_focal_length = False
        options.ba_refine_principal_point = False
        options.ba_refine_extra_params = False
        options.mapper.abs_pose_refine_focal_length = False
        options.mapper.abs_pose_refine_extra_params = False
    if args.init_pair:
        options.init_image_id1, options.init_image_id2 = args.init_pair

    start = time.time()
    models = pycolmap.incremental_mapping(database, frames, out / "sparse", options=options)
    mapping_s = round(time.time() - start, 1)
    print(f"mapping {mapping_s}s, {len(models)} model(s)", flush=True)
    if not models:
        print("no reconstruction produced")
        return 2

    best = max(models.values(), key=lambda model: model.num_reg_images())
    best_path = out / "best"
    best_path.mkdir(parents=True, exist_ok=True)
    best.write(best_path)

    total = len(list(frames.glob("*.png")))
    summary = {
        "database": str(database),
        "arm": args.label,
        "frames": total,
        "registered": best.num_reg_images(),
        "registered_fraction": round(best.num_reg_images() / max(1, total), 3),
        "points3d": best.num_points3D(),
        "init_pair": args.init_pair,
        "intrinsics_policy": "fixed" if args.fixed_intrinsics else "default refinement",
        "pipeline_random_seed": options.random_seed,
        "mapper_random_seed": options.mapper.random_seed,
        "calibration_refinement": {
            "ba_focal": options.ba_refine_focal_length,
            "ba_principal_point": options.ba_refine_principal_point,
            "ba_extra": options.ba_refine_extra_params,
            "absolute_pose_focal": options.mapper.abs_pose_refine_focal_length,
            "absolute_pose_extra": options.mapper.abs_pose_refine_extra_params,
        },
        "output_cameras": {str(k): str(v) for k, v in best.cameras.items()},
        "mapping_s": mapping_s,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
