"""Run offline structure-from-motion on one recorded session.

    python ue5/tools/run_colmap.py output/ue5-datasets/circle

COLMAP (through pycolmap) is the data-validity check, not the final SLAM system:
it has no notion of "now", so a bad intrinsic or a broken timestamp shows up as a
failed registration instead of a mysterious tracking loss at frame 40.

Outputs under output/ue5-eval/colmap-<session>/:
    database.db   features and matches
    sparse/       incremental models written by pycolmap
    best/         the largest reconstruction, binary format
    summary.json  registered images, points, timings
"""

import argparse
import json
import math
import shutil
import time
from pathlib import Path

import pycolmap


def read_intrinsics(session: Path):
    """Independent metadata only; never open the ground-truth pose file."""
    from colmap_depth_poses import load_calibration
    camera = load_calibration(session)
    return tuple(camera[key] for key in ('fx', 'fy', 'cx', 'cy', 'w', 'h'))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--out", default=None)
    parser.add_argument("--max-image-size", type=int, default=1280)
    # Consecutive frames are only ~15 cm apart while the walls are ~10 m away, so a
    # 10-frame window has a triangulation angle below the initialisation threshold.
    # Matching across a wider window is what makes this scene reconstructable at all.
    parser.add_argument("--overlap", type=int, default=40)
    parser.add_argument("--peak-threshold", type=float, default=0.004)
    parser.add_argument("--exhaustive", action="store_true",
                        help="match every pair; needed for closed loops because sequential windows never see the start again")
    args = parser.parse_args()

    session = Path(args.session).resolve()
    frames = session / "frames"
    if not frames.is_dir():
        print(f"missing {frames}")
        return 1

    fx, fy, cx, cy, width, height = read_intrinsics(session)
    out = Path(args.out) if args.out else Path("output/ue5-eval") / f"colmap-{session.name}"
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    db = out / "database.db"

    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "PINHOLE"
    reader.camera_params = f"{fx:.6f},{fy:.6f},{cx:.6f},{cy:.6f}"

    extraction = pycolmap.FeatureExtractionOptions()
    extraction.use_gpu = False
    extraction.max_image_size = args.max_image_size
    extraction.sift.peak_threshold = args.peak_threshold
    extraction.sift.max_num_features = 8192

    timings = {}
    start = time.time()
    pycolmap.extract_features(
        db, frames,
        camera_mode=pycolmap.CameraMode.SINGLE,
        reader_options=reader,
        extraction_options=extraction)
    timings["features_s"] = round(time.time() - start, 1)
    print(f"[1/3] features {timings['features_s']}s", flush=True)

    pairing = pycolmap.SequentialPairingOptions()
    pairing.overlap = args.overlap
    pairing.loop_detection = False
    matching = pycolmap.FeatureMatchingOptions()
    matching.use_gpu = False

    start = time.time()
    if args.exhaustive:
        pycolmap.match_exhaustive(db, matching_options=matching)
    else:
        pycolmap.match_sequential(db, matching_options=matching, pairing_options=pairing)
    timings["matching_s"] = round(time.time() - start, 1)
    print(f"[2/3] matching {timings['matching_s']}s", flush=True)

    options = pycolmap.IncrementalPipelineOptions()
    options.num_threads = 8

    start = time.time()
    models = pycolmap.incremental_mapping(db, frames, out / "sparse", options=options)
    timings["mapping_s"] = round(time.time() - start, 1)
    print(f"[3/3] mapping {timings['mapping_s']}s, {len(models)} model(s)", flush=True)

    if not models:
        print("no reconstruction produced")
        return 2

    best = max(models.values(), key=lambda model: model.num_reg_images())
    best_path = out / "best"
    if best_path.exists():
        shutil.rmtree(best_path)
    best_path.mkdir(parents=True, exist_ok=True)
    best.write(best_path)

    summary = {
        "session": session.name,
        "matching": "exhaustive" if args.exhaustive else f"sequential-{args.overlap}",
        "images_total": len(list(frames.glob("*.png"))),
        "registered": best.num_reg_images(),
        "registered_fraction": round(best.num_reg_images() / max(1, len(list(frames.glob("*.png")))), 3),
        "points3d": best.num_points3D(),
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "w": width, "h": height},
        "timings": timings,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
