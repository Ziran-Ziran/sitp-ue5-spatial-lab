"""Cut pairs out of a COLMAP match graph, and report what the graph contained.

    python ue5/tools/filter_match_graph.py output/ue5-datasets/city-04 \
        --database output/ue5-eval/colmap-city-04/database.db \
        --out output/ue5-eval/db-city-04-sequential --overlap 40

Exhaustive matching answers one question honestly -- "can these two images be
matched?" -- and a second one dishonestly -- "are they the same place?". In a street
of near-identical buildings the two answers differ, and every wrong edge that still
fits an essential matrix will bend the map.

So this tool builds a controlled comparison. You state which edges a system is
*allowed* to use:

    --overlap 40            consecutive frames, the odometry backbone
    --oracle-distance 2.5   every pair whose ground-truth camera centres are within
                            2.5 m -- cheating, but it is the ceiling
    --loop-edges FILE       pairs from place-recognition output

Everything else is deleted from a copy of the database. Run the mapper on each
variant and compare: the difference is the cost of removing edges, or the value of
adding them.

The report classifies every far pair (further apart than --overlap) by ground-truth
distance, which separates the two kinds and their very different fixes:

    real      far in time, close in space: a loop edge, the information we want
    aliasing  far in time and far in space: look-alike geometry, must be rejected
"""

import argparse
import json
import shutil
from pathlib import Path

import pycolmap

# COLMAP encodes a pair as image_id1 * kMaxNumImages + image_id2, kMaxNumImages = 2^31 - 1.
K_MAX_IMAGE_ID = 2147483647


def load_poses(session):
    poses = {}
    for line in (session / "poses.jsonl").read_text(encoding="utf-8").splitlines():
        if line.strip():
            pose = json.loads(line)
            poses[pose["i"]] = pose
    return poses


def frame_index(name):
    return int(Path(name).stem.split("_")[1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session")
    parser.add_argument("--database", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--overlap", type=int, default=40)
    parser.add_argument("--oracle-distance", type=float, default=None,
                        help="keep every pair closer than this in ground-truth metres")
    parser.add_argument("--loop-edges", default=None, help="JSON list of [frame_a, frame_b]")
    parser.add_argument("--min-matches", type=int, default=0,
                        help="also drop pairs with fewer raw matches than this")
    args = parser.parse_args()

    session = Path(args.session).resolve()
    poses = load_poses(session)
    source = Path(args.database).resolve()
    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.unlink(missing_ok=True)
    shutil.copy2(source, out)

    loop_edges = set()
    if args.loop_edges:
        for a, b in json.loads(Path(args.loop_edges).read_text(encoding="utf-8")):
            loop_edges.add((min(a, b), max(a, b)))

    db = pycolmap.Database.open(out)
    images = db.read_all_images()
    images = images.values() if isinstance(images, dict) else images
    id_to_index = {image.image_id: frame_index(image.name) for image in images}
    pair_ids, pair_counts = db.read_num_matches()
    match_counts = dict(zip(pair_ids, pair_counts))

    # One pass over the pairs that actually have matches: decide, act, and record.
    kept = dropped = 0
    far = []          # every pair beyond the odometry window that had matches
    dropped_far = 0
    dropped_far_matches = 0

    for pair_id in sorted(match_counts):
        num = match_counts[pair_id]
        id1, id2 = divmod(pair_id, K_MAX_IMAGE_ID)
        if id1 not in id_to_index or id2 not in id_to_index:
            continue
        i1, i2 = id_to_index[id1], id_to_index[id2]
        gap = abs(i1 - i2)
        in_window = gap <= args.overlap
        in_loops = (min(i1, i2), max(i1, i2)) in loop_edges
        distance = None
        if gap > args.overlap:
            # poses.jsonl stores UE world coordinates verbatim: centimetres, left-handed.
            p1, p2 = poses[i1]["loc"], poses[i2]["loc"]
            distance = round(0.01 * ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2
                                     + (p1[2] - p2[2]) ** 2) ** 0.5, 2)
            far.append({"frames": [i1, i2], "matches": num, "distance_m": distance})

        keep = in_window or in_loops
        if not keep and args.oracle_distance is not None:
            keep = distance is not None and distance <= args.oracle_distance
        if keep and args.min_matches and num < args.min_matches:
            keep = False

        if keep:
            kept += 1
            continue

        dropped += 1
        if gap > args.overlap:
            dropped_far += 1
            dropped_far_matches += num
        db.delete_matches(id1, id2)
        db.delete_two_view_geometry(id1, id2)

    report = {
        "session": session.name,
        "database": str(out),
        "overlap": args.overlap,
        "oracle_distance_m": args.oracle_distance,
        "loop_edges_supplied": len(loop_edges),
        "kept_pairs": kept,
        "dropped_pairs": dropped,
        "dropped_far_pairs": dropped_far,
        "dropped_far_matches": dropped_far_matches,
        "far_pairs_before_filter": len(far),
        "far_pairs_over_100_matches": sum(1 for r in far if r["matches"] >= 100),
        "real_loop_pairs": sorted([r for r in far if r["distance_m"] <= 2.5],
                                  key=lambda r: -r["matches"]),
        "aliasing_pairs": sorted([r for r in far if r["distance_m"] > 2.5],
                                 key=lambda r: -r["matches"]),
        "surviving_matched_pairs": db.num_matched_image_pairs(),
    }
    db.close()

    out_json = out.with_suffix(".graph.json")
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    real = report["real_loop_pairs"]
    alias = report["aliasing_pairs"]
    print(f"kept {kept} pairs, dropped {dropped} "
          f"({dropped_far} of them beyond the {args.overlap}-frame window, {dropped_far_matches} matches)")
    print(f"far pairs with matches: {len(far)}, of those >=100 matches: {report['far_pairs_over_100_matches']}")
    print(f"\nreal loop pairs (same place, far in time): {len(real)}")
    for row in real[:10]:
        print(f"  {row['frames']} {row['distance_m']:5.2f} m {row['matches']:5d} matches")
    print(f"\naliasing pairs (different place, matched anyway): {len(alias)}")
    for row in alias[:10]:
        print(f"  {row['frames']} {row['distance_m']:6.1f} m {row['matches']:5d} matches")
    print(f"\ndatabase -> {out}")
    print(f"report   -> {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
