"""Read-only graph/model diagnosis. Never reads poses.jsonl or landmarks.

    python ue5/tools/diagnose_reconstruction.py --out NEW_DIRECTORY

Records verified graph connectivity, missing-frame boundaries and existing
3D-to-2D correspondence support. It does not modify any input database/model.
"""
from collections import Counter, defaultdict
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
import pycolmap

PAIR_BASE = 2147483647


def index(name):
    return int(Path(name).stem.rsplit('_', 1)[-1])


def segments(ids):
    result = []
    for value in sorted(ids):
        if result and value == result[-1][1]+1:
            result[-1][1] = value
        else:
            result.append([value, value])
    return result


def components(ids, edges, minimum):
    adjacency = defaultdict(set)
    for a, b, count, *_ in edges:
        if count >= minimum:
            adjacency[a].add(b)
            adjacency[b].add(a)
    unseen = set(ids)
    result = []
    while unseen:
        todo = [unseen.pop()]
        current = []
        while todo:
            node = todo.pop()
            current.append(node)
            neighbors = adjacency[node] & unseen
            unseen -= neighbors
            todo.extend(neighbors)
        result.append(dict(size=len(current), segments=segments(current)))
    return sorted(result, key=lambda r: -r['size'])


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def analyze(root, session):
    folder = root/'output/ue5-eval'/('colmap-'+session)
    db = folder/'database.db'
    before = digest(db)
    reconstruction = pycolmap.Reconstruction(folder/'best')
    with sqlite3.connect(db.resolve().as_uri()+'?mode=ro', uri=True) as connection:
        images = dict(connection.execute('SELECT image_id,name FROM images'))
        raw = dict(connection.execute('SELECT pair_id,rows FROM matches'))
        records = connection.execute('SELECT pair_id,rows,config,data FROM two_view_geometries').fetchall()
    ids = {key: index(name) for key,name in images.items()}
    edges = []
    correspondence = defaultdict(lambda: defaultdict(set))
    for pair, count, config, data in records:
        a, b = divmod(pair, PAIR_BASE)
        ia, ib = ids[a], ids[b]
        edges.append((ia, ib, count, config, raw.get(pair, 0)))
        if not count:
            continue
        matches = np.frombuffer(data, dtype=np.uint32).reshape(-1, 2)
        for source,target,column in [(a,b,0),(b,a,1)]:
            if source not in reconstruction.images or target in reconstruction.images:
                continue
            image = reconstruction.images[source]
            for point2d, candidate2d in matches[:, [column,1-column]]:
                point = image.points2D[int(point2d)]
                if point.has_point3D():
                    correspondence[ids[target]][int(candidate2d)].add(int(point.point3D_id))
    registered = {index(im.name) for im in reconstruction.images.values()}
    cross = [e for e in edges if ((e[0] in registered) != (e[1] in registered))]
    candidates = []
    for missing in sorted(set(ids.values())-registered):
        incident = [e for e in cross if missing in e[:2] and e[2]>0]
        points = correspondence[missing]
        unambiguous = {pixel:next(iter(values)) for pixel,values in points.items() if len(values)==1}
        candidates.append(dict(frame=missing, verified_edges_to_model=len(incident),
                               max_pair_inliers=max((e[2] for e in incident),default=0),
                               sum_pair_inliers=sum(e[2] for e in incident),
                               pixels_with_3d=len(points), unambiguous_pixels_with_3d=len(unambiguous),
                               unique_3d=len(set(unambiguous.values()))))
    models = []
    for model_path in sorted((folder/'sparse').iterdir()):
        if not (model_path/'images.bin').exists():
            continue
        rec = pycolmap.Reconstruction(model_path)
        models.append(dict(path=str(model_path.relative_to(root)),registered=rec.num_reg_images(),
                           segments=segments(index(im.name) for im in rec.images.values()),
                           cameras={str(k):list(c.params) for k,c in rec.cameras.items()}))
    report = dict(session=session, ground_truth_read=False, total=len(images),
                  registered=len(registered),coverage=len(registered)/len(images),
                  registered_segments=segments(registered), models=models,
                  raw_pairs=len(raw), geometry_configs=dict(Counter(e[3] for e in edges)),
                  pair_gaps=dict(sorted(Counter(abs(e[0]-e[1]) for e in edges).items())),
                  connected_components={str(n):components(ids.values(),edges,n) for n in [15,30,100]},
                  cross_model_edges={str(n):sum(e[2]>=n for e in cross) for n in [15,30,100]},
                  missing_candidates=candidates,
                  strongest_cross_edges=[dict(frame_a=e[0],frame_b=e[1],inliers=e[2],config=e[3],raw=e[4])
                                         for e in sorted(cross,key=lambda e:-e[2])[:30]],
                  db_sha256_before=before,db_sha256_after=digest(db))
    assert report['db_sha256_before']==report['db_sha256_after']
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',required=True)
    args=parser.parse_args()
    out=Path(args.out).resolve()
    out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[2]
    for session in ['city-lap2','night-city-lap2-a','night-city-lap2-b']:
        report=analyze(root,session)
        (out/(session+'.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({k:report[k] for k in ['session','registered_segments','pair_gaps',
                          'cross_model_edges','connected_components']},indent=2),flush=True)


if __name__=='__main__':
    main()
