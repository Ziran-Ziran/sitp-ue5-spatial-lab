"""Controlled bridge probe: fixed existing cameras, toggle two-view tracks.

No truth inputs, no BA, no re-matching, and no input model/database writes.
The saved models keep the original registration coverage; PnP is diagnostic only.
"""
from collections import defaultdict
import argparse
import json
from pathlib import Path
import sqlite3
import time

import numpy as np
import pycolmap

from diagnose_reconstruction import PAIR_BASE, digest, index


def support(rec, images, records):
    linked=defaultdict(lambda:defaultdict(set))
    for pair,count,config,data in records:
        if not count:
            continue
        a,b=divmod(pair,PAIR_BASE)
        matches=np.frombuffer(data,dtype=np.uint32).reshape(-1,2)
        for source,target,column in [(a,b,0),(b,a,1)]:
            if source not in rec.images or target in rec.images:
                continue
            for point,candidate in matches[:,[column,1-column]]:
                obs=rec.images[source].points2D[int(point)]
                if obs.has_point3D():
                    linked[target][int(candidate)].add(int(obs.point3D_id))
    return {i:{pixel:next(iter(points)) for pixel,points in pixels.items() if len(points)==1}
            for i,pixels in linked.items()}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--session',default='night-city-lap2-a')
    parser.add_argument('--out',required=True)
    parser.add_argument('--min-angle',type=float,default=1.5)
    parser.add_argument('--ignore-two-view',choices=['both','yes','no'],default='both')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    folder=root/'output/ue5-eval'/('colmap-'+args.session)
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    db=folder/'database.db';db_hash=digest(db)
    with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as c:
        images=dict(c.execute('SELECT image_id,name FROM images'))
        keypoints={i:np.frombuffer(data,dtype=np.float32).reshape(n,k)[:,:2].copy()
                   for i,n,k,data in c.execute('SELECT image_id,rows,cols,data FROM keypoints')}
        records=c.execute('SELECT pair_id,rows,config,data FROM two_view_geometries').fetchall()
    # Frozen-pose triangulation has no poses for missing images. Restrict this
    # graph to registered cameras; full DB edges remain available in support().
    registered_ids=set(pycolmap.Reconstruction(folder/'best').images)
    graph=pycolmap.CorrespondenceGraph()
    for i in registered_ids:
        graph.add_image(i,len(keypoints[i]))
    for pair,count,config,data in records:
        if count<15:
            continue
        a,b=divmod(pair,PAIR_BASE)
        if a not in registered_ids or b not in registered_ids:
            continue
        geometry=pycolmap.TwoViewGeometry()
        geometry.config=pycolmap.TwoViewGeometryConfiguration(config)
        geometry.inlier_matches=np.frombuffer(data,dtype=np.uint32).reshape(-1,2).copy()
        graph.add_two_view_geometry(a,b,geometry)
    graph.finalize()
    results=[]
    policies={'both':[True,False],'yes':[True],'no':[False]}[args.ignore_two_view]
    for ignore in policies:
        start=time.monotonic(); rec=pycolmap.Reconstruction(folder/'best')
        original_poses={i:im.cam_from_world().matrix().copy() for i,im in rec.images.items()}
        original_cameras={i:c.params.copy() for i,c in rec.cameras.items()}
        before=rec.num_points3D(); before_support=support(rec,images,records)
        manager=pycolmap.ObservationManager(rec,graph)
        triangulator=pycolmap.IncrementalTriangulator(graph,rec,manager)
        options=pycolmap.IncrementalTriangulatorOptions()
        options.ignore_two_view_tracks=ignore;options.random_seed=0
        options.min_angle=args.min_angle
        added=sum(triangulator.triangulate_image(options,i) for i in sorted(rec.images))
        completed=triangulator.complete_all_tracks(options)
        after_support=support(rec,images,records)
        assert all(np.array_equal(im.cam_from_world().matrix(),original_poses[i]) for i,im in rec.images.items())
        assert all(np.array_equal(c.params,original_cameras[i]) for i,c in rec.cameras.items())
        candidates=sorted(after_support,key=lambda i:-len(after_support[i]))[:12]
        probes=[]
        for candidate in candidates:
            matches=after_support[candidate]
            pixels=sorted(matches)
            points2d=np.array([keypoints[candidate][p] for p in pixels],dtype=float)
            points3d=np.array([rec.points3D[matches[p]].xyz for p in pixels],dtype=float)
            camera=next(iter(rec.cameras.values()))
            camera=pycolmap.Camera(model=camera.model,width=camera.width,height=camera.height,params=camera.params.copy())
            estimation=pycolmap.AbsolutePoseEstimationOptions()
            estimation.ransac.min_inlier_ratio=0.25;estimation.ransac.random_seed=0
            refinement=pycolmap.AbsolutePoseRefinementOptions()
            refinement.refine_focal_length=True;refinement.refine_extra_params=True
            pose=pycolmap.estimate_and_refine_absolute_pose(points2d,points3d,camera,estimation,refinement) if len(pixels)>=4 else None
            inliers=pose['num_inliers'] if pose else 0
            probes.append(dict(frame=index(images[candidate]),before_pixels=len(before_support.get(candidate,{})),
                               pixels=len(pixels),unique_3d=len(set(matches.values())),pnp_inliers=inliers,
                               meets_default_30_inliers=inliers>=30))
        name='ignore_two_view' if ignore else 'include_two_view'
        target=out/name;target.mkdir();rec.write(target)
        row=dict(policy=name,ignore_two_view_tracks=ignore,registered=rec.num_reg_images(),total=len(images),
                 min_angle_deg=options.min_angle,
                 cameras_and_poses_exactly_unchanged=True,points_before=before,points_after=rec.num_points3D(),
                 added_observations=added,completed_observations=completed,top_candidates=probes,
                 elapsed_s=time.monotonic()-start,ground_truth_read=False)
        results.append(row);(out/'progress.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
        print(json.dumps(row,indent=2),flush=True)
    assert digest(db)==db_hash
    (out/'result.json').write_text(json.dumps(dict(results=results,input_database_sha_unchanged=True),indent=2),encoding='utf-8')


if __name__=='__main__':
    main()
