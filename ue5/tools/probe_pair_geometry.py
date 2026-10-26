"""Re-estimate existing cross-boundary pairs using independent DB calibration.

Read-only inputs; no new matching, no truth, no model selection or DB writes.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import time

import numpy as np
import pycolmap

from diagnose_reconstruction import PAIR_BASE,digest,index


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);args=parser.parse_args()
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[2]
    for session in ['city-lap2','night-city-lap2-a','night-city-lap2-b']:
        start=time.monotonic();db=root/'output/ue5-eval'/('colmap-'+session)/'database.db';sha=digest(db)
        with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as c:
            images={i:dict(name=name,camera=cam) for i,name,cam in c.execute('SELECT image_id,name,camera_id FROM images')}
            keypoints={i:np.frombuffer(data,dtype=np.float32).reshape(n,k)[:,:2].copy()
                       for i,n,k,data in c.execute('SELECT image_id,rows,cols,data FROM keypoints')}
            cameras={i:pycolmap.Camera(camera_id=i,model=pycolmap.CameraModelId(model),width=w,height=h,
                                      params=np.frombuffer(params,dtype=np.float64).copy(),has_prior_focal_length=bool(prior))
                     for i,model,w,h,params,prior in c.execute('SELECT camera_id,model,width,height,params,prior_focal_length FROM cameras')}
            pairs=[]
            def half(frame):return frame<=123 or 240<=frame<=363
            for pair,n,config,data in c.execute('SELECT pair_id,rows,config,data FROM two_view_geometries WHERE rows>=100'):
                a,b=divmod(pair,PAIR_BASE);ia,ib=index(images[a]['name']),index(images[b]['name'])
                if half(ia)!=half(ib):pairs.append((pair,n,config,data,a,b,ia,ib))
        rows=[]
        for pair,n,config,data,a,b,ia,ib in sorted(pairs,key=lambda r:-r[1])[:24]:
            options=pycolmap.TwoViewGeometryOptions();options.compute_relative_pose=True;options.min_num_inliers=100
            options.ransac.random_seed=0
            geometry=pycolmap.estimate_two_view_geometry(cameras[images[a]['camera']],keypoints[a],
                    cameras[images[b]['camera']],keypoints[b],np.frombuffer(data,dtype=np.uint32).reshape(-1,2).copy(),options)
            rows.append(dict(image_ids=[a,b],frames=[ia,ib],original_inliers=n,original_config=config,
                             reestimated_config=int(geometry.config),reestimated_inliers=len(geometry.inlier_matches),
                             triangulation_angle_deg=float(np.degrees(geometry.tri_angle)),
                             relative_pose_found=geometry.cam2_from_cam1 is not None))
        assert digest(db)==sha
        result=dict(session=session,cut='same frame cut for comparison: 0..123 and 240..363 versus other frames',
                    candidates=len(pairs),rows=rows,ground_truth_read=False,database_sha_unchanged=True,
                    elapsed_s=time.monotonic()-start)
        (out/(session+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
