"""Diagnose fixed boundary matches with RGB-D rigid SE(3), no truth or scale fit."""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
import pycolmap

from colmap_depth_poses import load_calibration
from diagnose_reconstruction import PAIR_BASE,digest,index


def rigid(source,target):
    a=source-source.mean(0);b=target-target.mean(0)
    u,_,vt=np.linalg.svd(b.T@a);correction=np.eye(3)
    correction[2,2]=np.linalg.det(u@vt)
    rotation=u@correction@vt
    return rotation,target.mean(0)-rotation@source.mean(0)


def points(keypoints,depth,camera):
    result=[]
    for u,v in keypoints:
        x,y=int(round(float(u))),int(round(float(v)))
        if x<1 or y<1 or x>=depth.shape[1]-1 or y>=depth.shape[0]-1:
            result.append([np.nan]*3);continue
        patch=depth[y-1:y+2,x-1:x+2].astype(float)/1000;z=float(np.median(patch))
        if z<.2 or z>60 or np.ptp(patch)>max(.05,.02*z):
            result.append([np.nan]*3);continue
        result.append([(u-camera['cx'])/camera['fx']*z,(v-camera['cy'])/camera['fy']*z,z])
    return np.array(result)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);args=parser.parse_args()
    out=Path(args.out);assert not out.exists()
    root=Path(__file__).resolve().parents[2];rows=[]
    for session_name in ['city-lap2','night-city-lap2-a','night-city-lap2-b']:
        session=root/'output/ue5-datasets'/session_name;camera=load_calibration(session)
        db=root/'output/ue5-eval'/('colmap-'+session_name)/'database.db';sha=digest(db)
        with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as c:
            images={index(n):(i,n) for i,n in c.execute('SELECT image_id,name FROM images')}
            for fa,fb in [(122,123),(123,124),(124,125),(238,240),(239,240),(362,364),(363,364),(363,379)]:
                ia,na=images[fa];ib,nb=images[fb];a,b=sorted([ia,ib]);pair=a*PAIR_BASE+b
                row=c.execute('SELECT rows,data FROM two_view_geometries WHERE pair_id=?',(pair,)).fetchone()
                if row is None or row[0]<3:continue
                matches=np.frombuffer(row[1],dtype=np.uint32).reshape(-1,2)
                if ia>ib:matches=matches[:,::-1]
                key=[]
                for im,col in [(ia,0),(ib,1)]:
                    n,k,data=c.execute('SELECT rows,cols,data FROM keypoints WHERE image_id=?',(im,)).fetchone()
                    key.append(np.frombuffer(data,dtype=np.float32).reshape(n,k)[matches[:,col],:2])
                source=points(key[0],cv2.imread(str(session/'depth'/na),cv2.IMREAD_UNCHANGED),camera)
                target=points(key[1],cv2.imread(str(session/'depth'/nb),cv2.IMREAD_UNCHANGED),camera)
                valid=np.isfinite(source).all(1)&np.isfinite(target).all(1);source=source[valid];target=target[valid]
                if len(source)<6:continue
                rng=np.random.default_rng(0);best=np.zeros(len(source),dtype=bool)
                for _ in range(1000):
                    picks=rng.choice(len(source),3,replace=False)
                    rotation,translation=rigid(source[picks],target[picks])
                    residual=np.linalg.norm(source@rotation.T+translation-target,axis=1);mask=residual<.1
                    if mask.sum()>best.sum():best=mask
                if best.sum()<3:continue
                rotation,translation=rigid(source[best],target[best]);residual=np.linalg.norm(source@rotation.T+translation-target,axis=1)
                # Localization diagnostic: measured source-camera depth supplies
                # the 3D anchors that are absent in the monocular sparse model.
                calibrated=pycolmap.Camera(model='PINHOLE',width=camera['w'],height=camera['h'],
                                          params=[camera[k] for k in ['fx','fy','cx','cy']])
                estimation=pycolmap.AbsolutePoseEstimationOptions()
                estimation.ransac.min_inlier_ratio=.25;estimation.ransac.random_seed=0
                pose=pycolmap.estimate_and_refine_absolute_pose(key[1][valid].astype(float),source,
                                                              calibrated,estimation)
                rays_a=source/np.linalg.norm(source,axis=1)[:,None]
                rays_b=(target/np.linalg.norm(target,axis=1)[:,None])@rotation
                angles=np.degrees(np.arccos(np.clip((rays_a*rays_b).sum(1),-1,1)))
                rows.append(dict(session=session_name,frames=[fa,fb],verified_matches=row[0],depth_supported=len(source),
                                 rigid_inliers_2cm=int((residual<.02).sum()),rigid_inliers_5cm=int((residual<.05).sum()),
                                 rigid_inliers_10cm=int((residual<.1).sum()),translation_norm_m=float(np.linalg.norm(translation)),
                                 rotation_deg=float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1)))),
                                 median_parallax_deg=float(np.median(angles[residual<.1])),
                                 depth_anchor_pnp_inliers=int(pose['num_inliers']) if pose else 0,
                                 depth_anchor_pnp_passes_30=bool(pose and pose['num_inliers']>=30),
                                 diagnostic_only_not_added_to_reconstruction=True,
                                 method='depth backprojection + rigid SE(3) RANSAC, scale fixed 1; no truth'))
        assert digest(db)==sha
    out.write_text(json.dumps(rows,indent=2),encoding='utf-8')
    for row in rows:print(json.dumps(row),flush=True)


if __name__=='__main__':main()
