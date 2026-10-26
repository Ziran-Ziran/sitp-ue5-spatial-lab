"""Evaluation-only depth-consistent known-box projection; not instance masks.

World pose and fixture truth are privileged inputs here, never detector inputs.
Matching a box-front depth is a geometric visibility proxy: coincident scene
surfaces, material opacity and rasterization can invalidate optical identity.
"""
import math
import numpy as np
from local_depth_grid import unproject,number
from semantic_memory import position

HALF_EXTENTS_M=np.array([.1,.1,.3])
DEPTH_TOLERANCE_M=.002


def depth_consistent_box_masks(depth,calibration,*,camera_position_cm,heading_deg,entities):
    unproject(depth,calibration,eye_height_m=1.5)
    camera=np.array(position(camera_position_cm),dtype=float)/100;angle=math.radians(number(heading_deg))
    if (not isinstance(entities,list) or len(entities)>24 or depth.size>76800
            or np.any(np.abs(camera)>1e6)):raise ValueError('bounded evaluation camera/pixels/fixture targets required')
    rows,columns=np.indices(depth.shape);right=(columns-calibration['cx'])/calibration['fx'];up=-(rows-calibration['cy'])/calibration['fy']
    # Optical-depth parameter t: forward component is 1, not unit ray length.
    rays=np.stack((math.cos(angle)-right*math.sin(angle),math.sin(angle)+right*math.cos(angle),up),axis=-1)
    seen=set();results=[];entries=[]
    for entity in entities:
        if (not isinstance(entity,dict) or not {'entity_id','category','position_cm'}<=set(entity)
                or not isinstance(entity['entity_id'],str) or not entity['entity_id'] or entity['entity_id'] in seen
                or entity['category'] not in ('red_marker','blue_marker')):raise ValueError('unique controlled evaluation box identities required')
        seen.add(entity['entity_id']);center=np.array(position(entity['position_cm']),dtype=float)/100
        if np.any(np.abs(center)>1e6):raise ValueError('bounded fixture world coordinates required')
        low,high=center-HALF_EXTENTS_M,center+HALF_EXTENTS_M
        inside=bool(np.all(camera>=low)&np.all(camera<=high))
        enter=np.full(depth.shape,-np.inf);leave=np.full(depth.shape,np.inf);possible=np.ones(depth.shape,bool)
        for axis in range(3):
            parallel=np.abs(rays[:,:,axis])<1e-12
            possible &= ~parallel | ((camera[axis]>=low[axis])&(camera[axis]<=high[axis]))
            divisor=np.where(parallel,1,rays[:,:,axis]);a=(low[axis]-camera[axis])/divisor;b=(high[axis]-camera[axis])/divisor
            enter=np.maximum(enter,np.where(parallel,-np.inf,np.minimum(a,b)))
            leave=np.minimum(leave,np.where(parallel,np.inf,np.maximum(a,b)))
        projected=possible&(enter>0)&(enter<=leave)&(enter<=30)&~np.full(depth.shape,inside)
        entries.append(np.where(projected,enter,np.inf))
        results.append(dict(entity_id=entity['entity_id'],category=entity['category'],
            status='camera_inside_known_box' if inside else 'evaluated',projected_pixels=int(projected.sum()),projected_mask=projected))
    if not results:return dict(entities=[],ambiguous_depth_consistent_pixels=0,method='evaluation_only_depth_consistent_axis_aligned_fixture_boxes')
    entries=np.stack(entries);nearest=entries.min(axis=0);observed=depth.astype(float)/1000
    numeric=8*np.finfo(float).eps*np.maximum(1,np.where(np.isfinite(nearest),np.abs(nearest),0))
    with np.errstate(invalid='ignore'):
        nearest_boxes=np.isfinite(entries)&(np.abs(entries-nearest)<=DEPTH_TOLERANCE_M+numeric)
        consistent=nearest_boxes&(np.abs(entries-observed)<=DEPTH_TOLERANCE_M+numeric)&(depth>0)&(depth<65535)
    ambiguous=consistent.sum(axis=0)>1
    for index,row in enumerate(results):
        mask=consistent[index]&~ambiguous
        row.update(depth_consistent_pixels=int(mask.sum()),depth_consistent_mask=mask)
    return dict(entities=results,ambiguous_depth_consistent_pixels=int(ambiguous.sum()),
        method='evaluation_only_depth_consistent_axis_aligned_fixture_boxes',box_dimensions_m=(2*HALF_EXTENTS_M).tolist(),
        optical_depth_tolerance_m=DEPTH_TOLERANCE_M,
        interpretation='privileged same-pose box-front projection and saved optical depth; ambiguous boxes excluded; geometric proxy, not exact instance/material visibility ground truth')
