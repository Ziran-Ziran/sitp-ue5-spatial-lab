"""Local depth geometry with explicit upright/height priors, never engine meshes.

Coordinates are right, forward, up in meters. Observed near-reference-floor
samples support cells; body-height or below-floor samples block them. Unknown stays blocked.
This is a conservative sampled estimate, not a surface/physics certificate.
"""
from dataclasses import dataclass,asdict
import math
import numpy as np


def number(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError('expected a finite number')
    return value


@dataclass(frozen=True)
class GridConfig:
    resolution_m:float=.25
    right_min_m:float=-5
    right_max_m:float=5
    forward_min_m:float=-2
    forward_max_m:float=10
    eye_height_m:float=1.5
    floor_tolerance_m:float=.1
    body_height_m:float=1.76
    radius_m:float=.34

    def __post_init__(self):
        for value in asdict(self).values():number(value)
        if (not .05<=self.resolution_m<=1 or not .1<=self.eye_height_m<=3
                or not 0<self.floor_tolerance_m<self.body_height_m<=3 or not 0<=self.radius_m<=2):
            raise ValueError('invalid bounded grid/body/height prior')
        for low,high in ((self.right_min_m,self.right_max_m),(self.forward_min_m,self.forward_max_m)):
            size=(high-low)/self.resolution_m
            if size<=0 or size>500 or not math.isclose(size,round(size),abs_tol=1e-8):
                raise ValueError('grid bounds must span a positive integral number of cells, at most 500')

    @property
    def shape(self):
        return (round((self.forward_max_m-self.forward_min_m)/self.resolution_m),
                round((self.right_max_m-self.right_min_m)/self.resolution_m))

    def cell(self,right,forward):
        return (math.floor((number(forward)-self.forward_min_m)/self.resolution_m),
                math.floor((number(right)-self.right_min_m)/self.resolution_m))

    def center(self,cell):
        row,column=cell
        return [self.right_min_m+(column+.5)*self.resolution_m,
                self.forward_min_m+(row+.5)*self.resolution_m]


def unproject(depth,calibration,*,eye_height_m):
    """Integer pixel centers; optical mm depth -> local right/forward/up meters."""
    keys=('w','h','fx','fy','cx','cy')
    if not isinstance(calibration,dict) or not set(keys)<=set(calibration):raise ValueError('missing independent intrinsics')
    for key in keys:number(calibration[key])
    w,h=calibration['w'],calibration['h']
    if (type(w)!=int or type(h)!=int or not 1<=w<=2048 or not 1<=h<=2048
            or calibration['fx']<=0 or calibration['fy']<=0
            or not 0<=calibration['cx']<w or not 0<=calibration['cy']<h
            or not isinstance(depth,np.ndarray) or depth.dtype!=np.uint16 or depth.shape!=(h,w)
            or np.any((depth>30000)&(depth!=65535)) or not .1<=number(eye_height_m)<=3):
        raise ValueError('expected bounded uint16 optical-mm depth and valid independent intrinsics/eye height')
    rows,columns=np.nonzero((depth>0)&(depth<65535))
    forward=depth[rows,columns].astype(float)/1000
    right=(columns-calibration['cx'])*forward/calibration['fx']
    up=eye_height_m-(rows-calibration['cy'])*forward/calibration['fy']
    return np.column_stack((right,forward,up))


def clearance_mask(states,resolution_m,radius_m):
    """Inflate nonfree cell bounding circles and the unknown outside boundary."""
    if (not isinstance(states,np.ndarray) or states.ndim!=2 or not states.size
            or not np.isin(states,[-1,0,1,2]).all() or not 0<number(resolution_m)<=1
            or not 0<=number(radius_m)<=2):raise ValueError('invalid occupancy or body clearance')
    distance=radius_m/resolution_m+math.sqrt(2)/2
    padding=math.ceil(distance)
    padded=np.pad(states==0,padding,constant_values=False)
    rows,columns=states.shape
    free=np.ones(states.shape,dtype=bool)
    for dr in range(-padding,padding+1):
        for dc in range(-padding,padding+1):
            if math.hypot(dr,dc)<=distance:
                free &= padded[padding+dr:padding+dr+rows,padding+dc:padding+dc+columns]
    return free


def depth_grid(depth,calibration,config):
    if not isinstance(config,GridConfig):raise ValueError('explicit GridConfig required')
    points=unproject(depth,calibration,eye_height_m=config.eye_height_m)
    columns=np.floor((points[:,0]-config.right_min_m)/config.resolution_m).astype(int)
    rows=np.floor((points[:,1]-config.forward_min_m)/config.resolution_m).astype(int)
    inside=(rows>=0)&(rows<config.shape[0])&(columns>=0)&(columns<config.shape[1])
    rows,columns,up=rows[inside],columns[inside],points[inside,2]
    ground=np.abs(up)<=config.floor_tolerance_m
    obstacle=(up>config.floor_tolerance_m)&(up<=config.body_height_m)
    negative=up < -config.floor_tolerance_m
    states=np.full(config.shape,-1,dtype=np.int8)
    states[rows[ground],columns[ground]]=0
    states[rows[negative],columns[negative]]=2
    states[rows[obstacle],columns[obstacle]]=1
    return dict(config=config,states=states,free=clearance_mask(states,config.resolution_m,config.radius_m),
        valid_depth_points=len(points),in_bounds_points=int(inside.sum()),
        floor_samples=int(ground.sum()),obstacle_samples=int(obstacle.sum()),negative_samples=int(negative.sum()),
        condition='depth_geometry_upright_eye_height_prior',
        interpretation='sampled local estimate; no engine geometry, inferred unseen floor, slope or physics guarantee')
