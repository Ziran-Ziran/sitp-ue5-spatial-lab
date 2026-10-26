"""Conservative continuous square-footprint gate on a sampled local depth grid.

Grid-center clearance alone does not cover arbitrary query connectors. This
checks every supplied segment against expanded non-floor cells and bounds.
It remains a depth-grid estimate, never a guarantee about the physical world.
"""
import math
import numpy as np
from local_depth_grid import GridConfig,number


def point(value):
    if not isinstance(value,(tuple,list)) or len(value)!=2:raise ValueError('right/forward meter pair required')
    return [number(item) for item in value]


def validate_route(states,config,centers,*,start,goal,tracking_margin_m=.1):
    if (not isinstance(config,GridConfig) or not isinstance(states,np.ndarray) or states.shape!=config.shape
            or not np.isin(states,[-1,0,1,2]).all() or not 0<=number(tracking_margin_m)<=.5
            or not isinstance(centers,list)):
        raise ValueError('explicit valid local grid, bounded tracking margin and route required')
    start,goal=point(start),point(goal);centers=[point(value) for value in centers]
    base=dict(footprint='axis_aligned_square_enclosing_capsule_disk',body_radius_m=config.radius_m,
              tracking_margin_m=tracking_margin_m,waypoints_right_forward_m=[],continuous_route_length_m=None)
    def result(status,**details):return dict(base,status=status,**details)
    if not centers:return result('no_planned_path')
    if len(centers)>256:return result('waypoint_budget')
    radius=config.radius_m+tracking_margin_m
    rows,columns=np.nonzero(states!=0)
    lower=np.column_stack((config.right_min_m+columns*config.resolution_m,
                           config.forward_min_m+rows*config.resolution_m))-radius
    upper=lower+config.resolution_m+2*radius
    limit_low=np.array([config.right_min_m,config.forward_min_m])+radius
    limit_high=np.array([config.right_max_m,config.forward_max_m])-radius
    def blocked(a,b):
        a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
        if np.any(a<=limit_low) or np.any(a>=limit_high) or np.any(b<=limit_low) or np.any(b>=limit_high):
            return dict(blocking_source='unknown_outside_grid')
        enter=np.zeros(len(rows));leave=np.ones(len(rows));possible=np.ones(len(rows),dtype=bool)
        delta=b-a
        for axis in (0,1):
            if abs(delta[axis])<1e-12:
                possible &= (a[axis]>=lower[:,axis])&(a[axis]<=upper[:,axis])
            else:
                first=(lower[:,axis]-a[axis])/delta[axis];last=(upper[:,axis]-a[axis])/delta[axis]
                enter=np.maximum(enter,np.minimum(first,last));leave=np.minimum(leave,np.maximum(first,last))
        hits=np.flatnonzero(possible&(enter<=leave))
        if len(hits):
            index=hits[0];cell=[int(rows[index]),int(columns[index])]
            return dict(blocking_source='nonfloor_cell',blocking_cell=cell,blocking_state=int(states[tuple(cell)]))
        return None
    for name,location in (('start',start),('goal',goal)):
        failure=blocked(location,location)
        if failure:return result(name+'_footprint_blocked',**failure)
    route=[]
    for location in ([start] if start==goal else [start,*centers,goal]):
        if not route or math.dist(route[-1],location)>1e-9:route.append(location)
    for index,(a,b) in enumerate(zip(route,route[1:])):
        failure=blocked(a,b)
        if failure:return result('segment_footprint_blocked',segment_index=index,**failure)
    return dict(base,status='approved',waypoints_right_forward_m=route,
                continuous_route_length_m=sum(math.dist(a,b) for a,b in zip(route,route[1:])))
