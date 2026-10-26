"""A* center support and optional shortcuts using the unchanged square gate."""
import math
import numpy as np
from continuous_depth_path import validate_route
from depth_grid_replay import plan_grid
from local_depth_grid import GridConfig,number

VARIANTS=('legacy_centers','continuous_square','continuous_square_shortcuts')


def center_mask(states,config,*,tracking_margin_m=.1):
    if (not isinstance(config,GridConfig) or not isinstance(states,np.ndarray) or states.shape!=config.shape
            or not np.isin(states,[-1,0,1,2]).all() or not 0<=number(tracking_margin_m)<=.5):
        raise ValueError('valid local grid and bounded tracking margin required')
    # Closed nonfloor cell boxes expanded by the same square as validate_route.
    # A neighboring cell at offset n touches the square iff n*res <= R+res/2.
    reach=config.radius_m+tracking_margin_m+config.resolution_m/2
    padding=math.floor(reach/config.resolution_m+1e-10)
    blocked=np.pad(states!=0,padding,constant_values=True)
    integral=np.pad(blocked.astype(np.int64),((1,0),(1,0))).cumsum(0).cumsum(1)
    width=2*padding+1
    counts=integral[width:,width:]-integral[:-width,width:]-integral[width:,:-width]+integral[:-width,:-width]
    return counts==0


def shortcut_route(states,config,route,*,tracking_margin_m=.1,max_queries=4096):
    if type(max_queries)!=int or not 1<=max_queries<=4096 or not isinstance(route,list) or not route:
        raise ValueError('bounded nonempty safe route and query budget required')
    original=validate_route(states,config,route,start=route[0],goal=route[-1],tracking_margin_m=tracking_margin_m)
    if original['status']!='approved':raise ValueError('only an already continuously approved route may be shortened')
    route=original['waypoints_right_forward_m'];result=[route[0]];index=0;queries=0;exhausted=False
    while index<len(route)-1:
        if queries>=max_queries:
            result.extend(route[index+1:]);exhausted=True;break
        chosen=index+1
        for candidate in range(len(route)-1,index+1,-1):
            if queries>=max_queries:exhausted=True;break
            queries+=1
            gate=validate_route(states,config,[route[index],route[candidate]],start=route[index],goal=route[candidate],tracking_margin_m=tracking_margin_m)
            if gate['status']=='approved':chosen=candidate;break
        result.append(route[chosen]);index=chosen
    final=validate_route(states,config,result,start=route[0],goal=route[-1],tracking_margin_m=tracking_margin_m)
    if final['status']!='approved':raise ValueError('shortcut failed unchanged continuous gate')
    return dict(route=final['waypoints_right_forward_m'],queries=queries,budget_exhausted=exhausted)


def plan_route(grid,start,goal,variant,*,tracking_margin_m=.1):
    if variant not in VARIANTS:raise ValueError('explicit frozen planner variant required')
    free=grid['free'] if variant=='legacy_centers' else center_mask(grid['states'],grid['config'],tracking_margin_m=tracking_margin_m)
    planning=plan_grid(dict(grid,free=free),start,goal)
    gate=(validate_route(grid['states'],grid['config'],planning['path_right_forward_m'],start=start,goal=goal,tracking_margin_m=tracking_margin_m)
          if planning['status']=='path_found' else dict(status='planning_'+planning['status'],waypoints_right_forward_m=[]))
    queries=0;exhausted=False
    if gate['status']=='approved' and variant=='continuous_square_shortcuts':
        shortcut=shortcut_route(grid['states'],grid['config'],gate['waypoints_right_forward_m'],tracking_margin_m=tracking_margin_m)
        queries=shortcut['queries'];exhausted=shortcut['budget_exhausted']
        gate=validate_route(grid['states'],grid['config'],shortcut['route'],start=start,goal=goal,tracking_margin_m=tracking_margin_m)
    return dict(planner_variant=variant,planning=planning,continuous_gate=gate,clearance_free_cells=int(free.sum()),
        shortcut_queries=queries,shortcut_query_budget=4096,shortcut_budget_exhausted=exhausted)
