"""Pixel-only fixed one-meter ground controls; no fixture metadata or world pose."""
import numpy as np
from local_depth_grid import GridConfig,multiview_grid
from support_plane_grid import support_plane_grid,MODEL
from continuous_grid_planner import plan_route
from online_depth_policy import CALIBRATION


def plan_views(views):
    config=GridConfig();result={}
    for name,builder in (('horizontal_v1',multiview_grid),(MODEL,support_plane_grid)):
        grid=builder(views,CALIBRATION,config)
        result[name]=dict(support_plane=grid.get('support_plane'),
            observed_floor_cells=int(np.count_nonzero(grid['states']==0)),unknown_cells=int(np.count_nonzero(grid['states']==-1)),
            obstacle_cells=int(np.count_nonzero(grid['states']==1)),negative_cells=int(np.count_nonzero(grid['states']==2)),
            route=plan_route(grid,(0,0),(0,1),'legacy_centers'))
    return result
