"""Versioned numerical indexing of query points; never change raw geometry."""
import math
from local_depth_grid import GridConfig,number

POLICIES=('legacy_floor','boundary_snap_1e-9_cell')
TOLERANCE_CELLS=1e-9


def query_cell(config,point,policy):
    if not isinstance(config,GridConfig) or policy not in POLICIES or not isinstance(point,(list,tuple)) or len(point)!=2:
        raise ValueError('explicit valid grid, query pair and indexing policy required')
    columns=[]
    for value,low,high in zip(point,(config.right_min_m,config.forward_min_m),(config.right_max_m,config.forward_max_m)):
        number(value);fraction=(value-low)/config.resolution_m
        if not math.isfinite(fraction):raise ValueError('unrepresentable query index')
        index=math.floor(fraction);nearest=round(fraction);count=round((high-low)/config.resolution_m)
        if policy=='boundary_snap_1e-9_cell' and low<=value<high and 0<nearest<count and abs(fraction-nearest)<=TOLERANCE_CELLS:
            index=nearest
        columns.append(index)
    return columns[1],columns[0]
