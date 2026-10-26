"""Deterministic bounded A* on a supplied clearance mask; no engine API."""
import heapq
import math
import time
import numpy as np


def astar(free,start,goal,*,resolution_m=1,max_expansions=250000):
    begun=time.perf_counter()
    if (not isinstance(free,np.ndarray) or free.dtype!=bool or free.ndim!=2 or not free.size
            or isinstance(resolution_m,bool) or not isinstance(resolution_m,(int,float))
            or not math.isfinite(resolution_m) or resolution_m<=0
            or type(max_expansions)!=int or not 1<=max_expansions<=250000):
        raise ValueError('expected binary clearance mask, positive metric scale and bounded expansion count')
    for cell in (start,goal):
        if not isinstance(cell,(tuple,list)) or len(cell)!=2 or any(type(value)!=int for value in cell):
            raise ValueError('cell must contain two integer indices')
    start,goal=tuple(start),tuple(goal)
    def result(status,path=(),length=None,expanded=0):
        return dict(status=status,cells=[list(cell) for cell in path],path_length_m=length,
                    expanded_cells=expanded,planner_wall_s=time.perf_counter()-begun)
    def inside(cell):return 0<=cell[0]<free.shape[0] and 0<=cell[1]<free.shape[1]
    for name,cell in (('start',start),('goal',goal)):
        if not inside(cell):return result(name+'_outside')
        if not free[cell]:return result(name+'_blocked')
    scores={start:0.};parents={};closed=set()
    queue=[(math.dist(start,goal),0.,start)]
    while queue:
        _,cost,cell=heapq.heappop(queue)
        if cell in closed:continue
        if cell==goal:
            path=[cell]
            while path[-1] in parents:path.append(parents[path[-1]])
            return result('path_found',path[::-1],cost*resolution_m,len(closed))
        if len(closed)>=max_expansions:return result('expansion_budget',expanded=len(closed))
        closed.add(cell)
        for dr,dc in ((-1,0),(0,-1),(0,1),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)):
            nxt=(cell[0]+dr,cell[1]+dc)
            if not inside(nxt) or not free[nxt] or nxt in closed:continue
            if dr and dc and (not free[cell[0]+dr,cell[1]] or not free[cell[0],cell[1]+dc]):continue
            candidate=cost+math.hypot(dr,dc)
            if candidate<scores.get(nxt,math.inf):
                scores[nxt]=candidate;parents[nxt]=cell
                heapq.heappush(queue,(candidate+math.dist(nxt,goal),candidate,nxt))
    return result('no_path',expanded=len(closed))
