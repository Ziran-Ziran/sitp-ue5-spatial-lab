"""Target-independent bounded engine-geometry anchors. Never reads actor labels.

Accepts only start queries exported through semantic_search_tasks. Does not query
target reachability, tune against annotations, alter maps or save engine assets.
"""
import argparse
import json
import math
from pathlib import Path
import unreal


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--starts',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists():raise RuntimeError('use fresh anchor report output')
    data=json.loads(args.starts.read_text(encoding='utf-8-sig'))
    if set(data)!={'schema_version','map','queries'} or data['schema_version']!=1:
        raise ValueError('only start geometry queries accepted')
    if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(data['map']):
        raise RuntimeError('cannot load frozen map')
    world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    nav_data=next((actor for actor in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
                   if isinstance(actor,unreal.RecastNavMesh)),None)
    if nav_data is None:raise RuntimeError('frozen map has no NavMesh data')
    result=dict(schema_version=1,map=data['map'],source='engine_navmesh_fixed_grid',
                rule=dict(offsets_xy_cm=[-1500,-750,0,750,1500],projection_extent_cm=[150,150,100],
                          maximum_vertical_change_cm=100,capsule_radius_cm=34,capsule_half_height_cm=88,
                          collision_profile='Pawn'),
                points_by_episode={},queries=[])
    cache={}
    for query in data['queries']:
        if set(query)!={'episode_id','start_cm'}:raise ValueError('unexpected privileged start fields')
        start=query['start_cm']
        if len(start)!=3 or not all(math.isfinite(x) for x in start):raise ValueError('invalid start coordinates')
        key=tuple(start)
        if key not in cache:
            points=[]
            for dx in (-1500,-750,0,750,1500):
                for dy in (-1500,-750,0,750,1500):
                    if dx==dy==0:continue
                    requested=[start[0]+dx,start[1]+dy,start[2]]
                    # The static Python API returns a projected vector or None.
                    point=unreal.NavigationSystemV1.project_point_to_navigation(world,unreal.Vector(*requested),
                        nav_data,None,unreal.Vector(150,150,100))
                    row=dict(start_cm=start,requested_cm=requested,status='not_projected')
                    if point is not None:
                        xyz=[float(point.x),float(point.y),float(point.z)]
                        center=unreal.Vector(xyz[0],xyz[1],xyz[2]+90)
                        hit=unreal.SystemLibrary.capsule_trace_single_by_profile(world,center,
                            unreal.Vector(center.x,center.y,center.z+.1),34,88,'Pawn',False,[],unreal.DrawDebugTrace.NONE)
                        blocked=bool(hit and hit.to_tuple()[0])
                        row.update(projected_cm=xyz,status='blocked' if blocked else 'height_rejected'
                                   if abs(xyz[2]-start[2])>100 else 'ready')
                        if row['status']=='ready' and xyz not in points:points.append(xyz)
                    result['queries'].append(row)
            cache[key]=points
        result['points_by_episode'][query['episode_id']]=cache[key]
    args.out.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
