"""Synthetic UE integration regression: floor, capsule obstruction and NavMesh.

Uses built-in engine primitives in a disposable unsaved world. It verifies the
native commandlet lifecycle without relying on imported scene assets.
"""
import argparse
import json
from pathlib import Path
import unreal


def run():
    world = unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    mesh = unreal.load_asset('/Engine/BasicShapes/Cube')
    for label,location,scale in (
            ('SyntheticFloor',[0,0,-10],[20,20,.2]),
            ('SyntheticWall',[0,0,100],[1,12,2])):
        actor = subsystem.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*location))
        actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(mesh)
        actor.static_mesh_component.set_collision_profile_name('BlockAll')
        actor.set_actor_scale3d(unreal.Vector(*scale))
    report = json.loads(unreal.SpatialLabNavigation.build_oracle_navigation(
        world,unreal.Vector(0,0,100),unreal.Vector(1000,1000,500),34,176))
    report['evidence_kind']='synthetic_ue_integration'
    if report.get('populated_tiles',0) <= 0:
        return dict(report,passed=False,failure='empty_navmesh')
    nav = next(a for a in subsystem.get_all_level_actors() if isinstance(a,unreal.RecastNavMesh))
    a=unreal.NavigationSystemV1.project_point_to_navigation(world,unreal.Vector(-700,0,0),nav,None,unreal.Vector(50,50,100))
    b=unreal.NavigationSystemV1.project_point_to_navigation(world,unreal.Vector(700,0,0),nav,None,unreal.Vector(50,50,100))
    if a is None or b is None:
        return dict(report,passed=False,failure='floor_not_navigable')
    path=unreal.NavigationSystemV1.find_path_to_location_synchronously(world,a,b,nav)
    hit=unreal.SystemLibrary.capsule_trace_single(world,unreal.Vector(-500,0,90),unreal.Vector(500,0,90),
              34,88,unreal.TraceTypeQuery.ECC_VISIBILITY,False,[],unreal.DrawDebugTrace.NONE)
    points=[[float(p.x),float(p.y),float(p.z)] for p in path.path_points]
    report.update(path_cm=points,path_length_cm=path.get_path_length(),partial=path.is_partial(),
                  wall_blocks_capsule=hit is not None)
    report['passed']=bool(path.is_valid() and not path.is_partial() and path.get_path_length()>1500 and hit is not None)
    if not report['passed']:
        report['failure']='detour_or_capsule_query_failed'
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',required=True)
    args=parser.parse_args()
    out=Path(args.out)
    if out.exists():
        raise RuntimeError('use a new output path')
    result=run()
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    if not result['passed']:
        raise RuntimeError('synthetic navigation integration failed; see report')
    unreal.log('SITP_SYNTHETIC_NAVIGATION_TEST_PASSED')
