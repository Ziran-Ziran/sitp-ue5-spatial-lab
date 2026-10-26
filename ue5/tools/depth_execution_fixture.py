"""Owned primitive physics controls, deliberately without building NavMesh."""
import argparse
import json
from pathlib import Path
import unreal


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if not hasattr(unreal,'SpatialLabDepthRunner'):raise AssertionError('depth waypoint CharacterMovement runner not implemented')
    if args.out.exists() or unreal.EditorAssetLibrary.does_asset_exist(args.map):raise ValueError('new control map/output required')
    world=unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    subsystem=unreal.get_editor_subsystem(unreal.EditorActorSubsystem);mesh=unreal.load_asset('/Engine/BasicShapes/Cube')
    for label,position,scale in [('Floor',[0,0,-10],[20,20,.2]),('Wall',[0,0,100],[1,12,2])]:
        actor=subsystem.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*position));actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(mesh);actor.static_mesh_component.set_collision_profile_name('BlockAll')
        actor.set_actor_scale3d(unreal.Vector(*scale))
    world.get_world_settings().set_editor_property('default_game_mode',unreal.SpatialLabGameMode)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,args.map):raise RuntimeError('control map save failed')
    report=dict(evidence_kind='primitive_character_movement_fixture_no_navmesh_build',map=args.map,
        navmesh_build_called=False,navigation_data_actors=sum(actor.get_class().get_name()=='RecastNavMesh' for actor in subsystem.get_all_level_actors()))
    with args.out.open('x',encoding='utf-8') as target:json.dump(report,target,indent=2)
