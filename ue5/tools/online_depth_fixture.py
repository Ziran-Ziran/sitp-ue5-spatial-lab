"""Owned live-depth controls; no NavMesh and a fixed motion-triggered obstacle."""
import argparse
import json
from pathlib import Path
import unreal

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if not hasattr(unreal,'SpatialLabOnlineDepthRunner'):raise AssertionError('online stopped-depth runner not implemented')
    if args.out.exists() or unreal.EditorAssetLibrary.does_asset_exist(args.map):raise ValueError('fresh fixture required')
    world=unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem);cube=unreal.load_asset('/Engine/BasicShapes/Cube')
    for label,position,scale in (
            ('Floor',[0,0,-10],[40,40,.2]),
            ('StaticWall',[160,500,100],[.2,3,2]),
            ('SpatialLabInterventionWall',[180,-500,100],[.2,3,2])):
        actor=actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*position));actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(cube);actor.set_actor_scale3d(unreal.Vector(*scale))
        actor.static_mesh_component.set_collision_profile_name('BlockAll')
        if label=='SpatialLabInterventionWall':
            actor.tags=['SpatialLabInterventionWall'];actor.set_actor_hidden_in_game(True)
            actor.set_actor_enable_collision(False)
    world.get_world_settings().set_editor_property('default_game_mode',unreal.SpatialLabGameMode)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,args.map):raise RuntimeError('fixture map save failed')
    record=dict(evidence_kind='synthetic_online_depth_fixture',map=args.map,navmesh_build_called=False,
        intervention='same fixed wall enabled after50cm actual motion in both compared modes',
        navigation_data_actors=sum(a.get_class().get_name()=='RecastNavMesh' for a in actors.get_all_level_actors()))
    with args.out.open('x',encoding='utf-8') as target:json.dump(record,target,indent=2)
