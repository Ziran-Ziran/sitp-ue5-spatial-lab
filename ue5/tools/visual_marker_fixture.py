"""Owned neutral geometry and unlit color markers; truth exported separately."""
import argparse
import json
from pathlib import Path
import unreal


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--truth',type=Path,required=True);args=parser.parse_args()
    runner=getattr(unreal,'SpatialLabOnlineDepthRunner',None)
    if runner is None or not hasattr(runner,'supports_visual_marker_tasks') or not runner.supports_visual_marker_tasks():
        raise AssertionError('pixel-marker target runner not implemented in loaded native build')
    if args.out.exists() or args.truth.exists() or unreal.EditorAssetLibrary.does_asset_exist(args.map):raise ValueError('fresh owned map/truth/output required')
    world=unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem);cube=unreal.load_asset('/Engine/BasicShapes/Cube')
    assets=unreal.AssetToolsHelpers.get_asset_tools();materials={}
    for name,rgb in (('Neutral',[.2,.2,.2]),('Red',[.8,.015,.015]),('Blue',[.015,.015,.8])):
        material=assets.create_asset(name,args.map+'_Materials',unreal.Material,unreal.MaterialFactoryNew())
        material.set_editor_property('shading_model',unreal.MaterialShadingModel.MSM_UNLIT)
        expression=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector)
        expression.set_editor_property('constant',unreal.LinearColor(*rgb,1))
        if not unreal.MaterialEditingLibrary.connect_material_property(expression,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR):raise RuntimeError('material connection failed')
        unreal.MaterialEditingLibrary.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material);materials[name]=material
    truth=[]
    for label,center,scale,color,tag,category in (
            ('Floor',[0,0,-10],[40,40,.2],'Neutral',None,None),
            ('Marker0',[300,-200,100],[.2,.2,.6],'Red','SpatialLabMarkerRed','red_marker'),
            ('Marker1',[300,200,100],[.2,.2,.6],'Blue','SpatialLabMarkerBlue','blue_marker'),
            ('Occluder',[100,0,100],[.2,12,2],'Neutral','SpatialLabInterventionWall',None)):
        actor=actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*center));actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(cube);actor.set_actor_scale3d(unreal.Vector(*scale))
        actor.static_mesh_component.set_collision_profile_name('BlockAll');actor.static_mesh_component.set_material(0,materials[color])
        if tag:actor.tags=[tag]
        if category:
            actual=actor.get_actor_location();truth.append(dict(entity_id=f'fixture_{len(truth):03d}',category=category,
                position_cm=[actual.x,actual.y,actual.z],source='owned_fixture_material_palette_ground_truth'))
        if tag=='SpatialLabInterventionWall':actor.set_actor_hidden_in_game(True);actor.set_actor_enable_collision(False)
    world.get_world_settings().set_editor_property('default_game_mode',unreal.SpatialLabGameMode)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,args.map):raise RuntimeError('marker map save failed')
    with args.truth.open('x',encoding='utf-8') as target:
        json.dump(dict(schema_version=1,evidence_kind='independent_owned_marker_fixture_truth',map=args.map,entities=truth),target,indent=2)
    with args.out.open('x',encoding='utf-8') as target:
        json.dump(dict(evidence_kind='owned_rgb_marker_depth_fixture',map=args.map,navmesh_build_called=False,
            navigation_data_actors=sum(a.get_class().get_name()=='RecastNavMesh' for a in actors.get_all_level_actors()),
            marker_count=len(truth),truth_scope='post_run_evaluation_only_not_native_or_policy_input'),target,indent=2)
