"""Owned primitive ramp/roof capture, separate from pixel-only ground models."""
import argparse
import json
from pathlib import Path
import sys
import unreal
sys.path.insert(0,str(Path(__file__).resolve().parent))
from support_physics_cases import cases,floor_transform,ceiling_transform
from pitch_sweep_capture import capture_sequence,write
from rendered_replay import RENDER_SETTINGS


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists() or unreal.EditorAssetLibrary.does_asset_exist(args.map):raise ValueError('new owned map and capture required')
    if not args.map.startswith('/Game/SpatialLab/'):raise ValueError('owned fixture path required')
    world=unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    subsystem=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    mesh=unreal.load_asset('/Engine/BasicShapes/Cube')
    material=unreal.AssetToolsHelpers.get_asset_tools().create_asset('Control',args.map+'_Materials',unreal.Material,unreal.MaterialFactoryNew())
    material.set_editor_property('shading_model',unreal.MaterialShadingModel.MSM_UNLIT)
    expression=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector)
    expression.set_editor_property('constant',unreal.LinearColor(.2,.4,.6,1))
    if not unreal.MaterialEditingLibrary.connect_material_property(expression,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR):raise RuntimeError('unlit fixture material failed')
    unreal.MaterialEditingLibrary.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material)
    actors=[]
    for case in cases():
        for kind,transform in [('floor',floor_transform(case)),*( [('ceiling',ceiling_transform(case))] if case['ceiling_vertical_gap_cm'] is not None else [])]:
            center,scale,pitch=transform
            actor=subsystem.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*center),unreal.Rotator(pitch=pitch,yaw=0,roll=0))
            actor.set_actor_label(case['case_id']+'_'+kind)
            component=actor.static_mesh_component;component.set_static_mesh(mesh);component.set_collision_profile_name('BlockAll')
            component.set_material(0,material);actor.set_actor_scale3d(unreal.Vector(*scale))
            actual=actor.get_actor_rotation();location=actor.get_actor_location();size=actor.get_actor_scale3d()
            actors.append(dict(case_id=case['case_id'],kind=kind,center_cm=[location.x,location.y,location.z],
                scale=[size.x,size.y,size.z],pitch_deg=actual.pitch,collision_profile=str(component.get_collision_profile_name())))
    world.get_world_settings().set_editor_property('default_game_mode',unreal.SpatialLabGameMode)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,args.map):raise RuntimeError('fixture save failed')
    args.out.mkdir();(args.out/'input').mkdir();(args.out/'evaluation').mkdir()
    unreal.SystemLibrary.execute_console_command(world,'r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange 1')
    report=dict(schema_version=1,evidence_kind='synthetic_observed_support_plane_fixture',map=args.map,
        cases=cases(),actors=actors,navmesh_build_called=False,
        navigation_data_actors=sum(a.get_class().get_name()=='RecastNavMesh' for a in subsystem.get_all_level_actors()),
        finished=False,sequences=[],interpretation='known primitive construction/poses only in evaluation; geometry consumes isolated pixels')
    with (args.out/'evaluation/camera_poses.jsonl').open('x',encoding='utf-8') as poses:
        for index,case in enumerate(cases()):
            start=case['start_cm'];frame=dict(frame_id=0,source_frame_id=0,t_source_sim_s=0,
                camera_position_cm=[start[0],start[1],start[2]+150],camera_rotation_deg=[0,0,0])
            row=capture_sequence(world,args.out,f'sequence_{index:03d}',case['case_id'],[frame],RENDER_SETTINGS,poses)
            report['sequences'].append(row)
    report['finished']=True;write(args.out/'fixture.json',report)
