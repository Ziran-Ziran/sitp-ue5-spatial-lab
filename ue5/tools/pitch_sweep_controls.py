"""UE-only owned primitive flat/wall/drop maps for calibrated depth controls."""
import argparse
import json
from pathlib import Path
import sys
import unreal
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pitch_sweep_capture import capture_sequence,write
from rendered_replay import RENDER_SETTINGS


def fixture(path,kind,material):
    world=unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    subsystem=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    cube=unreal.load_asset('/Engine/BasicShapes/Cube')
    def block(label,center,scale):
        actor=subsystem.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*center));actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(cube);actor.set_actor_scale3d(unreal.Vector(*scale))
        actor.static_mesh_component.set_material(0,material)
    if kind=='drop':
        block('NearFloor',[-200,0,-10],[6,10,.2])
        block('FarFloor',[900,0,-10],[12,10,.2])
        block('LowerChannel',[200,0,-110],[2,10,.2])
    else:block('Floor',[500,0,-10],[20,10,.2])
    if kind=='wall':block('BodyHeightWall',[160,0,100],[.2,2,2])
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,path):raise RuntimeError('owned fixture save failed')
    return world


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-prefix',required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists() or any(unreal.EditorAssetLibrary.does_asset_exist(args.map_prefix+'_'+kind) for kind in ('flat','wall','drop')):
        raise ValueError('new owned control maps and output required')
    args.out.mkdir(parents=True);(args.out/'input').mkdir();(args.out/'evaluation').mkdir()
    assets=unreal.AssetToolsHelpers.get_asset_tools()
    material=assets.create_asset('Control',args.map_prefix+'_Materials',unreal.Material,unreal.MaterialFactoryNew())
    material.set_editor_property('shading_model',unreal.MaterialShadingModel.MSM_UNLIT)
    expression=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector)
    expression.set_editor_property('constant',unreal.LinearColor(.2,.4,.6,1))
    if not unreal.MaterialEditingLibrary.connect_material_property(expression,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR):
        raise RuntimeError('cannot connect control material')
    unreal.MaterialEditingLibrary.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material)
    report=dict(evidence_kind='synthetic_calibrated_pitch_controls',finished=False,sequences=[])
    with (args.out/'evaluation/camera_poses.jsonl').open('x',encoding='utf-8') as poses:
        for i,kind in enumerate(('flat','wall','drop')):
            world=fixture(args.map_prefix+'_'+kind,kind,material)
            unreal.SystemLibrary.execute_console_command(world,'r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange 1')
            frame=dict(frame_id=0,source_frame_id=0,t_source_sim_s=0,camera_position_cm=[0,0,150],camera_rotation_deg=[0,0,0])
            row=capture_sequence(world,args.out,f'sequence_{i:03d}',kind,[frame],RENDER_SETTINGS,poses)
            row.update(control=kind,map=args.map_prefix+'_'+kind);report['sequences'].append(row)
    report['finished']=True;write(args.out/'controls.json',report)
