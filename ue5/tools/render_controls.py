"""Render a new built-in asymmetric RGB/depth calibration fixture; no city assets."""
import argparse
import json
from pathlib import Path
import unreal


def create_fixture(map_path):
    world=unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    assets=unreal.AssetToolsHelpers.get_asset_tools()
    material_folder=map_path+'_Materials'
    materials={}
    for name,color in [('Red',(1,0,0)),('Green',(0,1,0)),('Blue',(0,0,1))]:
        material=assets.create_asset(name,material_folder,unreal.Material,unreal.MaterialFactoryNew())
        material.set_editor_property('shading_model',unreal.MaterialShadingModel.MSM_UNLIT)
        expression=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector)
        expression.set_editor_property('constant',unreal.LinearColor(*color,1))
        if not unreal.MaterialEditingLibrary.connect_material_property(expression,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR):
            raise RuntimeError('cannot connect synthetic emissive color')
        unreal.MaterialEditingLibrary.recompile_material(material)
        unreal.EditorAssetLibrary.save_loaded_asset(material)
        materials[name]=material
    subsystem=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    cube=unreal.load_asset('/Engine/BasicShapes/Cube')
    for label,xyz,scale,color in [('FarPlane',[1010,0,0],[.2,40,30],'Blue'),
                                 ('UpperLeft',[500,-250,200],[.2,2,2],'Red'),
                                 ('LowerRight',[500,250,-200],[.2,2,2],'Green')]:
        actor=subsystem.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*xyz))
        actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(cube)
        actor.set_actor_scale3d(unreal.Vector(*scale))
        actor.static_mesh_component.set_material(0,materials[color])
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,map_path):raise RuntimeError('fixture save failed')
    return world


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--reuse-fixture',action='store_true')
    args=parser.parse_args()
    if args.out.exists() or (not args.reuse_fixture and unreal.EditorAssetLibrary.does_asset_exist(args.map)):
        raise RuntimeError('fresh fixture map/output required')
    args.out.mkdir(parents=True)
    if not hasattr(unreal,'SpatialLabRgbd'):raise AssertionError('native RGB-D renderer is not available')
    if args.reuse_fixture:
        if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(args.map):
            raise RuntimeError('cannot load existing control fixture')
        world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    else:
        world=create_fixture(args.map)
    # This isolated render process uses EV100 values explicitly. The imported
    # project's legacy default may interpret brightness bounds in luminance.
    prior_mode=unreal.SystemLibrary.get_console_variable_int_value('r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange')
    unreal.SystemLibrary.execute_console_command(world,'r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange 1')
    result=dict(evidence_kind='synthetic_rendered_rgbd_controls',map=args.map,reused_fixture=args.reuse_fixture,frames=[])
    result.update(prior_luminance_mode=prior_mode,luminance_units='EV100',override_scope='render_process_only')
    for index,(name,position,yaw,exposure) in enumerate([('pose-a',[0,0,0],0,1),('pose-b',[200,0,0],0,1),
            ('pose-a-repeat',[0,0,0],0,1),('empty-backward',[0,0,0],180,1),('exposure-dark',[0,0,0],0,13)]):
        prefix=args.out/name
        record=json.loads(unreal.SpatialLabRgbd.capture_pose(world,unreal.Vector(*position),
            unreal.Rotator(pitch=0,yaw=yaw,roll=0),str(prefix)+'.rgb.png',str(prefix)+'.depth.png',exposure))
        record.update(name=name,camera_requested_cm=position,yaw_requested_deg=yaw,
                      color_file=name+'.rgb.png',depth_file=name+'.depth.png')
        result['frames'].append(record)
        if not record['captured']:
            (args.out/'render.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
            raise RuntimeError('native capture failed; raw partial outputs retained')
    (args.out/'render.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
