"""Read-only posthoc lighting/material metadata; never a policy/model input."""
import argparse
import json
from pathlib import Path
import unreal


def properties(obj,names):
    result={}
    for name in names:
        try:result[name]=str(obj.get_editor_property(name))
        except Exception:result[name]='not_exposed'
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists():raise RuntimeError('fresh diagnostics required')
    if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(args.map):raise RuntimeError('map load failed')
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    report=dict(evidence_kind='posthoc_render_lighting_material_metadata',map=args.map,lights=[],postprocess=[],materials=[])
    report['console_variables']={name:unreal.SystemLibrary.get_console_variable_int_value(name) for name in [
        'r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange','r.DefaultFeature.AutoExposure']}
    for actor in actors:
        if isinstance(actor,unreal.Light):
            component=actor.get_component_by_class(unreal.LightComponent)
            report['lights'].append(dict(actor_class=actor.get_class().get_name(),label=actor.get_actor_label(),
                properties=properties(component,['intensity','intensity_units','light_color','indirect_lighting_intensity'])))
        if isinstance(actor,unreal.PostProcessVolume):
            settings=actor.get_editor_property('settings')
            report['postprocess'].append(dict(label=actor.get_actor_label(),
                properties=properties(settings,['auto_exposure_method','auto_exposure_bias','auto_exposure_min_brightness',
                    'auto_exposure_max_brightness','color_saturation','color_contrast','scene_color_tint'])))
    seen=set()
    for actor in actors:
        if not isinstance(actor,unreal.StaticMeshActor):continue
        material=actor.static_mesh_component.get_material(0)
        if material is None or material.get_path_name() in seen:continue
        seen.add(material.get_path_name())
        row=dict(path=material.get_path_name(),class_name=material.get_class().get_name())
        for kind in ('scalar','vector','texture'):
            try:
                names=getattr(unreal.MaterialEditingLibrary,'get_'+kind+'_parameter_names')(material)
                row[kind+'_parameters']={}
                for name in names:
                    getter=getattr(unreal.MaterialEditingLibrary,'get_material_instance_'+kind+'_parameter_value')
                    row[kind+'_parameters'][str(name)]=str(getter(material,name))
            except Exception as error:row[kind+'_error']=type(error).__name__
        report['materials'].append(row)
        if len(report['materials'])>=6:break
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
