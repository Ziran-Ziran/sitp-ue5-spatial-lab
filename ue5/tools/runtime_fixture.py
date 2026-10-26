"""Create a disposable built-in primitive map for real Game-world acceptance."""
import argparse
import hashlib
import json
from pathlib import Path
import unreal


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--map',default='/Game/SpatialLab/RuntimeSynthetic_v1')
    parser.add_argument('--plan',type=Path)
    parser.add_argument('--semantic-fixture',action='store_true')
    args=parser.parse_args()
    if args.out.exists() or unreal.EditorAssetLibrary.does_asset_exist(args.map):
        raise RuntimeError('use fresh synthetic fixture output/map')
    if args.plan and args.plan.exists():
        raise RuntimeError('use a fresh synthetic fixture plan')
    # This assertion intentionally fails until the native runtime is implemented.
    if not hasattr(unreal,'SpatialLabGameMode'):
        raise AssertionError('native game-world experiment runner is not implemented')
    world=unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    subsystem=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    mesh=unreal.load_asset('/Engine/BasicShapes/Cube')
    shapes=[('Floor',[0,0,-10],[20,20,.2]),('Wall',[0,0,100],[1,12,2])]
    if args.semantic_fixture:
        shapes.extend([('Area_MdProps_Chair_SyntheticVisible',[-700,-500,50],[1,1,1]),
                       ('Area_MdProps_Lamp13_SyntheticOccluded',[700,-300,50],[1,1,1])])
    controls={}
    annotations=[]
    for label,xyz,scale in shapes:
        actor=subsystem.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*xyz))
        actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(mesh)
        actor.static_mesh_component.set_collision_profile_name('BlockAll')
        actor.set_actor_scale3d(unreal.Vector(*scale))
        if label.startswith('Area_MdProps_Chair'): controls['visible_chair']=actor.get_name()
        if label.startswith('Area_MdProps_Lamp13_'): controls['occluded_lamp']=actor.get_name()
        if label.startswith('Area_MdProps_'):
            annotations.append(dict(entity_id=actor.get_name(),category='chair' if 'Chair' in label else 'lamp',
                                    location_cm=[float(x) for x in xyz]))
    world.get_world_settings().set_editor_property('default_game_mode',unreal.SpatialLabGameMode)
    report=json.loads(unreal.SpatialLabNavigation.build_oracle_navigation(
        world,unreal.Vector(0,0,100),unreal.Vector(1000,1000,500),34,176))
    if report.get('populated_tiles',0)<=0:
        raise AssertionError('synthetic runtime fixture has empty NavMesh')
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,args.map):
        raise RuntimeError('cannot save fixture map')
    args.out.write_text(json.dumps(dict(map=args.map,navigation=report,semantic_controls=controls,
                                      semantic_annotations=annotations),indent=2),encoding='utf-8')
    if args.plan:
        rows=[]
        for name,start,goal in [('detour',[-700,0,10],[700,0,10]),
                                ('spawn-blocked',[0,0,10],[700,0,10]),
                                ('unreachable',[-700,0,10],[5000,0,10]),
                                ('direct-blocked',[-700,0,10],[700,0,10])]:
            task=dict(episode_id=name,category='chair',start_cm=start,max_sim_s=20,
                      max_path_cm=5000,arrival_tolerance_cm=30,success_radius_cm=300)
            rows.append(dict(run_id=name,policy='synthetic_navmesh',task=task,
                             decision=dict(status='complete',entity_id='synthetic-chair',goal_cm=goal,
                                           executor='direct_line' if name=='direct-blocked' else 'engine_navmesh')))
        plan=dict(schema_version=1,condition='engine_oracle',evidence_kind='synthetic_fixture_plan',map=args.map,runs=rows)
        plan['manifest_sha256']=hashlib.sha256(json.dumps(plan,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        args.plan.write_text(json.dumps(plan,indent=2),encoding='utf-8')
