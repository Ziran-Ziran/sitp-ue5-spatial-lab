"""Query the saved engine NavMesh for two explicit oracle selection baselines.

Consumes a frozen local task manifest and emits every decision, including failed
selections. Planning evidence is not movement or Play success.
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from navigation_tasks import digest_manifest, validate_plan
from semantic_navigation import choose_target
import unreal


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--tasks',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists():
        raise RuntimeError('use a fresh output path')
    manifest=json.loads(args.tasks.read_text(encoding='utf-8-sig'))
    if manifest['condition']!='engine_oracle':
        raise ValueError('engine planner only supports explicit oracle condition')
    if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(manifest['map']):
        raise RuntimeError('failed loading experiment map')
    world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    nav=next(a for a in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
             if isinstance(a,unreal.RecastNavMesh))
    def query(start,goal):
        path=unreal.NavigationSystemV1.find_path_to_location_synchronously(
            world,unreal.Vector(*start),unreal.Vector(*goal),nav)
        if path is None or not path.is_valid():
            return dict(status='unreachable',points_cm=[])
        if path.is_partial():
            return dict(status='partial',points_cm=[])
        return dict(status='complete',points_cm=[[float(p.x),float(p.y),float(p.z)] for p in path.path_points])
    result=dict(schema_version=1,condition='engine_oracle',evidence_kind='editor_navmesh_planning_not_execution',
                map=manifest['map'],manifest_sha256=digest_manifest(manifest),runs=[])
    for task in manifest['episodes']:
        for policy in manifest['policies']:
            decision=choose_target(manifest['entities'],task['start_cm'],task['category'],policy,query)
            result['runs'].append(dict(run_id=task['episode_id']+'-'+policy,task=task,policy=policy,decision=decision))
    validate_plan(manifest,result)
    args.out.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    unreal.log('SITP_PLANNING_COMPLETE runs='+str(len(result['runs'])))


if __name__=='__main__':
    main()
