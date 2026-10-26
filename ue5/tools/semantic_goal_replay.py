"""Sequential goal-only semantic selection replay on visibility-supervision logs.

This is not a closed-loop navigation experiment: routes were selected by the
global engine oracle. No catalog/truth file is accepted by the replay command.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
from navigation_tasks import digest_manifest
from semantic_memory import CONDITION,CATEGORIES,SemanticMemory
from semantic_observation_stream import iter_validated,jsonl


GOAL_KEYS={'schema_version','condition','map','manifest_sha256','goals'}


def export_goals(manifest):
    if manifest.get('condition')!='engine_oracle':
        raise ValueError('expected fixed oracle route tasks')
    goals=[dict(run_id=t['episode_id']+'-'+policy,category=t['category'])
           for t in manifest['episodes'] for policy in manifest['policies']]
    result=dict(schema_version=1,condition=CONDITION,map=manifest['map'],
                manifest_sha256=digest_manifest(manifest),goals=goals)
    validate_goals(result)
    return result


def validate_goals(goals):
    if (set(goals)!=GOAL_KEYS or goals['schema_version']!=1 or goals['condition']!=CONDITION
            or not goals['map'].startswith('/Game/SpatialLab/')):
        raise ValueError('replay accepts only goal-category whitelist input, never a full truth task manifest')
    index={}
    for goal in goals['goals']:
        if set(goal)!={'run_id','category'} or goal['category'] not in CATEGORIES:
            raise ValueError('unexpected goal fields or unsupported category')
        key=goal['run_id']
        if not isinstance(key,str) or not key or key in index:
            raise ValueError('invalid or duplicate goal run')
        index[key]=goal['category']
    if not index:
        raise ValueError('empty replay goal set')
    return index


def replay(goals,records,*,memory_ttl_s=5):
    goal_index=validate_goals(goals)
    policies={'frame_only':0,'temporal_memory':memory_ttl_s}
    # Validate the TTL before any record is consumed or output state is changed.
    SemanticMemory('preflight',memory_ttl_s,condition=CONDITION)
    results={}
    selections=[]
    memories={}
    active=None
    header=None
    for row in iter_validated(records):
        kind=row['record_type']
        if kind=='stream':
            header=row
            if header['manifest_sha256']!=goals['manifest_sha256'] or header['map']!=goals['map']:
                raise ValueError('observation stream and goal-only input do not refer to the same frozen route tasks')
        elif kind=='episode_start':
            active=row['run_id']
            if active not in goal_index:
                raise ValueError('unexpected observation episode')
            memories={name:SemanticMemory(active,ttl,condition=CONDITION) for name,ttl in policies.items()}
            results[active]=dict(run_id=active,category=goal_index[active],frames=0,goal_ever_observed=False,
                first_goal_observation_sim_s=None,route_status=None,observation_failure_reason=None,
                by_policy={name:dict(goal_available_frames=0,selection_changes=0,policy_wall_s=0,last_id=None)
                           for name in policies})
        elif kind=='frame':
            result=results[active]
            category=goal_index[active]
            output=dict(run_id=active,frame_id=row['frame_id'],t_sim_s=row['t_sim_s'])
            if any(entity['category']==category for entity in row['entities']):
                if not result['goal_ever_observed']:
                    result['first_goal_observation_sim_s']=row['t_sim_s']
                result['goal_ever_observed']=True
            for name,memory in memories.items():
                start=time.perf_counter()
                memory.update(row)
                selection=memory.select(category)
                elapsed=time.perf_counter()-start
                stats=result['by_policy'][name]
                key=selection['entity_id'] if selection else None
                stats['goal_available_frames']+=selection is not None
                stats['policy_wall_s']+=elapsed
                if result['frames'] and key!=stats['last_id']:
                    stats['selection_changes']+=1
                stats['last_id']=key
                output[name]=selection
            result['frames']+=1
            selections.append(output)
        elif kind=='episode_end':
            results[active]['route_status']=row['status']
            if results[active]['frames']==0:
                results[active]['observation_failure_reason']='no_observation_frames'
            active=None
    for key,category in goal_index.items():
        if key not in results:
            results[key]=dict(run_id=key,category=category,frames=0,goal_ever_observed=False,
                first_goal_observation_sim_s=None,route_status=None,observation_failure_reason='missing_observation_episode',
                by_policy={name:dict(goal_available_frames=0,selection_changes=0,policy_wall_s=0,last_id=None)
                           for name in policies})
    ordered=[results[key] for key in goal_index]
    summaries={}
    for name,ttl in policies.items():
        frames=sum(r['frames'] for r in ordered)
        available=sum(r['by_policy'][name]['goal_available_frames'] for r in ordered)
        wall=sum(r['by_policy'][name]['policy_wall_s'] for r in ordered)
        summaries[name]=dict(ttl_sim_s=ttl,runs=len(ordered),runs_with_frames=sum(r['frames']>0 for r in ordered),
            runs_with_available_goal=sum(r['by_policy'][name]['goal_available_frames']>0 for r in ordered),
            frames=frames,goal_available_frames=available,goal_available_frame_fraction=available/frames if frames else None,
            selection_changes=sum(r['by_policy'][name]['selection_changes'] for r in ordered),
            policy_wall_s=wall,mean_policy_wall_ms=wall*1000/frames if frames else None)
    for result in ordered:
        for stats in result['by_policy'].values():
            del stats['last_id']
    return dict(schema_version=1,evidence_kind='offline_semantic_selection_replay',condition=CONDITION,
                map=header['map'],manifest_sha256=header['manifest_sha256'],attempt_id=header['attempt_id'],
                finished=all(r['route_status'] is not None for r in ordered),
                route_source=header['route_source'],
                interpretation=('oracle-route-biased visibility supervision' if header['route_source']=='global_engine_oracle_selected_routes'
                                else 'observation-dependent routes with engine geometry supervision')
                    +'; offline availability and selection only, not visual recognition or counterfactual navigation success',
                observation_failures=dict(Counter(r['observation_failure_reason'] for r in ordered if r['observation_failure_reason'])),
                by_policy=summaries,runs=ordered,selections=selections)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    export=sub.add_parser('export-goals')
    export.add_argument('tasks',type=Path)
    export.add_argument('--out',type=Path,required=True)
    run=sub.add_parser('replay')
    run.add_argument('--goals',type=Path,required=True)
    run.add_argument('--observations',type=Path,required=True)
    run.add_argument('--ttl-s',type=float,default=5)
    run.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='export-goals':
        result=export_goals(json.loads(args.tasks.read_text(encoding='utf-8-sig')))
    else:
        result=replay(json.loads(args.goals.read_text(encoding='utf-8-sig')),jsonl(args.observations),memory_ttl_s=args.ttl_s)
        with args.observations.open('rb') as source:
            result['observation_stream_sha256']=hashlib.file_digest(source,'sha256').hexdigest()
    with args.out.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    if args.action=='replay':
        print(json.dumps(result['by_policy'],indent=2))
