"""Posthoc scoring: annotations never enter the live search policy or runner."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from navigation_tasks import digest_manifest
from semantic_memory import CONDITION,position
from semantic_navigation import score_episode,summarize_scores
from semantic_observation_stream import inspect_records,jsonl
from semantic_search_policy import POLICIES
from semantic_search_tasks import validate_bundle
from search_trace_audit import audit


def candidate_provenance(state,frames,policy):
    trace=state.get('policy_trace',[])
    if not trace:return False
    terminal=trace[-1]
    action=terminal['action']
    source_id=action.get('source_frame_id')
    if (action.get('kind')!='stop' or action.get('reason')!='candidate_reached'
            or action.get('entity_id')!=state.get('entity_id') or not isinstance(source_id,int)
            or source_id>terminal['frame_id'] or source_id not in frames or terminal['frame_id'] not in frames):
        return False
    source=frames[source_id]
    current=frames[terminal['frame_id']]
    if (current['t_sim_s']-source['t_sim_s']>POLICIES[policy]+1e-6
            or abs(current['t_sim_s']-terminal['t_sim_s'])>1e-4):return False
    for entity in source['entities']:
        if entity['entity_id']==state.get('entity_id'):
            return (math.dist(position(entity['position_cm']),position(action['entity_position_cm']))<1e-4
                    and math.dist(position(entity['position_cm']),position(action['goal_cm']))<1e-4)
    return False


def evaluate(bundle,truth,record,observations):
    validate_bundle(bundle)
    if (digest_manifest(truth)!=bundle['evaluation_manifest_sha256'] or truth['map']!=bundle['map']
            or record.get('condition')!=CONDITION or record.get('manifest_sha256')!=bundle['manifest_sha256']
            or record.get('map')!=bundle['map'] or record.get('units')!='centimeters'
            or record.get('position_reference')!='capsule_feet'
            or record.get('evidence_kind')!='ue_game_world_observation_dependent_search'
            or record.get('geometry_source')!='engine_navmesh_exploration_and_paths'):
        raise ValueError('search execution, frozen missions and independent annotation reference mismatch')
    # A new digest must not legitimize dropping a difficult policy/task or
    # changing its budget while still claiming the original annotation suite.
    frozen={episode['episode_id']+'-'+policy:(episode,policy)
            for episode in truth['episodes'] for policy in POLICIES}
    if set(frozen)!={mission['run_id'] for mission in bundle['missions']}:
        raise ValueError('mission set differs from every frozen episode and search policy')
    task_fields=('category','start_cm','max_sim_s','max_path_cm','arrival_tolerance_cm','success_radius_cm')
    for mission in bundle['missions']:
        episode,policy=frozen[mission['run_id']]
        if mission['policy']!=policy or any(mission[field]!=episode[field] for field in task_fields):
            raise ValueError('search mission changes a frozen category, start or budget')
    # Evaluation is explicitly posthoc; it can index the completed raw stream.
    records=list(observations)
    inspect_records(records,record)
    if records[0]['route_source']!='observation_dependent_engine_navmesh_geometry':
        raise ValueError('search scoring requires observation-dependent route evidence')
    frames={}
    for frame in records:
        if frame['record_type']=='frame':frames.setdefault(frame['run_id'],{})[frame['frame_id']]=frame
    expected={mission['run_id']:mission for mission in bundle['missions']}
    rows={}
    for row in record['runs']:
        key=row['run_id']
        if key in rows or key not in expected or row['mission']!=expected[key] or row['policy']!=expected[key]['policy']:
            raise ValueError('duplicate, unexpected or changed search mission execution')
        rows[key]=row
    scores=[]
    for key,mission in expected.items():
        state=rows.get(key,{}).get('execution',dict(status='missing_execution',trajectory_cm=[],collisions=None,wall_s=None))
        for_scoring=copy.deepcopy(state)
        if state['status']=='candidate_reached':for_scoring['status']='arrived'
        elif state['status'] in {'arrived','success'}:for_scoring['status']='unsupported_search_success_status'
        task=dict(mission,episode_id=key)
        score=score_episode(task,for_scoring,truth['entities'])
        if state['status']=='candidate_reached' and not candidate_provenance(state,frames.get(key,{}),mission['policy']):
            score.update(success=False,failure_reason='invalid_observation_provenance')
        trace_audit=audit(mission,state,frames.get(key,{})) if key in rows else None
        if trace_audit is not None and not trace_audit['passed'] and score['failure_reason']!='invalid_observation_provenance':
            score.update(success=False,failure_reason='invalid_policy_trace')
        score.update(run_id=key,policy=mission['policy'],status=state['status'],sim_s=state.get('sim_s'),
            policy_wall_s=state.get('policy_wall_s'),navigation_requests=state.get('navigation_requests'),
            observation_frames=len(frames.get(key,{})),policy_steps=len(state.get('policy_trace',[])),
            policy_trace_audit=trace_audit)
        scores.append(score)
    def summary(group):
        result=summarize_scores(group)
        queries=[s['navigation_requests'] for s in group if s['navigation_requests'] is not None]
        policy_times=[s['policy_wall_s'] for s in group if s['policy_wall_s'] is not None]
        result.update(total_navigation_requests=sum(queries),navigation_request_observed_episodes=len(queries),
            policy_latency_observed_episodes=len(policy_times),total_policy_wall_s=sum(policy_times),
            observation_frames=sum(s['observation_frames'] for s in group),policy_steps=sum(s['policy_steps'] for s in group))
        return result
    return dict(schema_version=1,condition=CONDITION,evidence_kind='posthoc_observed_search_score',
        map=bundle['map'],manifest_sha256=bundle['manifest_sha256'],attempt_id=record['attempt_id'],
        evaluation_manifest_sha256=bundle['evaluation_manifest_sha256'],
        finished=record.get('finished') is True and len(rows)==len(expected),
        interpretation='observed-only semantic selection with engine labels, poses and NavMesh geometry; no visual recognition',
        latency_definition='episode wall time includes online policy bridge and navigation; excludes engine startup; simulated time separate',
        by_policy={policy:summary([s for s in scores if s['policy']==policy]) for policy in POLICIES},runs=scores)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--missions',type=Path,required=True)
    parser.add_argument('--truth',type=Path,required=True)
    parser.add_argument('--execution',type=Path,required=True)
    parser.add_argument('--observations',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
    result=evaluate(read(args.missions),read(args.truth),read(args.execution),jsonl(args.observations))
    result['raw_execution_sha256']=hashlib.sha256(args.execution.read_bytes()).hexdigest()
    with args.out.open('x',encoding='utf-8') as output:json.dump(result,output,indent=2,allow_nan=False)
    print(json.dumps(result['by_policy'],indent=2))
