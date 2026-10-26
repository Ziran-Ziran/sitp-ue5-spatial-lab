"""Independently replay stopped pixels/actions and score all actual task outcomes."""
from collections import Counter
import json
import math
from pathlib import Path
import uuid
from depth_execution_report import metric,segment_distance,distance_xy
from online_depth_policy import decide,world_goal_to_local
from online_depth_tasks import CONDITION,validate_tasks
from pitch_sweep_protocol import VIEWS,rotation_matches
from semantic_memory import position


def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def without_cpu(answer):
    result={k:v for k,v in answer.items() if k!='geometry_and_planning_wall_s'}
    result['planning']={k:v for k,v in answer['planning'].items() if k!='planner_wall_s'}
    return result


def audit_execution(task,raw,artifacts,attempt,run_index,*,exposure):
    if raw['engine_path_requests']!=0:raise ValueError('engine paths forbidden in depth-only geometry condition')
    measured=raw['measurement_started']
    if type(measured)!=bool or type(raw['agent_spawned'])!=bool:raise ValueError('explicit native measurement state required')
    if not measured:
        if (raw['status']=='arrived' or raw['trajectory_cm'] or raw['actions'] or raw['waypoint_events'] or raw['capture_count']!=0
                or any(raw[key] is not None for key in ('initial_position_cm','continuous_path_length_cm','collisions','sim_s','wall_s'))):
            raise ValueError('unmeasured attempt cannot fabricate motion or success')
        return dict(success=False,measurement_valid=False,collisions=None,path_length_cm=None,wall_s=None,captures=0)
    path=[position(p) for p in raw['trajectory_cm']]
    if not raw['agent_spawned'] or not path:raise ValueError('measured attempt needs actual agent/trajectory')
    initial,final=position(raw['initial_position_cm']),position(raw['final_position_cm'])
    length=sum(math.dist(a,b) for a,b in zip(path,path[1:]));sim=metric(raw['sim_s']);wall=metric(raw['wall_s'])
    if raw['status']=='wall_timeout' and wall<60:raise ValueError('wall timeout before its actual budget')
    budget_preempt=raw['status']=='wall_timeout' and wall>=60
    if (math.dist(initial,path[0])>.01 or math.dist(final,path[-1])>.01
            or abs(metric(raw['continuous_path_length_cm'])-length)>.01 or abs(sim-(len(path)-1)/60)>.002
            or any(distance_xy(a,b)>6 for a,b in zip(path,path[1:]))):raise ValueError('invalid actual trajectory, speed or time')
    if type(raw['collisions'])!=int or raw['collisions']<0:raise ValueError('unknown contact count')
    for key in ('reset_wall_s','reset_xy_cm','reset_z_cm','max_tracking_error_cm','capture_wall_s','policy_bridge_wall_s'):metric(raw[key])
    reset_xy=distance_xy(initial,task['start_cm']);reset_z=abs(initial[2]-task['start_cm'][2])
    if abs(raw['reset_xy_cm']-reset_xy)>.01 or abs(raw['reset_z_cm']-reset_z)>.01:raise ValueError('incorrect reset measurement')
    actions=raw['actions'];routes={};last_time=-1
    if raw['capture_count']!=len(actions) or not 0<=len(actions)<=8:raise ValueError('missing/extra capture feedback')
    if task['mode']=='initial_sweep_once' and len(actions)>1:raise ValueError('one-shot condition captured again')
    if not actions and (len(path)!=1 or length!=0 or sim!=0):raise ValueError('motion without an observed depth command')
    if actions and abs(actions[0]['t_sim_s'])>.002:raise ValueError('motion preceded the initial depth observation')
    capture_wall=bridge_wall=0
    for step,event in enumerate(actions):
        timestamp=metric(event['t_sim_s']);sample=round(timestamp*60)
        if (event['step_id']!=step or type(event['step_id'])!=int or timestamp<=last_time or timestamp>sim+.002
                or abs(timestamp-sample/60)>.002 or not 0<=sample<len(path)
                or math.dist(position(event['feet_cm']),path[sample])>.01):raise ValueError('capture not tied to actual stopped trajectory')
        last_time=timestamp;capture_wall+=metric(event['capture_wall_s']);bridge_wall+=metric(event.get('bridge_wall_s',0))
        folder=Path(artifacts).absolute()/f'run_{run_index:03d}'/f'step_{step:03d}'
        if Path(event['folder']).absolute()!=folder:raise ValueError('capture directory does not belong to this attempt/step')
        if event['status']!='complete':
            if (raw['status']=='arrived' or step!=len(actions)-1 or abs(timestamp-sim)>.002
                    or math.dist(final,event['feet_cm'])>.01):raise ValueError('motion or arrival after failed observation/policy')
            continue
        request=event['request'];expected_goal=world_goal_to_local(event['feet_cm'],task['goal_cm'],task['heading_deg'])
        if (request.get('schema_version')!=(2 if 'query_cell_policy' in task else 1)
                or request.get('query_cell_policy')!=task.get('query_cell_policy')):raise ValueError('changed query index policy/schema')
        if (request!=read(folder/'request.json') or request['attempt_id']!=attempt or request['run_id']!=task['run_id']
                or request['step_id']!=step or request['t_sim_s']!=timestamp or request['mode']!=task['mode']
                or Path(request['input_dir']).absolute()!=folder/'input'
                or math.dist(request['goal_right_forward_m'],expected_goal)>1e-9):raise ValueError('wrong step request or privileged relative task goal')
        captures=event['captures']
        if len(captures)!=3:raise ValueError('complete calibrated three-view observation required')
        eye=list(event['feet_cm']);eye[2]+=150
        for native,view in zip(captures,VIEWS):
            if (native.get('captured') is not True or native['view']!=view['name'] or native['width']!=320 or native['height']!=240
                    or native['fixed_exposure_ev100']!=exposure or math.dist(position(native['camera_position_cm']),eye)>.001
                    or not rotation_matches(native['camera_rotation_deg'],[-view['pitch_down_deg'],task['heading_deg'],0])):
                raise ValueError('native capture pose/calibration does not match current stopped agent')
        response=event['response']
        if response!=read(folder/'response.json') or without_cpu(response)!=without_cpu(decide(request)):
            raise ValueError('saved depth action cannot be reproduced from its actual pixels')
        metric(response['geometry_and_planning_wall_s'])
        metric(response['planning']['planner_wall_s'])
        if response['action']=='reject':
            if (step!=len(actions)-1 or (raw['status']!='geometry_'+response['reason'] and not budget_preempt)
                    or abs(timestamp-sim)>.002 or math.dist(final,event['feet_cm'])>.01):
                raise ValueError('rejected geometry was executed or mislabeled')
            continue
        angle=math.radians(task['heading_deg']);start=event['feet_cm']
        route=[[start[0]+100*(f*math.cos(angle)-r*math.sin(angle)),start[1]+100*(f*math.sin(angle)+r*math.cos(angle)),start[2]]
               for r,f in response['waypoints_right_forward_m']]
        routes[step]=route
    if abs(capture_wall-raw['capture_wall_s'])>.01 or abs(bridge_wall-raw['policy_bridge_wall_s'])>.01:
        raise ValueError('latency totals differ from actual observation/policy calls')
    if capture_wall+bridge_wall>wall+.02:raise ValueError('episode wall excludes observation or policy cost')
    events_by_step={step:[] for step in routes};previous=-1
    for event in raw['waypoint_events']:
        step=event['step_id'];index=event['waypoint_index'];timestamp=metric(event['t_sim_s']);sample=round(timestamp*60)
        if (step not in routes or type(index)!=int or index!=len(events_by_step[step]) or index>=len(routes[step])
                or timestamp<previous or abs(timestamp-sample/60)>.002 or not 0<=sample<len(path)
                or math.dist(position(event['position_cm']),path[sample])>.01
                or distance_xy(event['position_cm'],routes[step][index])>5.01
                or timestamp<actions[step]['t_sim_s']-.002
                or (step+1<len(actions) and timestamp>actions[step+1]['t_sim_s']+.002)):
            raise ValueError('waypoint event is inconsistent with the actual commanded route')
        events_by_step[step].append(event);previous=timestamp
    for step,route in routes.items():
        if (step+1<len(actions) or raw['status']=='arrived') and len(events_by_step[step])!=len(route):
            raise ValueError('route was replaced or declared arrived before waypoint feedback')
    for i,p in enumerate(path):
        applicable=[step for step in range(len(actions)) if actions[step]['t_sim_s']<=i/60+.002]
        if not applicable:continue  # Only the zero-motion, no-observation case can reach here.
        step=max(applicable)
        if step not in routes:continue  # Terminal rejection/failure was checked at its stopped pose above.
        route=routes[step];segments=list(zip(route,route[1:])) or [(route[0],route[0])]
        error=min(segment_distance(p,a,b) for a,b in segments)
        # A failure may terminate on the first tick crossing the tracking limit.
        # One bounded movement tick is allowed only at that terminal sample.
        limit=16.01 if raw['status']!='arrived' and i==len(path)-1 else 10.01
        if error>limit:raise ValueError('actual failure/success movement left its pixel-supported command')
    if raw['status']=='arrived':
        if (reset_xy>5.01 or reset_z>10.01 or distance_xy(final,task['goal_cm'])>10 or abs(final[2]-task['goal_cm'][2])>20
                or length>2000.01 or sim>30.002 or wall>60.01 or raw['max_tracking_error_cm']>10.01
                or any(abs(p[2]-initial[2])>50.01 for p in path)):raise ValueError('arrival outside reset, endpoint or route budgets')
        if not routes and (length!=0 or sim!=0 or distance_xy(initial,task['goal_cm'])>10):raise ValueError('arrival without actual route')
    return dict(success=raw['status']=='arrived',measurement_valid=True,collisions=raw['collisions'],
        path_length_cm=length,wall_s=wall,captures=len(actions),capture_wall_s=capture_wall,policy_bridge_wall_s=bridge_wall,
        sim_s=sim)


def summarize(rows):
    measured=[r for r in rows if r['measurement_valid']];count=len(measured)
    return dict(runs=len(rows),successes=sum(r['success'] for r in rows),success_rate=sum(r['success'] for r in rows)/len(rows) if rows else 0,
        failures=dict(Counter(r['failure_reason'] for r in rows if not r['success'])),measured_runs=count,
        collisions=sum(r['collisions'] for r in measured) if count else None,
        total_path_length_cm=sum(r['path_length_cm'] for r in measured) if count else None,
        captures=sum(r['captures'] for r in rows),mean_episode_wall_s=sum(r['wall_s'] for r in measured)/count if count else None,
        total_capture_wall_s=sum(r['capture_wall_s'] for r in measured) if count else None,
        total_policy_bridge_wall_s=sum(r['policy_bridge_wall_s'] for r in measured) if count else None)


def evaluate(tasks,record,artifacts):
    validate_tasks(tasks)
    if (record.get('schema_version')!=tasks['schema_version'] or record['condition']!=CONDITION or record['evidence_kind']!='ue_game_world_stopped_rgbd_observation_dependent_direct_motion'
            or record['executor']!='CharacterMovement_AddMovementInput' or record['position_reference']!='capsule_feet_cm'
            or record['pose_source']!=tasks['pose_source'] or record['map']!=tasks['map']
            or record['manifest_sha256']!=tasks['manifest_sha256'] or record['scope']!=tasks['scope']
            or record['expected_runs']!=len(tasks['runs'])):raise ValueError('wrong online native execution evidence')
    attempt=str(uuid.UUID(record['attempt_id']))
    if attempt!=record['attempt_id']:raise ValueError('canonical attempt required')
    expected={r['run_id']:r for r in tasks['runs']};actual={}
    for row in record['runs']:
        if row['run_id'] not in expected or row['run_id'] in actual or row['input']!=expected[row['run_id']]:raise ValueError('changed/duplicate online mission')
        actual[row['run_id']]=row['execution']
    scores=[]
    for index,task in enumerate(tasks['runs']):
        score=dict(run_id=task['run_id'],mode=task['mode'],success=False,measurement_valid=False,failure_reason='missing_execution',
            collisions=None,path_length_cm=None,wall_s=None,captures=0)
        if task['run_id'] in actual:
            raw=actual[task['run_id']]
            try:
                score.update(audit_execution(task,raw,artifacts,attempt,index,exposure=tasks['fixed_exposure_ev100']))
                score['failure_reason']=None if score['success'] else raw['status']
            except (ValueError,KeyError,TypeError,IndexError,OSError) as error:
                score.update(failure_reason='invalid_execution',validation_error=str(error))
        scores.append(score)
    return dict(schema_version=tasks['schema_version'],query_cell_policy=tasks.get('query_cell_policy','legacy_floor'),condition=CONDITION,attempt_id=attempt,manifest_sha256=tasks['manifest_sha256'],
        finished=record.get('finished') is True and len(actual)==len(expected),overall=summarize(scores),
        by_mode={mode:summarize([r for r in scores if r['mode']==mode]) for mode in sorted({r['mode'] for r in scores})},runs=scores,
        interpretation='actual stopped sensing and direct depth-route feedback with explicit engine pose supervision; extra receding observations/compute; local geometric tasks, not semantic search or real-time navigation')
