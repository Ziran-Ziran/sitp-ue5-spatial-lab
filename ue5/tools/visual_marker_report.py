"""Audit pixel-only marker actions, then score independent fixture truth."""
from collections import Counter
import copy
import json
import math
from pathlib import Path
import uuid
from depth_execution_report import metric,distance_xy,segment_distance
from online_depth_report import summarize
from pitch_sweep_protocol import VIEWS,rotation_matches
from semantic_memory import position
from visual_marker import CATEGORIES
from visual_marker_depth_policy import decide
from visual_marker_tasks import CONDITION,POSE_SOURCE,validate_tasks


def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def without_cpu(response):
    result=copy.deepcopy(response);result.pop('perception_and_planning_wall_s',None)
    if result['geometry'] is not None:result['geometry']['planning'].pop('planner_wall_s',None)
    return result


def local_to_world(point,feet,heading):
    r,f,u=point;angle=math.radians(heading)
    return [feet[0]+100*(f*math.cos(angle)-r*math.sin(angle)),feet[1]+100*(f*math.sin(angle)+r*math.cos(angle)),feet[2]+100*u]


def audit_execution(task,raw,artifacts,attempt,run_index,*,exposure=1):
    if raw['engine_path_requests']!=0:raise ValueError('engine paths forbidden in visual depth condition')
    if type(raw['measurement_started'])!=bool or type(raw['agent_spawned'])!=bool:raise ValueError('explicit native measurement state required')
    if not raw['measurement_started']:
        if (raw['status']=='candidate_reached' or raw['trajectory_cm'] or raw['actions'] or raw['waypoint_events'] or raw['capture_count']!=0
                or any(raw[key] is not None for key in ('initial_position_cm','continuous_path_length_cm','collisions','sim_s','wall_s'))):
            raise ValueError('unmeasured visual task cannot fabricate movement or candidate')
        return dict(candidate_reached=False,measurement_valid=False,collisions=None,path_length_cm=None,wall_s=None,captures=0,scans=0)
    path=[position(p) for p in raw['trajectory_cm']]
    if not path or not raw['agent_spawned']:raise ValueError('measured visual task needs actual agent and trajectory')
    initial,final=position(raw['initial_position_cm']),position(raw['final_position_cm']);length=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
    sim=metric(raw['sim_s']);wall=metric(raw['wall_s'])
    if raw['status']=='wall_timeout' and wall<60:raise ValueError('wall timeout before its actual budget')
    budget_preempt=raw['status']=='wall_timeout' and wall>=60
    if (math.dist(path[0],initial)>.01 or math.dist(path[-1],final)>.01 or abs(metric(raw['continuous_path_length_cm'])-length)>.01
            or abs(sim-(len(path)-1)/60)>.002 or any(distance_xy(a,b)>6 for a,b in zip(path,path[1:]))):
        raise ValueError('invalid actual marker trajectory, speed or fixed-step time')
    if type(raw['collisions'])!=int or raw['collisions']<0:raise ValueError('invalid marker contacts')
    reset_xy=distance_xy(initial,task['start_cm']);reset_z=abs(initial[2]-task['start_cm'][2])
    for key in ('reset_wall_s','reset_xy_cm','reset_z_cm','capture_wall_s','policy_bridge_wall_s','max_tracking_error_cm'):metric(raw[key])
    if abs(reset_xy-raw['reset_xy_cm'])>.01 or abs(reset_z-raw['reset_z_cm'])>.01:raise ValueError('incorrect actual reset drift')
    actions=raw['actions'];routes={};last_time=-1;heading=task['heading_deg'];empty=0;scans=0;candidate=None;selected_points=[]
    if raw['capture_count']!=len(actions) or not 0<=len(actions)<=12:raise ValueError('incomplete or unbounded visual observations')
    if not actions and (len(path)!=1 or sim!=0 or length!=0):raise ValueError('movement without visual observation')
    if actions and abs(actions[0]['t_sim_s'])>.002:raise ValueError('visual movement preceded first observation')
    capture_wall=bridge_wall=0
    for step,event in enumerate(actions):
        timestamp=metric(event['t_sim_s']);sample=round(timestamp*60);feet=position(event['feet_cm'])
        if (type(event['step_id'])!=int or event['step_id']!=step or timestamp<=last_time or timestamp>sim+.002
                or abs(timestamp-sample/60)>.002 or not 0<=sample<len(path) or math.dist(feet,path[sample])>.01
                or abs(math.remainder(event['heading_deg']-heading,360))>1e-8):raise ValueError('wrong visual observation time, pose or scan heading')
        last_time=timestamp;capture_wall+=metric(event['capture_wall_s']);bridge_wall+=metric(event.get('bridge_wall_s',0))
        folder=Path(artifacts).absolute()/f'run_{run_index:03d}'/f'step_{step:03d}'
        if Path(event['folder']).absolute()!=folder:raise ValueError('wrong attempt/run/step visual input directory')
        if event['status']!='complete':
            if step!=len(actions)-1 or abs(timestamp-sim)>.002 or math.dist(final,feet)>.01 or raw['status']=='candidate_reached':
                raise ValueError('movement/candidate after observation or worker failure')
            continue
        request=event['request']
        if (request.get('schema_version')!=(3 if 'goal_horizon_policy' in task else 2 if 'planner_variant' in task else 1)
                or request.get('planner_variant')!=task.get('planner_variant')
                or request.get('goal_horizon_policy')!=task.get('goal_horizon_policy')):raise ValueError('changed visual planner/horizon/schema')
        if (request!=read(folder/'request.json') or request['attempt_id']!=attempt or request['run_id']!=task['run_id']
                or request['step_id']!=step or request['t_sim_s']!=timestamp or request['category']!=task['category']
                or request['consecutive_empty_views']!=empty or Path(request['input_dir']).absolute()!=folder/'input'):
            raise ValueError('visual request changes identity, category or scan state')
        captures=event['captures'];eye=list(feet);eye[2]+=150
        if len(captures)!=3:raise ValueError('all current calibrated pitch views required')
        for native,view in zip(captures,VIEWS):
            if (native.get('captured') is not True or native['view']!=view['name'] or native['width']!=320 or native['height']!=240
                    or native['fixed_exposure_ev100']!=exposure or math.dist(position(native['camera_position_cm']),eye)>.001
                    or not rotation_matches(native['camera_rotation_deg'],[-view['pitch_down_deg'],heading,0])):
                raise ValueError('actual marker camera does not match current stopped pose')
        response=event['response']
        if response!=read(folder/'response.json') or without_cpu(response)!=without_cpu(decide(request)):
            raise ValueError('visual target or depth command not reproducible from actual pixels')
        metric(response['perception_and_planning_wall_s'])
        if response['selected'] is not None:
            selected_points.append(dict(step_id=step,category=response['selected']['category'],
                position_cm=local_to_world(response['selected']['position_right_forward_up_m'],feet,heading)))
        action=response['action']
        if action in ('candidate_reached','stop','reject'):
            expected='candidate_reached' if action=='candidate_reached' else ('geometry_'+response['reason'] if action=='reject' else response['reason'])
            if (step!=len(actions)-1 or (raw['status']!=expected and not budget_preempt)
                    or abs(sim-timestamp)>.002 or math.dist(final,feet)>.01):
                raise ValueError('terminal visual action was ignored or followed by uncommanded movement')
            if action=='candidate_reached' and not budget_preempt:
                candidate=dict(category=response['selected']['category'],position_cm=local_to_world(response['selected']['position_right_forward_up_m'],feet,heading),
                    observed_planar_range_m=response['selected']['planar_range_m'],step_id=step)
        elif action=='rotate':
            if empty>=3 or response['relative_yaw_delta_deg']!=90:raise ValueError('invalid or unbounded visual scan')
            heading+=90;empty+=1;scans+=1
        elif action=='move':
            empty=0;route=[local_to_world([r,f,0],feet,heading) for r,f in response['waypoints_right_forward_m']]
            if (not route or math.dist(route[0],feet)>.001 or sum(math.dist(a,b) for a,b in zip(route,route[1:]))>100.001):
                raise ValueError('unbounded or disconnected visual route prefix')
            routes[step]=route
        else:raise ValueError('unknown visual action')
    if abs(capture_wall-raw['capture_wall_s'])>.01 or abs(bridge_wall-raw['policy_bridge_wall_s'])>.01 or capture_wall+bridge_wall>wall+.02:
        raise ValueError('visual episode latency excludes observation or worker')
    events_by_step={step:[] for step in routes};previous=-1
    for event in raw['waypoint_events']:
        step=event['step_id'];index=event['waypoint_index'];timestamp=metric(event['t_sim_s']);sample=round(timestamp*60)
        if (step not in routes or type(index)!=int or index!=len(events_by_step[step]) or index>=len(routes[step]) or timestamp<previous
                or abs(timestamp-sample/60)>.002 or not 0<=sample<len(path) or math.dist(position(event['position_cm']),path[sample])>.01
                or distance_xy(event['position_cm'],routes[step][index])>5.01 or timestamp<actions[step]['t_sim_s']-.002
                or (step+1<len(actions) and timestamp>actions[step+1]['t_sim_s']+.002)):
            raise ValueError('marker waypoint feedback differs from actual pixel command')
        events_by_step[step].append(event);previous=timestamp
    for step,route in routes.items():
        if step+1<len(actions) and len(events_by_step[step])!=len(route):raise ValueError('visual route replaced before reaching its prefix')
    for index,p in enumerate(path):
        applicable=[step for step in range(len(actions)) if actions[step]['t_sim_s']<=index/60+.002]
        if not applicable:continue
        step=max(applicable);event=actions[step]
        if step in routes:
            route=routes[step];segments=list(zip(route,route[1:])) or [(route[0],route[0])]
            limit=16.01 if raw['status']!='candidate_reached' and index==len(path)-1 else 10.01
            if min(segment_distance(p,a,b) for a,b in segments)>limit:raise ValueError('visual failure/success trajectory lacks route support')
        elif distance_xy(p,event['feet_cm'])>.01:raise ValueError('movement while visual action requested stopping or scanning')
    if raw['status']=='candidate_reached':
        if (candidate is None or reset_xy>5.01 or reset_z>10.01 or length>2000.01 or sim>30.002 or wall>60.01
                or raw['max_tracking_error_cm']>10.01 or any(abs(p[2]-initial[2])>50.01 for p in path)):
            raise ValueError('candidate outside actual reset, trajectory or resource budgets')
    return dict(candidate_reached=candidate is not None,candidate=candidate,measurement_valid=True,collisions=raw['collisions'],
        path_length_cm=length,wall_s=wall,captures=len(actions),scans=scans,sim_s=sim,
        capture_wall_s=capture_wall,policy_bridge_wall_s=bridge_wall,selected_points=selected_points)


def associate_selections(points,entities):
    """Post-run metric association only; never passed to the pixel worker."""
    result=[]
    for point in points:
        same=[e for e in entities if e['category']==point['category']]
        nearest=min(((math.dist(point['position_cm'],e['position_cm']),e['entity_id']) for e in same),default=(None,None))
        result.append(dict(step_id=point['step_id'],category=point['category'],nearest_entity_id=nearest[1],
            distance_cm=nearest[0],matched=nearest[0] is not None and nearest[0]<=50))
    return result


def validate_truth(truth,map_path):
    if (set(truth)!={'schema_version','evidence_kind','map','entities'} or truth['schema_version']!=1
            or truth['evidence_kind']!='independent_owned_marker_fixture_truth' or truth['map']!=map_path):raise ValueError('independent same-map marker truth required')
    seen=set()
    for entity in truth['entities']:
        if (set(entity)!={'entity_id','category','position_cm','source'} or entity['entity_id'] in seen or entity['category'] not in CATEGORIES
                or entity['source']!='owned_fixture_material_palette_ground_truth'):raise ValueError('invalid marker truth entity')
        seen.add(entity['entity_id']);position(entity['position_cm'])
    return truth


def evaluate(tasks,record,artifacts,truth):
    validate_tasks(tasks);validate_truth(truth,tasks['map'])
    if (record.get('schema_version')!=tasks['schema_version'] or record['condition']!=CONDITION or record['evidence_kind']!='ue_game_world_rgb_marker_depth_target_search'
            or record['executor']!='CharacterMovement_AddMovementInput' or record['position_reference']!='capsule_feet_cm'
            or record['pose_source']!=POSE_SOURCE or record['map']!=tasks['map'] or record['scope']!=tasks['scope']
            or record['manifest_sha256']!=tasks['manifest_sha256'] or record['expected_runs']!=len(tasks['runs'])):raise ValueError('wrong visual native evidence or task set')
    attempt=str(uuid.UUID(record['attempt_id']))
    if attempt!=record['attempt_id']:raise ValueError('canonical visual attempt required')
    expected={r['run_id']:r for r in tasks['runs']};actual={}
    for row in record['runs']:
        if row['run_id'] not in expected or row['run_id'] in actual or row['input']!=expected[row['run_id']]:raise ValueError('changed/duplicate visual task')
        actual[row['run_id']]=row['execution']
    scores=[]
    for index,task in enumerate(tasks['runs']):
        score=dict(run_id=task['run_id'],category=task['category'],success=False,measurement_valid=False,failure_reason='missing_execution',
            collisions=None,path_length_cm=None,wall_s=None,captures=0,scans=0)
        if task['run_id'] in actual:
            raw=actual[task['run_id']]
            try:
                score.update(audit_execution(task,raw,artifacts,attempt,index,exposure=tasks['fixed_exposure_ev100']));score['failure_reason']=raw['status']
                entities=[e for e in truth['entities'] if not (task['environment_case']=='red_hidden' and e['category']=='red_marker')]
                score['selection_associations']=associate_selections(score.pop('selected_points',[]),entities)
                if score['candidate_reached']:
                    entities=[e for e in truth['entities'] if e['category']==task['category']
                        and not (task['environment_case']=='red_hidden' and e['category']=='red_marker')]
                    matches=[e for e in entities if math.dist(score['candidate']['position_cm'],e['position_cm'])<=50
                        and math.dist(raw['final_position_cm'],e['position_cm'])<=tasks['success_radius_cm']]
                    score.update(success=bool(matches),failure_reason=None if matches else 'candidate_not_near_correct_active_true_marker',
                        matched_entity_id=matches[0]['entity_id'] if matches else None)
            except (ValueError,KeyError,TypeError,IndexError,OSError) as error:score.update(failure_reason='invalid_execution',validation_error=str(error))
        scores.append(score)
    associations=[a for row in scores for a in row.get('selection_associations',[])]
    perception=dict(selected_observations=len(associations),true_beacon_associations=sum(a['matched'] for a in associations),
        nonmatching_color_selections=sum(not a['matched'] for a in associations),
        runs_with_true_selection=sum(any(a['matched'] for a in row.get('selection_associations',[])) for row in scores),
        interpretation='post-run 50cm metric association to active same-category beacon truth; not visible-target recall or calibrated confidence')
    result=dict(schema_version=tasks['schema_version'],planner_variant=tasks.get('planner_variant','legacy_centers'),condition=CONDITION,attempt_id=attempt,manifest_sha256=tasks['manifest_sha256'],
        finished=record.get('finished') is True and len(actual)==len(expected),overall=summarize(scores),perception_associations=perception,
        by_category={category:summarize([r for r in scores if r['category']==category]) for category in sorted(CATEGORIES)},runs=scores,
        interpretation='controlled red/blue RGB-marker categories, optical-depth goals and direct movement; engine pose for reset/actuation only, synthetic fixture, no furniture/generalization or real-time claim')
    if tasks['schema_version']==3:result['goal_horizon_policy']=tasks['goal_horizon_policy']
    return result
