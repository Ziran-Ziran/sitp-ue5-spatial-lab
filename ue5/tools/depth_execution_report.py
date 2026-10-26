"""Independent frozen depth-transfer scoring; retain nonexecuted/missing cases."""
from collections import Counter
import math
import uuid
from depth_execution_plan import validate_bundle,CONDITION
from semantic_memory import finite
from semantic_memory import position


def metric(value):
    value=finite(value)
    if value<0:raise ValueError('negative execution metric')
    return value


def distance_xy(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])


def segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];denominator=dx*dx+dy*dy
    t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denominator)) if denominator else 0
    return math.hypot(p[0]-a[0]-dx*t,p[1]-a[1]-dy*t)


def validate_execution(task,execution,settings):
    if execution.get('engine_path_requests')!=0:raise ValueError('engine path requests are not allowed')
    spawned,measured=execution['agent_spawned'],execution['measurement_started']
    if type(spawned)!=bool or type(measured)!=bool or measured and not spawned:raise ValueError('invalid execution presence flags')
    trajectory=execution['trajectory_cm'];events=execution['waypoint_events']
    if not isinstance(trajectory,list) or not isinstance(events,list):raise ValueError('invalid trajectory/feedback')
    if not measured:
        if trajectory or events or execution.get('initial_position_cm') is not None or any(execution.get(key) is not None for key in
                ('continuous_path_length_cm','collisions','sim_s','wall_s','reset_wall_s','reset_xy_cm','reset_z_cm','max_tracking_error_cm')):
            raise ValueError('unmeasured episode cannot claim zero metrics')
        if task['gate_status']!='approved' and (spawned or execution['status']!='not_executed_'+task['gate_status']):
            raise ValueError('rejected geometry must not execute')
        if execution['status']=='arrived':raise ValueError('arrival requires actual measured movement evidence')
        return
    if task['gate_status']!='approved':raise ValueError('rejected geometry moved')
    initial,final=position(execution['initial_position_cm']),position(execution['final_position_cm'])
    if not trajectory or math.dist(position(trajectory[0]),initial)>.01 or math.dist(position(trajectory[-1]),final)>.01:
        raise ValueError('actual trajectory endpoints missing or inconsistent')
    path=[position(p) for p in trajectory]
    length=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
    if abs(metric(execution['continuous_path_length_cm'])-length)>max(.01,length*1e-5):raise ValueError('path metric differs from raw trajectory')
    sim=metric(execution['sim_s']);metric(execution['wall_s']);metric(execution['reset_wall_s'])
    if abs(sim-(len(path)-1)/60)>.002:raise ValueError('trajectory does not match fixed simulated time')
    if any(distance_xy(a,b)>settings['speed_cm_s']/60+1 for a,b in zip(path,path[1:])):
        raise ValueError('trajectory exceeds the controlled planar speed')
    if type(execution['collisions'])!=int or execution['collisions']<0:raise ValueError('unknown or invalid collision count')
    reset_xy,reset_z=distance_xy(initial,task['start_cm']),abs(initial[2]-task['start_cm'][2])
    if abs(metric(execution['reset_xy_cm'])-reset_xy)>.01 or abs(metric(execution['reset_z_cm'])-reset_z)>.01:
        raise ValueError('reset drift does not match measured initial pose')
    metric(execution['max_tracking_error_cm'])
    last=-1
    waypoints=task['world_waypoints_cm']
    for index,event in enumerate(events):
        timestamp=metric(event['t_sim_s']);sample=round(timestamp*60);location=position(event['position_cm'])
        if (type(event['waypoint_index'])!=int or event['waypoint_index']!=index or index>=len(waypoints)
                or timestamp<last or timestamp>sim+.002 or abs(timestamp-sample/60)>.002 or not 0<=sample<len(path)
                or math.dist(location,path[sample])>.01 or distance_xy(location,waypoints[index])>settings['waypoint_tolerance_cm']+.01):
            raise ValueError('waypoint feedback is not supported by actual ordered trajectory')
        last=timestamp
    if execution['status']=='arrived':
        if (len(events)!=len(waypoints) or reset_xy>settings['reset_xy_limit_cm']+.01 or reset_z>settings['reset_z_limit_cm']+.01
                or distance_xy(final,waypoints[-1])>settings['final_xy_tolerance_cm']
                or abs(final[2]-waypoints[-1][2])>settings['final_z_tolerance_cm']
                or length>settings['max_path_cm']+.01 or sim>settings['max_sim_s']+.002
                or execution['wall_s']>settings['max_wall_s']+.01
                or execution['max_tracking_error_cm']>settings['tracking_limit_cm']+.01
                or any(abs(p[2]-initial[2])>settings['max_vertical_excursion_cm']+.01 for p in path)):
            raise ValueError('arrival is outside reset, endpoint, feedback or route budgets')
        segments=list(zip(waypoints,waypoints[1:])) or [(waypoints[0],waypoints[0])]
        if any(min(segment_distance(p,a,b) for a,b in segments)>settings['tracking_limit_cm']+.01 for p in path):
            raise ValueError('actual route left the permitted tracking corridor')


def summarize(rows):
    measured=[row for row in rows if row['measurement_valid']]
    return dict(runs=len(rows),approved_for_execution=sum(row['gate_status']=='approved' for row in rows),
        measured_runs=len(measured),successes=sum(row['success'] for row in rows),
        success_rate=sum(row['success'] for row in rows)/len(rows) if rows else 0,
        failures=dict(Counter(row['failure_reason'] for row in rows if not row['success'])),
        collision_observed_runs=len(measured),collision_events=sum(row['collisions'] for row in measured) if measured else None,
        path_length_observed_runs=len(measured),total_path_length_cm=sum(row['path_length_cm'] for row in measured) if measured else None,
        latency_observed_runs=len(measured),mean_execution_wall_s=sum(row['wall_s'] for row in measured)/len(measured) if measured else None)


def evaluate(bundle,record,*,bundle_validator=validate_bundle):
    bundle_validator(bundle)
    if (record.get('condition')!=CONDITION or record.get('execution_scope')!=bundle['execution_scope']
            or record.get('manifest_sha256')!=bundle['manifest_sha256'] or record.get('map')!=bundle['map']
            or record.get('evidence_kind')!='ue_game_world_supplied_depth_waypoint_character_movement'
            or record.get('executor')!='CharacterMovement_AddMovementInput'
            or record.get('position_feedback_source')!='engine_ground_truth_for_actuation'
            or record.get('position_reference')!='capsule_feet_cm' or record.get('expected_runs')!=len(bundle['runs'])):
        raise ValueError('wrong native evidence, actuation supervision or frozen depth plan')
    try:attempt=str(uuid.UUID(record['attempt_id']))
    except (KeyError,ValueError,TypeError,AttributeError):raise ValueError('missing native attempt UUID') from None
    if attempt!=record['attempt_id']:raise ValueError('noncanonical native attempt')
    expected={row['run_id']:row for row in bundle['runs']};actual={}
    for row in record['runs']:
        key=row['run_id']
        if key not in expected or key in actual or row.get('input')!=expected[key]:raise ValueError('execution changes or duplicates frozen input')
        actual[key]=row['execution']
    scores=[]
    for key,task in expected.items():
        score=dict(run_id=key,mode=task['mode'],gate_status=task['gate_status'],success=False,measurement_valid=False,
            failure_reason='missing_execution',status='missing_execution',collisions=None,path_length_cm=None,wall_s=None,sim_s=None)
        if key in actual:
            execution=actual[key];score['status']=execution.get('status')
            try:
                validate_execution(task,execution,bundle['settings'])
                score['success']=execution['status']=='arrived'
                score['failure_reason']=None if score['success'] else execution['status']
                if execution['measurement_started']:
                    score.update(measurement_valid=True,collisions=execution['collisions'],path_length_cm=execution['continuous_path_length_cm'],
                        wall_s=execution['wall_s'],sim_s=execution['sim_s'])
            except (ValueError,KeyError,TypeError,IndexError,OverflowError) as error:
                score.update(failure_reason='invalid_execution',validation_error=str(error))
        scores.append(score)
    return dict(schema_version=1,condition=CONDITION,execution_scope=bundle['execution_scope'],attempt_id=attempt,
        manifest_sha256=bundle['manifest_sha256'],finished=record.get('finished') is True and len(actual)==len(expected),
        overall=summarize(scores),by_mode={mode:summarize([row for row in scores if row['mode']==mode]) for mode in sorted({r['mode'] for r in scores})},
        runs=scores,interpretation='actual transfer of frozen depth-local probes with engine-pose reset/actuation; not live sensing, semantic target success or independent search episodes')


def control_acceptance(report,record):
    scores={row['run_id']:row for row in report['runs']};raw={row['run_id']:row['execution'] for row in record['runs']}
    if set(scores)!={'detour','direct-blocked','spawn-blocked','planning-rejected','immediate'}:raise ValueError('all supplied-route controls required')
    checks=dict(detour_arrived=scores['detour']['success'],
        wall_blocked=scores['direct-blocked']['failure_reason']=='blocked_no_progress',
        wall_contact_observed=bool(scores['direct-blocked']['collisions'] and scores['direct-blocked']['collisions']>0),
        bad_spawn_rejected=scores['spawn-blocked']['failure_reason']=='spawn_blocked',
        no_plan_never_spawned=scores['planning-rejected']['failure_reason']=='not_executed_planning_start_unobserved'
            and raw['planning-rejected']['agent_spawned'] is False,
        immediate_arrived=scores['immediate']['success'],
        immediate_zero_motion=raw['immediate']['continuous_path_length_cm']==0 and raw['immediate']['sim_s']==0,
        no_engine_path_requests=all(row['engine_path_requests']==0 for row in raw.values()))
    return dict(evidence_kind='synthetic_character_movement_supplied_route_controls',passed=report['finished'] and all(checks.values()),checks=checks,
        interpretation='physics/controller controls, including deliberate unsafe wall route; not depth-estimation or city success')
