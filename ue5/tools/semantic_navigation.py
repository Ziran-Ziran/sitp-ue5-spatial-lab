"""Deterministic target-selection baselines and independent endpoint scoring.

All coordinates here are UE centimeters, left-handed X-forward/Y-right/Z-up.
This version accepts engine_actor_name_oracle entities only, with all positions
explicitly privileged. It implements no visual semantic recognition.
"""
from collections import Counter
import math
import time


def position(value):
    if not isinstance(value, (tuple,list)) or len(value) != 3 or any(
            isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in value):
        raise ValueError('expected finite three-coordinate position in centimeters')
    return value


def distance(a,b):
    return math.dist(position(a),position(b))


def path_length(points):
    for point in points:
        position(point)
    return sum(distance(a,b) for a,b in zip(points,points[1:]))


def checked_path(query,start,goal):
    result = query(start,goal)
    status = result['status']
    if status != 'complete':
        return dict(status=status,points_cm=[])
    points = result.get('points_cm',[])
    try:
        length = path_length(points)
        if not points or distance(points[0],start)>5 or distance(points[-1],goal)>5:
            raise ValueError('path endpoint mismatch')
    except (ValueError,TypeError):
        return dict(status='invalid_path',points_cm=[])
    return dict(status='complete',points_cm=points,path_length_cm=length)


def choose_target(entities,start,category,policy,path_query,condition='engine_oracle'):
    """Use fixed target poses/approaches. Failed path queries remain countable.

    euclidean_nearest makes one query for the nearest entity and its nearest
    approach, preserving its unreachable outcome. shortest_reachable queries all
    approaches and picks the minimum complete path; this has a larger, reported
    query budget. Neither is an equal-compute comparison or a visual condition.
    """
    position(start)
    if condition != 'engine_oracle':
        raise ValueError('engine metadata is allowed only in the explicit engine_oracle condition')
    if policy not in ('euclidean_nearest','shortest_reachable'):
        raise ValueError('unknown target selection policy')
    ids = [e['entity_id'] for e in entities]
    if len(set(ids)) != len(ids):
        raise ValueError('duplicate entity identifiers')
    for entity in entities:
        if entity['source'] != 'engine_actor_name_oracle':
            raise ValueError('unsupported semantic provenance')
        position(entity['location_cm'])
        for point in entity['approaches_cm']:
            position(point)
    started = time.perf_counter()
    output = dict(condition=condition,policy=policy,category=category,path_queries=0,
                  candidate_failures={},entity_id=None,points_cm=[])
    candidates = sorted((e for e in entities if e['category']==category),
                        key=lambda e:(distance(start,e['location_cm']),e['entity_id']))
    if not candidates:
        return dict(output,status='category_absent',selection_wall_s=time.perf_counter()-started)
    if policy=='euclidean_nearest':
        candidates = candidates[:1]
    failures = Counter()
    best = None
    for entity in candidates:
        approaches = sorted(entity['approaches_cm'],key=lambda point:(distance(start,point),tuple(point)))
        if policy=='euclidean_nearest':
            approaches = approaches[:1]
        if not approaches:
            failures['no_valid_approach'] += 1
            if policy=='euclidean_nearest':
                output.update(entity_id=entity['entity_id'],status='no_valid_approach')
        for goal in approaches:
            output['path_queries'] += 1
            path = checked_path(path_query,start,goal)
            if path['status']!='complete':
                failures[path['status']] += 1
                if policy=='euclidean_nearest':
                    output.update(path,entity_id=entity['entity_id'],goal_cm=goal)
                continue
            key = (path['path_length_cm'],entity['entity_id'],tuple(goal))
            if best is None or key < best[0]:
                best = (key,dict(path,entity_id=entity['entity_id'],goal_cm=goal))
    if best:
        output.update(best[1])
    elif 'status' not in output:
        output['status']='no_reachable_target'
    output.update(candidate_failures=dict(failures),selection_wall_s=time.perf_counter()-started)
    return output


def score_episode(task,execution,truth_entities):
    """Score terminal pose against separate truth; do not trust 'success' flags.

    This endpoint metric uses distance to the chosen annotated actor origin. It
    does not assess contact with a physical object or visual category accuracy.
    Failures (including missing execution/pose) stay in the denominator.
    """
    truth = {e['entity_id']:e for e in truth_entities}
    entity = truth.get(execution.get('entity_id'))
    trajectory = execution.get('trajectory_cm',[])
    reason = None
    endpoint_distance = None
    try:
        length = path_length(trajectory) if trajectory else None
        if execution.get('status') not in ('arrived','success'):
            reason=execution.get('status','missing_execution')
        elif not entity:
            reason='unknown_target'
        elif entity['category']!=task['category']:
            reason='wrong_category'
        else:
            endpoint_distance = distance(execution['final_position_cm'],entity['location_cm'])
            if not trajectory or distance(trajectory[-1],execution['final_position_cm'])>5:
                reason='invalid_execution'
            elif endpoint_distance > task['success_radius_cm']:
                reason='outside_goal_radius'
    except (ValueError,TypeError,KeyError):
        length=None
        reason='invalid_execution'
    collisions = execution.get('collisions')
    if collisions is not None and (isinstance(collisions,bool) or not isinstance(collisions,int) or collisions<0):
        raise ValueError('collisions must be a nonnegative count or unknown/null')
    wall = execution.get('wall_s')
    if wall is not None and (not math.isfinite(wall) or wall<0):
        raise ValueError('wall latency must be finite and nonnegative or null')
    return dict(episode_id=task['episode_id'],success=reason is None,failure_reason=reason,
                endpoint_distance_cm=endpoint_distance,path_length_cm=length,collisions=collisions,wall_s=wall)


def summarize_scores(scores):
    count = len(scores)
    observed_collisions = [s['collisions'] for s in scores if s['collisions'] is not None]
    lengths = [s['path_length_cm'] for s in scores if s['path_length_cm'] is not None]
    latencies = [s['wall_s'] for s in scores if s['wall_s'] is not None]
    return dict(episodes=count,successes=sum(s['success'] for s in scores),
                success_rate=sum(s['success'] for s in scores)/count if count else None,
                failures=dict(Counter(s['failure_reason'] for s in scores if not s['success'])),
                collision_observed_episodes=len(observed_collisions),collision_events=sum(observed_collisions),
                path_length_observed_episodes=len(lengths),total_path_length_cm=sum(lengths),
                latency_observed_episodes=len(latencies),mean_wall_s=sum(latencies)/len(latencies) if latencies else None)
