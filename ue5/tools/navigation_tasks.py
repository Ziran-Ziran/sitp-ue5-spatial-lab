"""Freeze a small local oracle task suite before querying target reachability.

Only engine-ready spawn locations are selected, using a fixed seed over sorted
positions. No route success or endpoint score participates in task selection.
The generated manifest contains engine metadata and stays local/ignored.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random


def digest_manifest(manifest):
    return hashlib.sha256(json.dumps(manifest,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def build_manifest(probe):
    if probe.get('condition')!='engine_navmesh_and_collision_oracle' or probe.get('navigation',{}).get('populated_tiles',0)<=0:
        raise ValueError('requires a measured, populated engine NavMesh')
    if not probe.get('saved_map','').startswith('/Game/SpatialLab/'):
        raise ValueError('requires separate saved experiment map')
    entities=[]
    spawns=set()
    for target in sorted(probe['targets'],key=lambda x:x['entity_id']):
        approaches=[a['nav_projected_cm'] for a in target['approaches'] if a['status']=='ready']
        entities.append(dict(entity_id=target['entity_id'],category=target['category'],
                             location_cm=target['location_cm'],approaches_cm=sorted(approaches),
                             source='engine_actor_name_oracle'))
    # Freeze difficulty before scoring: ground-level grid spawns outside every
    # target's success neighborhood, and near an annotated level in Z. This does
    # not test target reachability, discard routes or optimize success rates.
    for candidate in probe.get('probes',[]):
        if candidate['status']!='ready':
            continue
        point=candidate['nav_projected_cm']
        if (all(math.dist(point,e['location_cm'])>=500 for e in entities)
                and any(abs(point[2]-e['location_cm'][2])<=100 for e in entities)):
            spawns.add(tuple(point))
    if len(spawns)<3:
        raise ValueError('need three distinct measured spawn locations')
    starts=random.Random(20261026).sample(sorted(spawns),3)
    episodes=[dict(episode_id=f'start-{i:02d}-{category}',category=category,start_cm=list(start),
                   max_sim_s=60,max_path_cm=20000,success_radius_cm=300,arrival_tolerance_cm=30)
              for i,start in enumerate(starts) for category in ('chair','lamp')]
    return dict(schema_version=1,condition='engine_oracle',evidence_kind='task_definition_not_execution',
                map=probe['saved_map'],units='centimeters',coordinate_system='UE left-handed X-forward Y-right Z-up',
                semantic_source='engine_actor_name_oracle',spawn_source='engine_floor_capsule_and_navmesh_queries',
                spawn_rule=dict(min_target_distance_cm=500,max_nearest_annotated_level_height_difference_cm=100),
                seed=20261026,policies=['euclidean_nearest','shortest_reachable'],
                agent=dict(radius_cm=34,half_height_cm=88,max_speed_cm_s=300,max_step_cm=35,max_slope_degrees=45),
                selection_budget='nearest: one path query; shortest_reachable: every valid approach; unequal compute reported',
                success_definition='arrived within 300cm Euclidean distance of selected category-annotated actor origin',
                entities=entities,episodes=episodes)


def validate_plan(manifest,plan):
    """Reject edits, omitted/duplicate episodes and malformed native input."""
    if (plan.get('condition')!='engine_oracle' or plan.get('manifest_sha256')!=digest_manifest(manifest)
            or plan.get('map')!=manifest.get('map')):
        raise ValueError('plan condition or frozen manifest hash mismatch')
    tasks={t['episode_id']:t for t in manifest['episodes']}
    expected={episode+'-'+policy for episode in tasks for policy in manifest['policies']}
    rows=plan.get('runs',[])
    ids=[r['run_id'] for r in rows]
    if len(ids)!=len(set(ids)) or set(ids)!=expected:
        raise ValueError('plan must contain every policy/episode exactly once')
    entities={e['entity_id']:e for e in manifest['entities']}
    def point(p):
        if not isinstance(p,list) or len(p)!=3 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in p):
            raise ValueError('invalid finite position')
        return p
    for row in rows:
        task=row['task']
        if (task!=tasks.get(task['episode_id']) or row['policy'] not in manifest['policies']
                or row['run_id']!=task['episode_id']+'-'+row['policy']):
            raise ValueError('plan changes frozen task or policy')
        point(task['start_cm'])
        decision=row['decision']
        if decision.get('condition')!='engine_oracle' or decision.get('policy')!=row['policy']:
            raise ValueError('decision provenance mismatch')
        if not isinstance(decision.get('status'),str) or not decision['status']:
            raise ValueError('missing decision status')
        if decision['status']=='complete':
            entity=entities.get(decision.get('entity_id'))
            if not entity or entity['category']!=task['category'] or decision['goal_cm'] not in entity['approaches_cm']:
                raise ValueError('goal is not an annotated approach for this category')
            points=decision['points_cm']
            if not points or any(point(p) is None for p in points):
                raise ValueError('empty path')
            if math.dist(points[0],task['start_cm'])>5 or math.dist(points[-1],decision['goal_cm'])>5:
                raise ValueError('path endpoints differ from task')
    return True


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('probe',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    manifest=build_manifest(json.loads(args.probe.read_text(encoding='utf-8-sig')))
    with args.out.open('x',encoding='utf-8') as stream:
        json.dump(manifest,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(episodes=len(manifest['episodes']),condition=manifest['condition'],sha256=digest_manifest(manifest))))
