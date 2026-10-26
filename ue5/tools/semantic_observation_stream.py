"""Strict, lazy reader for explicit engine visibility-supervision JSONL.

The reader exposes one current event at a time. It accepts no full catalog or
truth fields, and verifies completion after consuming the footer and file end.
"""
import json
import math
import uuid
from pathlib import Path
from semantic_memory import CONDITION,finite,position,validate_entities


HEADER_KEYS={'record_type','schema_version','condition','semantic_source','pose_source','route_source',
             'visibility_method','units','coordinate_system','map','manifest_sha256','attempt_id',
             'range_cm','sample_period_sim_s','eye_height_cm'}
FRAME_KEYS={'record_type','run_id','frame_id','t_sim_s','t_monotonic_s',
            'observer_position_cm','sensor_eye_cm','entities'}


def jsonl(path):
    with Path(path).open(encoding='utf-8-sig') as stream:
        for line in stream:
            if len(line)>1024*1024 or not line.strip():
                raise ValueError('oversized or empty observation record')
            yield json.loads(line)


def iter_validated(records):
    iterator=iter(records)
    try:
        header=next(iterator)
    except StopIteration:
        raise ValueError('missing observation header') from None
    if (set(header)!=HEADER_KEYS or header['record_type']!='stream' or header['schema_version']!=1
            or header['condition']!=CONDITION or header['semantic_source']!='engine_actor_label'
            or header['pose_source']!='engine_ground_truth' or header['route_source'] not in {
                'global_engine_oracle_selected_routes','observation_dependent_engine_navmesh_geometry'}
            or header['visibility_method']!='panoramic_range_three_bbox_samples_first_hit_target'
            or header['units']!='centimeters' or header['coordinate_system']!='UE left-handed X-forward Y-right Z-up'
            or not header['map'].startswith('/Game/SpatialLab/')):
        raise ValueError('unsupported or undisclosed observation supervision protocol')
    sha=header['manifest_sha256']
    if not isinstance(sha,str) or len(sha)!=64 or any(c not in '0123456789abcdef' for c in sha):
        raise ValueError('invalid frozen manifest hash')
    try:
        canonical=str(uuid.UUID(header['attempt_id']))
    except (ValueError,AttributeError,TypeError):
        raise ValueError('invalid attempt identifier') from None
    if canonical!=header['attempt_id']:
        raise ValueError('attempt identifier must be canonical UUID')
    if not (0<finite(header['range_cm'])<=10000 and 0<finite(header['sample_period_sim_s'])<=10
            and 0<finite(header['eye_height_cm'])<=300):
        raise ValueError('invalid bounded sensor specification')
    yield header
    seen=set()
    active=None
    frame_index=total_frames=0
    last_sim=None
    last_wall=-1
    finished=False
    for row in iterator:
        if finished:
            raise ValueError('records after stream footer')
        kind=row.get('record_type')
        if kind=='episode_start':
            if set(row)!={'record_type','run_id'} or active is not None:
                raise ValueError('nested or malformed observation episode')
            name=row['run_id']
            if not isinstance(name,str) or not name or name in seen:
                raise ValueError('invalid or duplicate observation run')
            active=name
            seen.add(name)
            frame_index=0
            last_sim=None
        elif kind=='frame':
            if set(row)!=FRAME_KEYS or active is None or row['run_id']!=active:
                raise ValueError('frame fields, run or episode mismatch')
            index=row['frame_id']
            sim,wall=finite(row['t_sim_s']),finite(row['t_monotonic_s'])
            if (isinstance(index,bool) or not isinstance(index,int) or index!=frame_index or sim<0 or wall<0
                    or (last_sim is not None and sim<=last_sim) or wall<=last_wall):
                raise ValueError('noncontiguous frame IDs or nonmonotonic clocks')
            feet,eye=position(row['observer_position_cm']),position(row['sensor_eye_cm'])
            expected_eye=[feet[0],feet[1],feet[2]+header['eye_height_cm']]
            if math.dist(eye,expected_eye)>1e-3:
                raise ValueError('sensor eye and capsule feet reference mismatch')
            entities=validate_entities(row['entities'])
            if any(math.dist(eye,e['position_cm'])>header['range_cm']+1e-3 for e in entities.values()):
                raise ValueError('entity outside declared observation range')
            frame_index+=1
            total_frames+=1
            last_sim,last_wall=sim,wall
        elif kind=='episode_end':
            if (set(row)!={'record_type','run_id','status','frames'} or active is None or row['run_id']!=active
                    or row['frames']!=frame_index or not isinstance(row['status'],str) or not row['status']):
                raise ValueError('incomplete or malformed observation episode end')
            active=None
        elif kind=='stream_end':
            if (set(row)!={'record_type','finished','episodes','frames'} or active is not None
                    or row['finished'] is not True or row['episodes']!=len(seen) or row['frames']!=total_frames):
                raise ValueError('incomplete or inconsistent observation footer')
            finished=True
        else:
            raise ValueError('unknown observation record')
        yield row
    if not finished:
        raise ValueError('observation stream has no completed footer')


def inspect_records(records,execution=None):
    summary=dict(evidence_kind='engine_visibility_supervision_stream',frames=0,empty_frames=0,
                 observed_entity_samples=0,episodes={})
    header=None
    endpoints={}
    for row in iter_validated(records):
        kind=row['record_type']
        if kind=='stream':
            header=row
            summary.update(map=row['map'],manifest_sha256=row['manifest_sha256'],attempt_id=row['attempt_id'])
        elif kind=='frame':
            bounds=endpoints.setdefault(row['run_id'],[row,row])
            bounds[1]=row
            summary['frames']+=1
            summary['empty_frames']+=not row['entities']
            summary['observed_entity_samples']+=len(row['entities'])
        elif kind=='episode_end':
            summary['episodes'][row['run_id']]=dict(status=row['status'],frames=row['frames'])
    if execution is not None:
        expected={r['run_id']:r['execution']['status'] for r in execution['runs']}
        if (not execution.get('finished') or execution['map']!=header['map']
                or execution['manifest_sha256']!=header['manifest_sha256']
                or execution.get('attempt_id')!=header['attempt_id']
                or len(expected)!=len(execution['runs'])
                or {k:v['status'] for k,v in summary['episodes'].items()}!=expected):
            raise ValueError('observation stream does not match the completed execution')
        for run in execution['runs']:
            if run['run_id'] not in endpoints:
                continue
            first,last=endpoints[run['run_id']]
            actual=run['execution']
            if (math.dist(position(first['observer_position_cm']),position(actual['initial_position_cm']))>5
                    or math.dist(position(last['observer_position_cm']),position(actual['final_position_cm']))>5
                    or abs(finite(first['t_sim_s']))>1e-4
                    or abs(finite(last['t_sim_s'])-finite(actual['sim_s']))>1e-4):
                raise ValueError('observation endpoints or simulation time differ from execution')
    summary['finished']=True
    return summary
