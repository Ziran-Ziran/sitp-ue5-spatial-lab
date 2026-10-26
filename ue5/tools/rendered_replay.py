"""Frozen offline camera-pose replay, separate from live search and recognition."""
import argparse
import copy
import hashlib
import json
import math
import re
import uuid
from pathlib import Path
from semantic_memory import finite,position
from semantic_observation_stream import iter_validated,jsonl
from replay_source_binding import binding


CALIBRATION=dict(w=320,h=240,fx=160,fy=160,cx=159.5,cy=119.5,
                 hfov_degrees=90,pixel_coordinates='integer_pixel_centers')
DEPTH=dict(encoding='uint16_png',unit='millimeters',meaning='optical_axis_z',
           invalid_values=[0,65535],maximum_valid_mm=30000)
PLAN_KEYS={'schema_version','evidence_kind','condition','map','source_attempt_id','source_manifest_sha256',
           'source_stream_sha256','pose_source','selection_rule','frames_per_episode','calibration','depth',
           'episodes','plan_sha256','source_provenance','render_settings','world_state_scope'}
RENDER_SETTINGS=dict(fixed_exposure_ev100=1,exposure_source='explicit_scene_or_synthetic_control_setting',
                     luminance_units='EV100',luminance_override_scope='render_process_only')


def digest(plan):
    body={key:value for key,value in plan.items() if key!='plan_sha256'}
    return hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def selected_indices(count,limit):
    if count<=limit:return list(range(count))
    return [round(index*(count-1)/(limit-1)) for index in range(limit)]


def build_plan(records,source_stream_sha256,*,source_provenance,frames_per_episode=4,exposure_ev100=1):
    if isinstance(frames_per_episode,bool) or not isinstance(frames_per_episode,int) or not 2<=frames_per_episode<=8:
        raise ValueError('bounded uniform replay requires 2..8 requested frames per episode')
    episodes=[]
    active=None
    for row in iter_validated(records):
        kind=row['record_type']
        if kind=='stream':
            header=row
            if row['route_source']!='observation_dependent_engine_navmesh_geometry':
                raise ValueError('this replay requires an observed-search source stream')
        elif kind=='episode_start':
            active=dict(run_id=row['run_id'],source_frames=0,source_status=None,frames=[])
            candidates=[]
            previous=None
            yaw=0
        elif kind=='frame':
            feet=position(row['observer_position_cm'])
            if previous is not None:
                dx,dy=feet[0]-previous[0],feet[1]-previous[1]
                if math.hypot(dx,dy)>.1:yaw=math.degrees(math.atan2(dy,dx))
            candidates.append(dict(source_frame_id=row['frame_id'],t_source_sim_s=row['t_sim_s'],
                camera_position_cm=position(row['sensor_eye_cm']),camera_rotation_deg=[0,yaw,0]))
            previous=feet
        elif kind=='episode_end':
            active.update(source_frames=len(candidates),source_status=row['status'])
            active['frames']=[dict(candidates[index],frame_id=number)
                              for number,index in enumerate(selected_indices(len(candidates),frames_per_episode))]
            episodes.append(active)
            if len(episodes)>100:raise ValueError('bounded replay permits at most 100 source episodes')
    plan=dict(schema_version=1,evidence_kind='rendered_pose_replay_plan_not_live_sensor',condition='rendered_pose_replay',
        map=header['map'],source_attempt_id=header['attempt_id'],source_manifest_sha256=header['manifest_sha256'],
        source_stream_sha256=source_stream_sha256,pose_source='replayed_capsule_eye_with_causal_motion_heading_initial_yaw_zero',
        selection_rule='uniform_by_source_frame_index_including_endpoints',frames_per_episode=frames_per_episode,
        calibration=copy.deepcopy(CALIBRATION),depth=copy.deepcopy(DEPTH),episodes=episodes,
        source_provenance=copy.deepcopy(source_provenance),
        render_settings=dict(RENDER_SETTINGS,fixed_exposure_ev100=exposure_ev100),
        world_state_scope='source_map_bytes_matched; referenced_assets_and_lighting_as_currently_loaded')
    plan['plan_sha256']=digest(plan)
    return validate_plan(plan)


def validate_plan(plan):
    if (set(plan)!=PLAN_KEYS or plan['schema_version']!=1 or plan['condition']!='rendered_pose_replay'
            or plan['evidence_kind']!='rendered_pose_replay_plan_not_live_sensor'
            or plan['calibration']!=CALIBRATION or plan['depth']!=DEPTH
            or plan['pose_source']!='replayed_capsule_eye_with_causal_motion_heading_initial_yaw_zero'
            or plan['selection_rule']!='uniform_by_source_frame_index_including_endpoints'
            or not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',plan['map']) or plan['plan_sha256']!=digest(plan)):
        raise ValueError('invalid or changed frozen rendered replay plan')
    try:attempt=str(uuid.UUID(plan['source_attempt_id']))
    except (ValueError,TypeError,AttributeError):raise ValueError('invalid source attempt') from None
    if attempt!=plan['source_attempt_id']:raise ValueError('source attempt must be canonical UUID')
    if (set(plan['source_provenance'])!={'map_sha256','job_sha256','execution_sha256'}
            or plan['world_state_scope']!='source_map_bytes_matched; referenced_assets_and_lighting_as_currently_loaded'):
        raise ValueError('missing source map/job/execution binding')
    for value in plan['source_provenance'].values():
        if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('invalid source binding hash')
    validate_render_settings(plan['render_settings'])
    for key in ('source_manifest_sha256','source_stream_sha256'):
        value=plan[key]
        if not isinstance(value,str) or len(value)!=64 or any(char not in '0123456789abcdef' for char in value):
            raise ValueError('invalid source provenance hash')
    limit=plan['frames_per_episode']
    if isinstance(limit,bool) or not isinstance(limit,int) or not 2<=limit<=8:raise ValueError('unbounded replay')
    if not isinstance(plan['episodes'],list) or not 1<=len(plan['episodes'])<=100:raise ValueError('invalid episode count')
    seen=set()
    for episode in plan['episodes']:
        if set(episode)!={'run_id','source_frames','source_status','frames'} or episode['run_id'] in seen:
            raise ValueError('invalid or duplicate replay episode')
        seen.add(episode['run_id'])
        count=episode['source_frames']
        if isinstance(count,bool) or not isinstance(count,int) or not 0<=count<=2000:raise ValueError('unbounded source frame count')
        expected=selected_indices(count,limit)
        if len(episode['frames'])!=len(expected):raise ValueError('missing selected replay frames')
        last=-1
        for index,frame in enumerate(episode['frames']):
            if (set(frame)!={'frame_id','source_frame_id','t_source_sim_s','camera_position_cm','camera_rotation_deg'}
                    or frame['frame_id']!=index or frame['source_frame_id']!=expected[index]
                    or finite(frame['t_source_sim_s'])<0 or finite(frame['t_source_sim_s'])<=last):raise ValueError('invalid replay ordering')
            position(frame['camera_position_cm'])
            rotation=position(frame['camera_rotation_deg'])
            if rotation[0]!=0 or rotation[2]!=0 or not -180<=rotation[1]<=180:raise ValueError('invalid causal heading')
            last=frame['t_source_sim_s']
    return plan


def algorithm_metadata(plan):
    validate_plan(plan)
    return dict(schema_version=1,modality='rgbd_offline_pose_replay',calibration=copy.deepcopy(plan['calibration']),
        depth=copy.deepcopy(plan['depth']),render_settings=copy.deepcopy(plan['render_settings']),time_source='replay_source_simulated_seconds',
        timestamp_interpretation='saved engine episode time; not physical sensor sampling or render throughput')


def validate_render_settings(settings):
    if (set(settings)!=set(RENDER_SETTINGS)
            or any(settings[key]!=RENDER_SETTINGS[key] for key in RENDER_SETTINGS if key!='fixed_exposure_ev100')
            or not -10<=finite(settings['fixed_exposure_ev100'])<=20):
        raise ValueError('invalid explicitly frozen exposure setting')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations',type=Path,required=True)
    parser.add_argument('--frames-per-episode',type=int,default=4)
    parser.add_argument('--source-job',type=Path,required=True)
    parser.add_argument('--exposure-ev100',type=float,default=1,help='freeze independently from scene lighting, never navigation truth')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    with args.observations.open('rb') as stream:sha=hashlib.file_digest(stream,'sha256').hexdigest()
    result=build_plan(jsonl(args.observations),sha,source_provenance=binding(args.source_job,args.observations),
                      frames_per_episode=args.frames_per_episode,exposure_ev100=args.exposure_ev100)
    with args.out.open('x',encoding='utf-8') as output:json.dump(result,output,indent=2,allow_nan=False)
