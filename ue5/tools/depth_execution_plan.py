"""Evaluation-side bridge: isolated depth plans -> pose-supervised actuation.

Only the bridge reads saved global transforms. Geometry receives the isolated
input directories. Frozen transfer is not live visual localization/navigation.
"""
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import numpy as np
from local_depth_grid import GridConfig,number
from continuous_depth_path import validate_route,point
from pitch_sweep_capture_report import inspect_sweep_capture
from pitch_sweep_report import evaluate_sweep_sequence
from rendered_input_report import file_hash
from semantic_memory import position

CONDITION='depth_grid_pose_supervised_frozen_path_execution'
SETTINGS=dict(radius_cm=34,half_height_cm=88,speed_cm_s=300,step_cm=35,slope_degrees=45,
    waypoint_tolerance_cm=5,tracking_limit_cm=10,reset_xy_limit_cm=5,reset_z_limit_cm=10,
    max_sim_s=30,max_path_cm=2000,max_wall_s=30,no_progress_sim_s=2,max_vertical_excursion_cm=50,
    final_xy_tolerance_cm=10,final_z_tolerance_cm=20,tracking_margin_m=.1)
GEOMETRY_FILES=('local_depth_grid.py','grid_path.py','continuous_depth_path.py','depth_grid_replay.py',
                'pitch_sweep_report.py','rendered_input_report.py','pitch_sweep_protocol.py','depth_execution_plan.py')
BINDING_KEYS={'sequence','source_run_id','frame_id','source_frame_id','t_source_sim_s','start_cm','heading_deg'}
RUN_KEYS=BINDING_KEYS|{'run_id','mode','local_goal_m','planning_status','gate_status','gate_detail',
                       'local_waypoints_m','world_waypoints_cm'}


def digest_bundle(bundle):
    return hashlib.sha256(json.dumps({key:value for key,value in bundle.items() if key!='manifest_sha256'},
        sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def local_to_world(local,start_cm,heading_deg):
    right,forward=point(local);start=position(start_cm);yaw=math.radians(number(heading_deg))
    return [start[0]+100*(forward*math.cos(yaw)-right*math.sin(yaw)),
            start[1]+100*(forward*math.sin(yaw)+right*math.cos(yaw)),start[2]]


def finish(bundle):
    bundle['manifest_sha256']=digest_bundle(bundle)
    return validate_bundle(bundle)


def build_bundle(capture,*,source_job_binding):
    capture=Path(capture).absolute();inspect_sweep_capture(capture)
    sweep=json.loads((capture/'evaluation/sweep-plan.json').read_text(encoding='utf-8-sig'));base=sweep['base_plan']
    config=GridConfig();bindings=[];runs=[];hashes={}
    for index,episode in enumerate(base['episodes']):
        sequence=f'sequence_{index:03d}'
        # No evaluation path, source pose or category is passed into geometry.
        geometry=evaluate_sweep_sequence(capture/'input'/sequence,config=config)
        hashes.update({sequence+'/'+name:sha for name,sha in geometry['source_files_sha256'].items()})
        for source,frame in zip(episode['frames'],geometry['frames']):
            start=list(source['camera_position_cm']);start[2]-=config.eye_height_m*100
            binding=dict(sequence=sequence,source_run_id=episode['run_id'],frame_id=source['frame_id'],
                source_frame_id=source['source_frame_id'],t_source_sim_s=source['t_source_sim_s'],
                start_cm=start,heading_deg=source['camera_rotation_deg'][1]);bindings.append(binding)
            for mode in ('level','sweep'):
                item=frame[mode];planning=item['planning'];status=planning['status']
                gate=(validate_route(np.array(item['states']),config,planning['path_right_forward_m'],
                    start=(0,0),goal=(0,3),tracking_margin_m=SETTINGS['tracking_margin_m'])
                    if status=='path_found' else dict(status='planning_'+status,waypoints_right_forward_m=[]))
                local=gate['waypoints_right_forward_m']
                runs.append(dict(binding,run_id=f"{sequence}-frame_{source['frame_id']:06d}-{mode}",mode=mode,
                    local_goal_m=[0,3],planning_status=status,gate_status=gate['status'],gate_detail=gate,
                    local_waypoints_m=local,world_waypoints_cm=[local_to_world(p,start,binding['heading_deg']) for p in local]))
    return finish(dict(schema_version=1,condition=CONDITION,execution_scope='frozen_static_depth_transfer',
        map=base['map'],map_sha256=base['source_provenance']['map_sha256'],settings=copy.deepcopy(SETTINGS),
        source_provenance=dict(kind='static_pitch_sweep_capture',sweep_sha256=sweep['sweep_sha256'],
            source_attempt_id=base['source_attempt_id'],capture_index_sha256=file_hash(capture/'capture-index.json'),
            sweep_plan_sha256=file_hash(capture/'evaluation/sweep-plan.json'),camera_poses_sha256=file_hash(capture/'evaluation/camera_poses.jsonl'),
            **source_job_binding),
        geometry_source_sha256={name:file_hash(Path(__file__).parent/name) for name in GEOMETRY_FILES},
        source_files_sha256=hashes,grid_config=asdict(config),frame_bindings=bindings,runs=runs))


def build_controls(map_path,map_sha256):
    runs=[]
    for name,start,goal,route,gate in [
        ('detour',[-700,0,0],[0,14],[[0,0],[-8,5],[-8,9],[0,14]],'approved'),
        ('direct-blocked',[-700,0,0],[0,14],[[0,0],[0,14]],'approved'),
        ('spawn-blocked',[0,0,0],[0,7],[[0,0],[0,7]],'approved'),
        ('planning-rejected',[-700,0,0],[0,3],[],'planning_start_unobserved'),
        ('immediate',[-700,-500,0],[0,0],[[0,0]],'approved')]:
        runs.append(dict(sequence='synthetic',source_run_id=name,frame_id=0,source_frame_id=0,t_source_sim_s=0,
            start_cm=start,heading_deg=0,run_id=name,mode='control',local_goal_m=goal,planning_status='synthetic_supplied_route',
            gate_status=gate,gate_detail=dict(status=gate),local_waypoints_m=route,
            world_waypoints_cm=[local_to_world(p,start,0) for p in route]))
    return finish(dict(schema_version=1,condition=CONDITION,execution_scope='synthetic_supplied_route_controls',
        map=map_path,map_sha256=map_sha256,settings=dict(SETTINGS,max_path_cm=5000),
        source_provenance=dict(kind='synthetic_supplied_routes_not_depth_estimates'),geometry_source_sha256={},
        source_files_sha256={},grid_config=asdict(GridConfig()),frame_bindings=[],runs=runs))


def validate_bundle(bundle):
    keys={'schema_version','condition','execution_scope','map','map_sha256','settings','source_provenance',
          'geometry_source_sha256','source_files_sha256','grid_config','frame_bindings','runs','manifest_sha256'}
    if (set(bundle)!=keys or bundle['schema_version']!=1 or bundle['condition']!=CONDITION
            or not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',bundle['map'])
            or not re.fullmatch('[0-9a-f]{64}',bundle['map_sha256']) or bundle['manifest_sha256']!=digest_bundle(bundle)
            or bundle['grid_config']!=asdict(GridConfig()) or not 1<=len(bundle['runs'])<=200):
        raise ValueError('invalid frozen depth execution bundle')
    city=bundle['execution_scope']=='frozen_static_depth_transfer'
    if city:
        if (bundle['settings']!=SETTINGS or bundle['source_provenance'].get('kind')!='static_pitch_sweep_capture'
                or set(bundle['geometry_source_sha256'])!=set(GEOMETRY_FILES)
                or not {'render_job_sha256','render_status_sha256','historical_render_binary_sha256'}<=set(bundle['source_provenance'])):
            raise ValueError('depth transfer must retain observed geometry source and fixed settings')
    elif (bundle['execution_scope']!='synthetic_supplied_route_controls'
            or bundle['settings']!=dict(SETTINGS,max_path_cm=5000)
            or bundle['source_provenance']!={'kind':'synthetic_supplied_routes_not_depth_estimates'}
            or bundle['frame_bindings'] or bundle['source_files_sha256'] or bundle['geometry_source_sha256']):
        raise ValueError('invalid separate supplied-route control scope')
    expected={}
    for binding in bundle['frame_bindings']:
        if set(binding)!=BINDING_KEYS:raise ValueError('unexpected frame binding fields')
        key=(binding['sequence'],binding['frame_id'])
        if key in expected:raise ValueError('duplicate source frame')
        expected[key]=binding
    seen=set()
    for row in bundle['runs']:
        if set(row)!=RUN_KEYS or row['run_id'] in seen or not re.fullmatch('[A-Za-z0-9_-]+',row['run_id']):raise ValueError('invalid/duplicate depth execution run')
        seen.add(row['run_id']);position(row['start_cm']);number(row['heading_deg']);point(row['local_goal_m'])
        local=row['local_waypoints_m']
        if not isinstance(local,list) or len(local)>256:raise ValueError('unbounded waypoint count')
        expected_world=[local_to_world(p,row['start_cm'],row['heading_deg']) for p in local]
        if row['world_waypoints_cm']!=expected_world:raise ValueError('world waypoints differ from local geometry/actuation transform')
        if row['gate_detail']['status']!=row['gate_status'] or bool(local)!=(row['gate_status']=='approved'):
            raise ValueError('only approved routes may move')
        if local and (local[0]!=[0,0] or local[-1]!=row['local_goal_m']):raise ValueError('continuous start/goal connector missing')
        if city:
            key=(row['sequence'],row['frame_id'])
            if (key not in expected or any(row[k]!=value for k,value in expected[key].items())
                    or row['mode'] not in ('level','sweep') or row['local_goal_m']!=[0,3]
                    or row['run_id']!=f"{row['sequence']}-frame_{row['frame_id']:06d}-{row['mode']}"):
                raise ValueError('execution changes the frozen source-frame probe')
        elif row['mode']!='control':raise ValueError('controls cannot be relabeled as depth estimates')
    if city and ({(row['sequence'],row['frame_id'],row['mode']) for row in bundle['runs']}
            !={(*key,mode) for key in expected for mode in ('level','sweep')} or len(bundle['runs'])!=2*len(expected)):
        raise ValueError('missing source frames or conditions')
    return bundle
