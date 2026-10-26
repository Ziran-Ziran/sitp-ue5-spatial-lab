"""Freeze complete pose-supervised geometric tasks; pixels acquired during execution."""
import copy
import hashlib
import json
import math
import re
from local_depth_grid import number
from semantic_memory import position
from query_grid_index import POLICIES

CONDITION='online_stopped_depth_geometry_pose_supervision'
MODES=('initial_sweep_once','receding_sweep')
SETTINGS=dict(radius_cm=34,half_height_cm=88,speed_cm_s=300,max_step_cm=35,max_slope_deg=45,
    reset_xy_limit_cm=5,reset_z_limit_cm=10,waypoint_tolerance_cm=5,tracking_limit_cm=10,
    max_vertical_excursion_cm=50,max_sim_s=30,max_wall_s=60,max_path_cm=2000,
    no_progress_sim_s=2,max_captures=8,receding_route_m=1,arrival_xy_cm=10,arrival_z_cm=20)


def digest(tasks):
    return hashlib.sha256(json.dumps({k:v for k,v in tasks.items() if k!='manifest_sha256'},
        sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def with_index_policy(tasks,policy):
    if policy is not None:
        if policy not in POLICIES:raise ValueError('explicit frozen query index policy required')
        tasks.update(schema_version=2,query_cell_policy=policy)
        tasks['runs']=[dict(run,query_cell_policy=policy) for run in tasks['runs']]
    tasks['manifest_sha256']=digest(tasks);return validate_tasks(tasks)


def build_tasks(map_path,map_sha256,frames,*,exposure,source_binding,query_cell_policy=None):
    runs=[]
    for frame in frames:
        heading=math.radians(number(frame['heading_deg']));start=position(frame['start_cm'])
        goal=[start[0]+300*math.cos(heading),start[1]+300*math.sin(heading),start[2]]
        goal=[round(v,10) for v in goal]
        for mode in MODES:
            runs.append(dict(run_id=f"{frame['sequence']}-frame_{frame['frame_id']:06d}-{mode}",mode=mode,
                source_sequence=frame['sequence'],source_frame_id=frame['frame_id'],start_cm=list(start),
                heading_deg=frame['heading_deg'],goal_cm=goal,intervention='none'))
    tasks=dict(schema_version=1,condition=CONDITION,scope='frozen_source_pose_geometric_tasks',
        geometry_source='current_stopped_rgbd_depth_only',pose_source='engine_ground_truth_reset_relative_goal_and_actuation',
        map=map_path,map_sha256=map_sha256,fixed_exposure_ev100=exposure,settings=copy.deepcopy(SETTINGS),
        source_binding=copy.deepcopy(source_binding),frame_bindings=copy.deepcopy(frames),runs=runs)
    return with_index_policy(tasks,query_cell_policy)


def validate_tasks(tasks):
    keys={'schema_version','condition','scope','geometry_source','pose_source','map','map_sha256','fixed_exposure_ev100',
        'settings','source_binding','frame_bindings','runs','manifest_sha256'}
    version=tasks.get('schema_version');policy=tasks.get('query_cell_policy')
    if version==2:
        keys.add('query_cell_policy')
        if policy not in POLICIES:raise ValueError('version-two query index policy required')
    if (set(tasks)!=keys or version not in (1,2) or tasks['condition']!=CONDITION
            or tasks['geometry_source']!='current_stopped_rgbd_depth_only'
            or tasks['pose_source']!='engine_ground_truth_reset_relative_goal_and_actuation'
            or not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',tasks['map'])
            or not re.fullmatch('[0-9a-f]{64}',tasks['map_sha256']) or tasks['settings']!=SETTINGS
            or not -10<=number(tasks['fixed_exposure_ev100'])<=20 or tasks['manifest_sha256']!=digest(tasks)
            or not 1<=len(tasks['runs'])<=200):raise ValueError('invalid frozen online tasks')
    seen=set()
    for row in tasks['runs']:
        row_keys={'run_id','mode','source_sequence','source_frame_id','start_cm','heading_deg','goal_cm','intervention'}|({'query_cell_policy'} if version==2 else set())
        if (set(row)!=row_keys or row.get('query_cell_policy')!=policy
                or not re.fullmatch('[A-Za-z0-9_-]+',row['run_id']) or row['run_id'] in seen
                or row['mode'] not in MODES or row['intervention'] not in ('none','wall_after_50cm')):
            raise ValueError('invalid/duplicate online run')
        seen.add(row['run_id']);position(row['start_cm']);position(row['goal_cm']);number(row['heading_deg'])
    if tasks['scope']=='frozen_source_pose_geometric_tasks':
        expected=[]
        for frame in tasks['frame_bindings']:
            expected.extend((frame['sequence'],frame['frame_id'],mode) for mode in MODES)
            matches=[r for r in tasks['runs'] if r['source_sequence']==frame['sequence'] and r['source_frame_id']==frame['frame_id']]
            angle=math.radians(frame['heading_deg']);goal=[round(frame['start_cm'][0]+300*math.cos(angle),10),
                round(frame['start_cm'][1]+300*math.sin(angle),10),round(frame['start_cm'][2],10)]
            if any(r['start_cm']!=frame['start_cm'] or r['heading_deg']!=frame['heading_deg'] or r['goal_cm']!=goal
                   or r['intervention']!='none' for r in matches):raise ValueError('changed original geometric task')
        actual=[(r['source_sequence'],r['source_frame_id'],r['mode']) for r in tasks['runs']]
        if not expected or len(expected)!=len(set(expected)) or actual!=expected:raise ValueError('missing/reordered source frames or modes')
    elif tasks['scope']!='synthetic_online_depth_controls' or tasks['frame_bindings']:
        raise ValueError('unknown online task scope')
    return tasks


def build_controls(map_path,map_sha256,*,query_cell_policy=None):
    rows=[]
    for name,start,goal,intervention in (
            ('floor',[0,0,0],[300,0,0],'none'),
            ('wall',[0,500,0],[300,500,0],'none'),
            ('spawn',[160,500,0],[460,500,0],'none'),
            ('immediate',[-500,0,0],[-500,0,0],'none'),
            ('changed',[0,-500,0],[300,-500,0],'wall_after_50cm')):
        for mode in MODES:rows.append(dict(run_id=name+'-'+mode,mode=mode,source_sequence=name,source_frame_id=0,
            start_cm=start,heading_deg=0,goal_cm=goal,intervention=intervention))
    tasks=dict(schema_version=1,condition=CONDITION,scope='synthetic_online_depth_controls',
        geometry_source='current_stopped_rgbd_depth_only',pose_source='engine_ground_truth_reset_relative_goal_and_actuation',
        map=map_path,map_sha256=map_sha256,fixed_exposure_ev100=1,settings=copy.deepcopy(SETTINGS),
        source_binding={'kind':'owned_primitive_online_controls'},frame_bindings=[],runs=rows)
    return with_index_policy(tasks,query_cell_policy)
