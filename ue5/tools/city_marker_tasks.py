"""Controlled city color goals; engine beacon placement is fixture-only oracle."""
import copy
import re
from online_depth_tasks import build_tasks as source_tasks,validate_tasks as validate_source,digest,SETTINGS
from visual_marker_tasks import CONDITION,POSE_SOURCE

SCOPE='frozen_city_marker_source_pose_tasks'
VARIANTS=('legacy_centers','continuous_square_shortcuts')


def build_placement_plan(source):
    validate_source(source)
    if source['schema_version']!=1 or source['scope']!='frozen_source_pose_geometric_tasks':raise ValueError('original complete source-pose tasks required')
    anchors=[];seen=set()
    for frame in source['frame_bindings']:
        key=tuple(frame['start_cm'])
        if frame['frame_id']==0 and key not in seen:
            seen.add(key);anchors.append(dict(start_cm=list(key),heading_deg=frame['heading_deg']))
    if not 1<=len(anchors)<=12:raise ValueError('one to twelve original initial anchors required')
    return dict(schema_version=2,evidence_kind='oracle_constructed_controlled_city_color_beacons',base_map=source['map'],
        base_map_sha256=source['map_sha256'],source_manifest_sha256=source['manifest_sha256'],
        fixed_exposure_ev100=source['fixed_exposure_ev100'],emission_multiplier=2**(source['fixed_exposure_ev100']-1),anchors=anchors,
        rule=dict(candidate_radius_cm=400,candidate_turn_offsets_deg=[0,90,180,270],category_angle_deg=dict(red_marker=-30,blue_marker=30),
            maximum_height_change_cm=100,marker_center_height_cm=100,marker_scale=[.2,.2,.6],
            projection_extent_cm=[75,75,150],capsule_radius_cm=34,capsule_half_height_cm=88,max_slope_deg=45,
            query_initialization='existing_native_bounded_oracle_navigation_setup'))


def runs(frames,variant):
    return [dict(run_id=f"{f['sequence']}-frame_{f['frame_id']:06d}-{category}",mode='marker_receding',category=category,
        start_cm=list(f['start_cm']),heading_deg=f['heading_deg'],environment_case='all_markers',planner_variant=variant,
        source_sequence=f['sequence'],source_frame_id=f['frame_id']) for f in frames for category in ('red_marker','blue_marker')]


def build_tasks(map_path,map_sha256,source,scene_binding,variant,*,goal_horizon_policy=None):
    validate_source(source)
    if source['schema_version']!=1 or source['scope']!='frozen_source_pose_geometric_tasks' or variant not in VARIANTS:
        raise ValueError('original source tasks and predeclared city variant required')
    tasks=dict(schema_version=2,condition=CONDITION,scope=SCOPE,pose_source=POSE_SOURCE,semantic_source='rgb_fixed_hsv_color_marker',
        geometry_source='current_stopped_rgbd_depth_only',map=map_path,map_sha256=map_sha256,
        fixed_exposure_ev100=source['fixed_exposure_ev100'],success_radius_cm=175,settings=dict(SETTINGS,max_captures=12),planner_variant=variant,
        source_tasks={k:copy.deepcopy(source[k]) for k in ('map','map_sha256','fixed_exposure_ev100','source_binding','manifest_sha256')},
        frame_bindings=copy.deepcopy(source['frame_bindings']),scene_binding=copy.deepcopy(scene_binding),runs=runs(source['frame_bindings'],variant))
    tasks['manifest_sha256']=digest(tasks)
    if goal_horizon_policy is not None:
        from marker_horizon_tasks import with_horizon
        tasks=with_horizon(tasks,goal_horizon_policy)
    return validate_city_tasks(tasks)


def validate_city_tasks(tasks):
    if tasks.get('schema_version')==3:
        from marker_horizon_tasks import without_horizon
        validate_city_tasks(without_horizon(tasks));return tasks
    keys={'schema_version','condition','scope','pose_source','semantic_source','geometry_source','map','map_sha256','fixed_exposure_ev100',
        'success_radius_cm','settings','planner_variant','source_tasks','frame_bindings','scene_binding','runs','manifest_sha256'}
    if (set(tasks)!=keys or tasks['schema_version']!=2 or tasks['scope']!=SCOPE or tasks['condition']!=CONDITION
            or tasks['pose_source']!=POSE_SOURCE or tasks['semantic_source']!='rgb_fixed_hsv_color_marker'
            or tasks['geometry_source']!='current_stopped_rgbd_depth_only' or tasks['success_radius_cm']!=175
            or tasks['settings']!=dict(SETTINGS,max_captures=12) or tasks['planner_variant'] not in VARIANTS
            or not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',tasks['map'])
            or not re.fullmatch('[0-9a-f]{64}',tasks['map_sha256']) or tasks['manifest_sha256']!=digest(tasks)):
        raise ValueError('complete fixed city color tasks required')
    source=tasks['source_tasks'];binding=tasks['scene_binding']
    frame_keys={'sequence','source_run_id','frame_id','source_frame_id','t_source_sim_s','start_cm','heading_deg'}
    if any(set(frame)!=frame_keys for frame in tasks['frame_bindings']):raise ValueError('original pose metadata only; no target fields')
    if (set(source)!={'map','map_sha256','fixed_exposure_ev100','source_binding','manifest_sha256'}
            or set(binding)!={'base_map_sha256','fixture_plan_sha256','fixture_report_sha256'}
            or any(not re.fullmatch('[0-9a-f]{64}',v) for v in binding.values())
            or binding['base_map_sha256']!=source['map_sha256'] or tasks['map']==source['map']
            or tasks['fixed_exposure_ev100']!=source['fixed_exposure_ev100']):raise ValueError('bound independent city overlay/source required')
    rebuilt=source_tasks(source['map'],source['map_sha256'],tasks['frame_bindings'],exposure=source['fixed_exposure_ev100'],source_binding=source['source_binding'])
    if rebuilt['manifest_sha256']!=source['manifest_sha256']:raise ValueError('changed original source poses')
    expected=runs(tasks['frame_bindings'],tasks['planner_variant'])
    if not 1<=len(expected)<=200 or tasks['runs']!=expected:raise ValueError('all original poses and both categories required without target metadata')
    return tasks
