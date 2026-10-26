"""Fixed controlled marker-search missions, with no target coordinates/catalog."""
import re
from online_depth_tasks import digest,SETTINGS
from continuous_grid_planner import VARIANTS

CONDITION='rgbd_color_marker_target_search_pose_actuation_supervision'
POSE_SOURCE='engine_ground_truth_reset_and_actuation_only'


def fixed_runs():
    runs=[]
    for index,start,heading in ((0,[-100,0,0],0),(1,[-100,0,0],180),
            (2,[-400,-400,0],90),(3,[0,400,0],270)):
        for category in ('red_marker','blue_marker'):
            runs.append(dict(run_id=f'start_{index:02d}-{category}',mode='marker_receding',category=category,
                start_cm=start,heading_deg=heading,environment_case='all_markers'))
    for name,start,category,environment in (
            ('immediate',[230,-200,0],'red_marker','all_markers'),
            ('missing',[-100,0,0],'red_marker','red_hidden'),
            ('occluded',[-100,0,0],'red_marker','wall_on'),
            ('spawn',[100,0,0],'red_marker','wall_on')):
        runs.append(dict(run_id=name,mode='marker_receding',category=category,start_cm=start,heading_deg=0,environment_case=environment))
    return runs


def build_tasks(map_path,map_sha256,*,planner_variant=None):
    tasks=dict(schema_version=1,condition=CONDITION,scope='synthetic_controlled_visual_marker_search',
        pose_source=POSE_SOURCE,semantic_source='rgb_fixed_hsv_color_marker',geometry_source='current_stopped_rgbd_depth_only',
        map=map_path,map_sha256=map_sha256,fixed_exposure_ev100=1,success_radius_cm=175,
        settings=dict(SETTINGS,max_captures=12),runs=fixed_runs())
    if planner_variant is not None:
        if planner_variant not in VARIANTS:raise ValueError('explicit frozen planner variant required')
        tasks.update(schema_version=2,planner_variant=planner_variant)
        tasks['runs']=[dict(run,planner_variant=planner_variant) for run in tasks['runs']]
    tasks['manifest_sha256']=digest(tasks);return validate_tasks(tasks)


def validate_tasks(tasks):
    if tasks.get('scope')=='frozen_city_marker_source_pose_tasks':
        from city_marker_tasks import validate_city_tasks
        return validate_city_tasks(tasks)
    version=tasks.get('schema_version');variant=tasks.get('planner_variant')
    expected_runs=fixed_runs()
    if version==2:
        if variant not in VARIANTS:raise ValueError('fixed version-two planner variant required')
        expected_runs=[dict(run,planner_variant=variant) for run in expected_runs]
    keys={'schema_version','condition','scope','pose_source','semantic_source','geometry_source','map','map_sha256',
            'fixed_exposure_ev100','success_radius_cm','settings','runs','manifest_sha256'}|({'planner_variant'} if version==2 else set())
    if (set(tasks)!=keys or version not in (1,2)
            or tasks['condition']!=CONDITION or tasks['scope']!='synthetic_controlled_visual_marker_search'
            or tasks['pose_source']!=POSE_SOURCE or tasks['semantic_source']!='rgb_fixed_hsv_color_marker'
            or tasks['geometry_source']!='current_stopped_rgbd_depth_only' or tasks['fixed_exposure_ev100']!=1 or tasks['success_radius_cm']!=175
            or not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',tasks['map'])
            or not re.fullmatch('[0-9a-f]{64}',tasks['map_sha256']) or tasks['settings']!=dict(SETTINGS,max_captures=12)
            or tasks['runs']!=expected_runs or tasks['manifest_sha256']!=digest(tasks)):
        raise ValueError('complete fixed visual-marker missions required; no target metadata')
    return tasks
