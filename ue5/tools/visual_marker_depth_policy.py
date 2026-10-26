"""Observed RGB marker category -> metric local goal -> depth-only safe prefix."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import re
import time
import uuid
import cv2
from continuous_depth_path import validate_route
from continuous_grid_planner import VARIANTS,plan_route
from depth_grid_replay import plan_grid
from local_depth_grid import GridConfig,multiview_grid,number
from online_depth_policy import CALIBRATION,inspect_observation,route_prefix,write_observation
from rendered_input_report import file_hash
from visual_marker import CATEGORIES,detect_markers
from marker_goal_horizon import observed_goal,POLICIES

REQUEST_KEYS={'schema_version','attempt_id','run_id','step_id','t_sim_s','input_dir','category','consecutive_empty_views'}


def decide(request):
    begun=time.perf_counter()
    version=request.get('schema_version');keys=REQUEST_KEYS|({'planner_variant'} if version in (2,3) else set())|({'goal_horizon_policy'} if version==3 else set())
    if (set(request)!=keys or version not in (1,2,3) or (version in (2,3) and request['planner_variant'] not in VARIANTS)
            or version==3 and request['goal_horizon_policy'] not in POLICIES):
        raise ValueError('isolated versioned visual category request required')
    try:canonical=str(uuid.UUID(request['attempt_id']))
    except (ValueError,TypeError,AttributeError):raise ValueError('canonical attempt required') from None
    if (canonical!=request['attempt_id'] or not re.fullmatch('[A-Za-z0-9_-]+',request['run_id'])
            or type(request['step_id'])!=int or not 0<=request['step_id']<12
            or request['category'] not in CATEGORIES or type(request['consecutive_empty_views'])!=int
            or not 0<=request['consecutive_empty_views']<=3 or number(request['t_sim_s'])<0):
        raise ValueError('invalid visual request identity/category/budget')
    directory=Path(request['input_dir']);views,hashes=inspect_observation(directory,request['t_sim_s'])
    color=cv2.imread(str(directory/'level/rgb.png'),cv2.IMREAD_COLOR)
    detections=detect_markers(color,views[0][1],CALIBRATION)
    targets=[d for d in detections['detections'] if d['category']==request['category']]
    selected=targets[0] if targets else None
    result={key:request[key] for key in ('attempt_id','run_id','step_id','t_sim_s','category','consecutive_empty_views')}
    result.update(schema_version=1,evidence_kind='rgb_marker_depth_observed_target_action',selected=selected,
        detection_report=detections,input_files_sha256=hashes,waypoints_right_forward_m=[],relative_yaw_delta_deg=0,
        geometry_goal_right_forward_m=None,geometry=None)
    if version in (2,3):result.update(schema_version=version,planner_variant=request['planner_variant'])
    if version==3:result['goal_horizon_policy']=request['goal_horizon_policy']
    if selected is None:
        if request['consecutive_empty_views']<3:result.update(action='rotate',reason='target_category_not_observed',relative_yaw_delta_deg=90)
        else:result.update(action='stop',reason='target_not_observed_after_scan')
    elif selected['planar_range_m']<=1.25:result.update(action='candidate_reached',reason='fresh_marker_within_planar_reach')
    else:
        r,f,_=selected['position_right_forward_up_m'];distance=selected['planar_range_m']
        travel=min(3,distance-.9);goal=[r*travel/distance,f*travel/distance]
        if version==3:goal=observed_goal(selected['position_right_forward_up_m'],distance,request['goal_horizon_policy'])
        config=GridConfig();grid=multiview_grid(views,CALIBRATION,config)
        if version==1:
            planning=plan_grid(grid,(0,0),goal)
            gate=(validate_route(grid['states'],config,planning['path_right_forward_m'],start=(0,0),goal=goal,tracking_margin_m=.1)
                  if planning['status']=='path_found' else dict(status='planning_'+planning['status'],waypoints_right_forward_m=[]))
            geometry=dict(planning=planning,continuous_gate=gate,config=asdict(config))
        else:
            geometry=plan_route(grid,(0,0),goal,request['planner_variant']);geometry['config']=asdict(config)
            gate=geometry['continuous_gate']
        route=gate['waypoints_right_forward_m']
        result.update(action='move' if route else 'reject',reason='visible_marker_route' if route else gate['status'],
            geometry_goal_right_forward_m=goal,geometry=geometry,
            waypoints_right_forward_m=route_prefix(route,1) if route else [])
    if any(file_hash(directory/name)!=sha for name,sha in hashes.items()):raise ValueError('visual pixels changed during decision')
    result['perception_and_planning_wall_s']=time.perf_counter()-begun
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--prepare-observation',action='store_true');args=parser.parse_args()
    request=json.loads(args.request.read_text(encoding='utf-8-sig'))
    if args.prepare_observation:write_observation(request['input_dir'],request['t_sim_s'])
    answer=decide(request)
    with args.out.open('x',encoding='utf-8') as stream:json.dump(answer,stream,allow_nan=False)


if __name__=='__main__':main()
