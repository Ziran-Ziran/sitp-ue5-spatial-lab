"""Stopped online depth -> local route; no unreal, global pose, map or labels."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import re
import time
import uuid
import cv2
import numpy as np
from continuous_depth_path import validate_route
from depth_grid_replay import plan_grid
from local_depth_grid import GridConfig,multiview_grid,number
from pitch_sweep_protocol import VIEWS
from rendered_input_report import file_hash,unlinked
from query_grid_index import POLICIES

CALIBRATION=dict(w=320,h=240,fx=160,fy=160,cx=159.5,cy=119.5)
TIME_SOURCE='game_world_simulated_seconds_at_stopped_capture'
REQUEST_KEYS={'schema_version','attempt_id','run_id','step_id','input_dir','goal_right_forward_m','mode','t_sim_s'}


def world_goal_to_local(feet,goal,heading_deg):
    """Actuation-side pose supervision only; never called by pixel geometry."""
    for p in (feet,goal):
        if not isinstance(p,(list,tuple)) or len(p)!=3:raise ValueError('feet/goal need three coordinates')
        for value in p:number(value)
    angle=math.radians(number(heading_deg));dx=(goal[0]-feet[0])/100;dy=(goal[1]-feet[1])/100
    result=[-dx*math.sin(angle)+dy*math.cos(angle),dx*math.cos(angle)+dy*math.sin(angle)]
    return [0 if abs(value)<1e-12 else value for value in result]


def route_prefix(route,budget=1):
    number(budget)
    if budget<=0 or not route:raise ValueError('positive budget and nonempty route required')
    result=[list(route[0])];remaining=budget
    for start,end in zip(route,route[1:]):
        length=math.dist(start,end)
        if length<=remaining:
            result.append(list(end));remaining-=length
        else:
            result.append([a+(b-a)*remaining/length for a,b in zip(start,end)]);break
    return result


def write_observation(directory,t_sim_s):
    """Collector-side sealing immediately after native PNG readback."""
    directory=Path(directory);unlinked(directory);number(t_sim_s)
    if t_sim_s<0:raise ValueError('negative simulated time')
    hashes={f"{view['name']}/{kind}.png":file_hash(directory/view['name']/(kind+'.png'))
            for view in VIEWS for kind in ('rgb','depth')}
    record=dict(schema_version=1,modality='rgbd_online_stopped_calibrated_pitch_views',time_source=TIME_SOURCE,
        t_sim_s=t_sim_s,calibration=CALIBRATION,views=VIEWS,files_sha256=hashes,
        depth_unit='uint16_optical_millimeters_invalid0_max30000',eye_height_prior_m=1.5)
    with (directory/'observation.json').open('x',encoding='utf-8') as stream:json.dump(record,stream,allow_nan=False)


def inspect_observation(directory,t_sim_s):
    directory=Path(directory).absolute();unlinked(directory)
    if {p.name for p in directory.iterdir()}!={'observation.json',*(v['name'] for v in VIEWS)}:
        raise ValueError('isolated three-view pixel input only')
    path=directory/'observation.json';record=json.loads(path.read_text(encoding='utf-8'))
    expected={'schema_version','modality','time_source','t_sim_s','calibration','views','files_sha256','depth_unit','eye_height_prior_m'}
    if (set(record)!=expected or record['schema_version']!=1 or record['modality']!='rgbd_online_stopped_calibrated_pitch_views'
            or record['time_source']!=TIME_SOURCE or record['t_sim_s']!=t_sim_s or record['calibration']!=CALIBRATION
            or record['views']!=VIEWS or record['depth_unit']!='uint16_optical_millimeters_invalid0_max30000'
            or record['eye_height_prior_m']!=1.5):raise ValueError('wrong stopped-observation contract')
    hashes=record['files_sha256'];views=[]
    if set(hashes)!={f"{v['name']}/{kind}.png" for v in VIEWS for kind in ('rgb','depth')}:
        raise ValueError('complete pixel hashes required')
    for view in VIEWS:
        folder=directory/view['name'];unlinked(folder)
        if {p.name for p in folder.iterdir()}!={'rgb.png','depth.png'}:raise ValueError('extra fields/files in isolated view')
        for kind in ('rgb','depth'):
            file=folder/(kind+'.png');expected_hash=hashes[view['name']+'/'+kind+'.png']
            if not re.fullmatch('[0-9a-f]{64}',expected_hash) or file_hash(file)!=expected_hash:raise ValueError('changed online pixels')
            pixels=cv2.imread(str(file),cv2.IMREAD_COLOR if kind=='rgb' else cv2.IMREAD_UNCHANGED)
            if pixels is None or file_hash(file)!=expected_hash:raise ValueError('unreadable or changing pixels')
            if kind=='rgb':
                if pixels.dtype!=np.uint8 or pixels.shape!=(240,320,3):raise ValueError('online RGB must decode BGR8')
            else:
                if pixels.dtype!=np.uint16 or pixels.shape!=(240,320):raise ValueError('online depth must decode uint16')
                views.append((view['pitch_down_deg'],pixels))
    return views,dict(hashes,**{'observation.json':file_hash(path)})


def decide(request):
    begun=time.perf_counter()
    version=request.get('schema_version');keys=REQUEST_KEYS|({'query_cell_policy'} if version==2 else set())
    if (set(request)!=keys or version not in (1,2) or (version==2 and request['query_cell_policy'] not in POLICIES)):
        raise ValueError('strict versioned isolated online request required')
    try:canonical=str(uuid.UUID(request['attempt_id']))
    except (ValueError,TypeError,AttributeError):raise ValueError('canonical attempt UUID required') from None
    if (canonical!=request['attempt_id'] or not re.fullmatch('[A-Za-z0-9_-]+',request['run_id'])
            or type(request['step_id'])!=int or not 0<=request['step_id']<8
            or request['mode'] not in ('initial_sweep_once','receding_sweep')):raise ValueError('invalid online identity or mode')
    goal=request['goal_right_forward_m']
    if not isinstance(goal,list) or len(goal)!=2 or any(abs(number(value))>20 for value in goal):raise ValueError('bounded relative task goal required')
    if number(request['t_sim_s'])<0:raise ValueError('invalid episode simulated time')
    views,hashes=inspect_observation(request['input_dir'],request['t_sim_s']);config=GridConfig()
    grid=multiview_grid(views,CALIBRATION,config);planning=plan_grid(grid,(0,0),goal,query_cell_policy=request.get('query_cell_policy'))
    gate=(validate_route(grid['states'],config,planning['path_right_forward_m'],start=(0,0),goal=goal,tracking_margin_m=.1)
          if planning['status']=='path_found' else dict(status='planning_'+planning['status'],waypoints_right_forward_m=[]))
    route=gate['waypoints_right_forward_m']
    command=route_prefix(route,1) if route and request['mode']=='receding_sweep' else route
    answer={key:request[key] for key in ('attempt_id','run_id','step_id','mode','t_sim_s','goal_right_forward_m')}
    answer.update(schema_version=1,evidence_kind='online_stopped_depth_geometry_action',action='move' if route else 'reject',
        reason=gate['status'],waypoints_right_forward_m=command,planning=planning,continuous_gate=gate,
        input_files_sha256=hashes,config=asdict(config),geometry_and_planning_wall_s=time.perf_counter()-begun)
    if version==2:answer.update(schema_version=2,query_cell_policy=request['query_cell_policy'])
    if any(file_hash(Path(request['input_dir'])/name)!=sha for name,sha in hashes.items()):raise ValueError('input changed during online decision')
    return answer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--prepare-observation',action='store_true');args=parser.parse_args()
    request=json.loads(args.request.read_text(encoding='utf-8-sig'))
    if args.prepare_observation:write_observation(request['input_dir'],request['t_sim_s'])
    answer=decide(request)
    with args.out.open('x',encoding='utf-8') as stream:json.dump(answer,stream,allow_nan=False)


if __name__=='__main__':main()
