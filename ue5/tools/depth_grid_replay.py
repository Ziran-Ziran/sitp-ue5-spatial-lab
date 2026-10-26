"""Offline depth-only geometric probes; no evaluation, engine or target inputs."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time
import cv2
import numpy as np
from local_depth_grid import GridConfig,depth_grid,number
from grid_path import astar
from rendered_input_report import inspect_sequence,file_hash,unlinked
from query_grid_index import query_cell,TOLERANCE_CELLS


def plan_grid(grid,start,goal,*,query_cell_policy=None):
    config,states,free=grid['config'],grid['states'],grid['free']
    cells=[config.cell(*point) for point in (start,goal)]
    raw_cells=cells
    if query_cell_policy is not None:cells=[query_cell(config,point,query_cell_policy) for point in (start,goal)]
    plan=astar(free,*cells,resolution_m=config.resolution_m)
    for label,cell in zip(('start','goal'),cells):
        if plan['status']==label+'_blocked':
            plan['status']=label+{-1:'_unobserved',0:'_clearance_blocked',1:'_obstacle',2:'_negative_hazard'}[int(states[cell])]
    plan['path_right_forward_m']=[config.center(cell) for cell in plan['cells']]
    if query_cell_policy is not None:
        plan['query_indexing']=dict(policy=query_cell_policy,tolerance_cells=TOLERANCE_CELLS if query_cell_policy!='legacy_floor' else 0,
            raw_cells=[list(c) for c in raw_cells],selected_cells=[list(c) for c in cells])
    return plan


def evaluate_sequence(directory,config,*,start=(0,0),goal=(0,3)):
    directory=Path(directory).absolute();unlinked(directory)
    for point in (start,goal):
        if not isinstance(point,(tuple,list)) or len(point)!=2:raise ValueError('local query needs right/forward meter pairs')
        for value in point:number(value)
    intake=inspect_sequence(directory)
    if intake['frames']>1000:raise ValueError('bounded offline probe accepts at most 1000 frames')
    sensor_path=directory/'sensor.json';timestamps_path=directory/'timestamps.jsonl'
    sensor=json.loads(sensor_path.read_text(encoding='utf-8-sig'))
    stamps=[json.loads(line) for line in timestamps_path.read_text(encoding='utf-8-sig').splitlines()]
    hashes={path.name:file_hash(path) for path in (sensor_path,timestamps_path)}
    result=dict(schema_version=1,evidence_kind='offline_depth_grid_path_probe',
        condition='depth_geometry_upright_eye_height_prior',config=asdict(config),
        coordinates='local right, forward meters; reference floor from explicit eye height; no global pose',
        query=dict(start_right_forward_m=list(start),goal_right_forward_m=list(goal),
                   source='explicit fixed local geometric probe; not a semantic target'),
        source_files_sha256=hashes,frames=[],path_found_frames=0)
    for index,statistics in enumerate(intake['frame_statistics']):
        begun=time.perf_counter()
        name=f'frame_{index:06d}.png'
        for subdir,key in (('frames','rgb_sha256'),('depth','depth_sha256')):
            if file_hash(directory/subdir/name)!=statistics[key]:raise ValueError('input changed after intake')
            hashes[f'{subdir}/{name}']=statistics[key]
        path=directory/'depth'/name
        grid=depth_grid(cv2.imread(str(path),cv2.IMREAD_UNCHANGED),sensor['calibration'],config)
        if file_hash(path)!=statistics['depth_sha256']:raise ValueError('depth changed while processing')
        states,free=grid['states'],grid['free']
        plan=plan_grid(grid,start,goal)
        result['path_found_frames']+=plan['status']=='path_found'
        result['frames'].append(dict(frame_id=index,t_source_sim_s=stamps[index]['t_source_sim_s'],
            observed_ground_cells=int(np.count_nonzero(states==0)),obstacle_cells=int(np.count_nonzero(states==1)),
            unknown_cells=int(np.count_nonzero(states==-1)),clearance_free_cells=int(free.sum()),
            negative_hazard_cells=int(np.count_nonzero(states==2)),negative_samples=grid['negative_samples'],
            valid_depth_points=grid['valid_depth_points'],floor_samples=grid['floor_samples'],
            obstacle_samples=grid['obstacle_samples'],states=states.tolist(),clearance_free=free.tolist(),
            planning=plan,geometry_and_planning_wall_s=time.perf_counter()-begun))
    if any(file_hash(directory/path)!=sha for path,sha in hashes.items()):raise ValueError('input changed during replay')
    result['failures']=dict(Counter(row['planning']['status'] for row in result['frames'] if row['planning']['status']!='path_found'))
    result['interpretation']='per-frame sampled depth geometry and fixed local path probe; no semantics, global map, execution, navigation success or slope/physics guarantee'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True,help='isolated algorithm input parent containing sequence_NNN only')
    parser.add_argument('--out',type=Path,required=True,help='new local report directory')
    args=parser.parse_args()
    source=args.input.absolute();unlinked(source)
    directories=sorted(source.iterdir())
    if (not directories or len(directories)>100 or any(path.name!=f'sequence_{index:03d}' or not path.is_dir()
            for index,path in enumerate(directories))):raise ValueError('only contiguous isolated sequence directories accepted')
    if args.out.exists():raise ValueError('new report directory required')
    args.out.mkdir(parents=True)
    config=GridConfig();reports=[]
    for directory in directories:
        report=evaluate_sequence(directory,config)
        with (args.out/(directory.name+'.json')).open('x',encoding='utf-8') as target:json.dump(report,target,indent=2,allow_nan=False)
        reports.append(dict(sequence=directory.name,frames=len(report['frames']),path_found_frames=report['path_found_frames'],
                            failures=report['failures'],source_files_sha256=report['source_files_sha256']))
    summary=dict(schema_version=1,evidence_kind='offline_depth_grid_path_probe_summary',config=asdict(config),sequences=reports,
        frames=sum(row['frames'] for row in reports),path_found_frames=sum(row['path_found_frames'] for row in reports))
    with (args.out/'summary.json').open('x',encoding='utf-8') as target:json.dump(summary,target,indent=2,allow_nan=False)
    print(json.dumps({key:summary[key] for key in ('evidence_kind','frames','path_found_frames')}))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError) as error:
        import sys
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
