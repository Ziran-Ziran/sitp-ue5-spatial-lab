"""Isolated static pitch views -> local depth geometry, never evaluation truth."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time
import cv2
import numpy as np
from local_depth_grid import GridConfig,depth_grid,multiview_grid
from depth_grid_replay import plan_grid
from pitch_sweep_protocol import VIEWS
from rendered_input_report import inspect_sequence,file_hash,unlinked


def inspect_sweep_sequence(directory):
    directory=Path(directory).absolute();unlinked(directory)
    if {path.name for path in directory.iterdir()}!={view['name'] for view in VIEWS}:
        raise ValueError('exactly level/down45/nadir isolated views required')
    reports={};stamps=None;settings=None;hashes={}
    for view in VIEWS:
        name=view['name'];path=directory/name
        report=inspect_sequence(path,pitch_down_deg=view['pitch_down_deg'])
        times=[json.loads(line) for line in (path/'timestamps.jsonl').read_text(encoding='utf-8-sig').splitlines()]
        sensor=json.loads((path/'sensor.json').read_text(encoding='utf-8-sig'))
        if stamps is not None and (times!=stamps or sensor['render_settings']!=settings):
            raise ValueError('static views must have identical source frames/times and exposure')
        stamps,settings=times,sensor['render_settings'];reports[name]=report
        for filename in ('sensor.json','timestamps.jsonl'):hashes[f'{name}/{filename}']=file_hash(path/filename)
        for row in report['frame_statistics']:
            for subdir,key in (('frames','rgb_sha256'),('depth','depth_sha256')):
                hashes[f"{name}/{subdir}/frame_{row['frame_id']:06d}.png"]=row[key]
    return dict(input_contract_ready=True,frames=len(stamps),timestamps=stamps,views=reports,
                calibration=sensor['calibration'],source_files_sha256=hashes)


def evaluate_sweep_sequence(directory,*,config=None,start=(0,0),goal=(0,3)):
    directory=Path(directory).absolute();config=config or GridConfig()
    intake=inspect_sweep_sequence(directory)
    if intake['frames']>1000:raise ValueError('bounded replay accepts at most 1000 source frames')
    result=dict(schema_version=1,evidence_kind='offline_calibrated_pitch_depth_path_probe',config=asdict(config),
        query=dict(start_right_forward_m=list(start),goal_right_forward_m=list(goal),source='fixed local geometric probe, not a semantic target'),
        source_files_sha256=intake['source_files_sha256'],frames=[],level_path_found_frames=0,sweep_path_found_frames=0)
    for stamp in intake['timestamps']:
        begun=time.perf_counter();index=stamp['frame_id'];views=[]
        for view in VIEWS:
            relative=f"{view['name']}/depth/frame_{index:06d}.png";path=directory/relative
            expected=intake['source_files_sha256'][relative]
            if file_hash(path)!=expected:raise ValueError('sweep depth changed after intake')
            depth=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
            if file_hash(path)!=expected:raise ValueError('sweep depth changed during read')
            views.append((view['pitch_down_deg'],depth))
        grids=dict(level=depth_grid(views[0][1],intake['calibration'],config),
                   sweep=multiview_grid(views,intake['calibration'],config))
        frame=dict(stamp)
        for mode,grid in grids.items():
            plan=plan_grid(grid,start,goal);states=grid['states']
            frame[mode]=dict(planning=plan,states=states.tolist(),clearance_free=grid['free'].tolist(),
                ground_cells=int(np.count_nonzero(states==0)),obstacle_cells=int(np.count_nonzero(states==1)),
                negative_hazard_cells=int(np.count_nonzero(states==2)),unknown_cells=int(np.count_nonzero(states==-1)),
                clearance_free_cells=int(grid['free'].sum()))
            result[mode+'_path_found_frames']+=plan['status']=='path_found'
        frame['geometry_and_planning_wall_s']=time.perf_counter()-begun
        result['frames'].append(frame)
    if any(file_hash(directory/path)!=sha for path,sha in intake['source_files_sha256'].items()):
        raise ValueError('sweep input changed during processing')
    result['failures']={mode:dict(Counter(row[mode]['planning']['status'] for row in result['frames']
        if row[mode]['planning']['status']!='path_found')) for mode in ('level','sweep')}
    result['interpretation']='offline fixed static views and explicit relative pitch/eye-height priors; no engine global poses, semantics, actual camera actuation or navigation execution'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();source=args.input.absolute();unlinked(source)
    directories=sorted(source.iterdir())
    if not 1<=len(directories)<=100 or any(path.name!=f'sequence_{i:03d}' for i,path in enumerate(directories)):
        raise ValueError('isolated contiguous sequence_NNN input only')
    if args.out.exists():raise ValueError('fresh report output required')
    args.out.mkdir(parents=True);summaries=[]
    for directory in directories:
        report=evaluate_sweep_sequence(directory)
        with (args.out/(directory.name+'.json')).open('x',encoding='utf-8') as target:json.dump(report,target,indent=2,allow_nan=False)
        summaries.append(dict(sequence=directory.name,frames=len(report['frames']),failures=report['failures'],
            level_path_found_frames=report['level_path_found_frames'],sweep_path_found_frames=report['sweep_path_found_frames']))
    summary=dict(evidence_kind='offline_static_pitch_sweep_comparison',sequences=summaries,
        frames=sum(row['frames'] for row in summaries),
        level_path_found_frames=sum(row['level_path_found_frames'] for row in summaries),
        sweep_path_found_frames=sum(row['sweep_path_found_frames'] for row in summaries))
    with (args.out/'summary.json').open('x',encoding='utf-8') as target:json.dump(summary,target,indent=2,allow_nan=False)
    print(json.dumps({key:value for key,value in summary.items() if key!='sequences'}))


if __name__=='__main__':main()
