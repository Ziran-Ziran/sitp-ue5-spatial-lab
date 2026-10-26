"""Same isolated pitch pixels, fixed local query, two explicit ground models."""
import argparse
import ast
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time
import cv2
import numpy as np
from local_depth_grid import GridConfig,multiview_grid
from support_plane_grid import support_plane_grid,MODEL
from continuous_grid_planner import plan_route
from pitch_sweep_report import inspect_sweep_sequence
from pitch_sweep_protocol import VIEWS
from rendered_input_report import file_hash,unlinked


def geometry_sources():
    directory=Path(__file__).parent;pending=['support_plane_replay'];found={}
    while pending:
        name=pending.pop();path=directory/(name+'.py')
        if name in found or not path.is_file():continue
        found[name]=file_hash(path)
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            names=([alias.name for alias in node.names] if isinstance(node,ast.Import)
                else [node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
            pending.extend(module.split('.')[0] for module in names)
    return {name+'.py':found[name] for name in sorted(found)}


def evaluate_sequence(directory):
    directory=Path(directory).absolute();intake=inspect_sweep_sequence(directory);config=GridConfig()
    if intake['frames']>1000:raise ValueError('bounded source frame count required')
    sources=geometry_sources()
    result=dict(schema_version=1,geometry_source_sha256=sources,evidence_kind='offline_same_pixels_ground_model_comparison',config=asdict(config),
        source_files_sha256=intake['source_files_sha256'],query=dict(start=[0,0],goal=[0,3],planner_variant='continuous_square_shortcuts'),frames=[])
    for stamp in intake['timestamps']:
        views=[]
        for view in VIEWS:
            relative=f"{view['name']}/depth/frame_{stamp['frame_id']:06d}.png";path=directory/relative
            if file_hash(path)!=intake['source_files_sha256'][relative]:raise ValueError('source changed before model comparison')
            views.append((view['pitch_down_deg'],cv2.imread(str(path),cv2.IMREAD_UNCHANGED)))
        frame=dict(stamp)
        for model,builder in (('horizontal_v1',multiview_grid),(MODEL,support_plane_grid)):
            begun=time.perf_counter();grid=builder(views,intake['calibration'],config)
            frame[model]=dict(support_plane=grid.get('support_plane'),states=grid['states'].tolist(),
                observed_floor_cells=int(np.count_nonzero(grid['states']==0)),unknown_cells=int(np.count_nonzero(grid['states']==-1)),
                obstacle_cells=int(np.count_nonzero(grid['states']==1)),negative_cells=int(np.count_nonzero(grid['states']==2)),
                route=plan_route(grid,(0,0),(0,3),'continuous_square_shortcuts'),geometry_and_planning_wall_s=time.perf_counter()-begun)
        result['frames'].append(frame)
    if any(file_hash(directory/name)!=sha for name,sha in intake['source_files_sha256'].items()):raise ValueError('source changed during model comparison')
    if geometry_sources()!=sources:raise ValueError('geometry source changed during model comparison')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();source=args.input.absolute();unlinked(source);directories=sorted(source.iterdir())
    if not 1<=len(directories)<=100 or any(p.name!=f'sequence_{i:03d}' or not p.is_dir() for i,p in enumerate(directories)):
        raise ValueError('isolated contiguous sequence input required')
    args.out.mkdir(parents=True,exist_ok=False);frames=[]
    for directory in directories:
        result=evaluate_sequence(directory);frames.extend(result['frames'])
        with (args.out/(directory.name+'.json')).open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    summary={}
    for model in ('horizontal_v1',MODEL):
        selected=[r[model] for r in frames]
        summary[model]=dict(frames=len(selected),route_statuses=dict(Counter(r['route']['continuous_gate']['status'] for r in selected)),
            fit_statuses=dict(Counter(r['support_plane']['status'] for r in selected if r['support_plane'])),
            mean_geometry_and_planning_wall_s=sum(r['geometry_and_planning_wall_s'] for r in selected)/len(selected))
    record=dict(schema_version=1,evidence_kind='offline_same_pixels_ground_model_comparison',models=summary,
        interpretation='observed point geometry and fixed local queries only; no engine truth, navigation execution, semantic success or generalization')
    with (args.out/'summary.json').open('x',encoding='utf-8') as stream:json.dump(record,stream,indent=2,allow_nan=False)
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
