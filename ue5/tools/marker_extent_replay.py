"""Pixel-only extent-prior replay of saved observations, never simulated motion."""
import argparse
import ast
from collections import Counter
import json
from pathlib import Path
import cv2
from online_depth_policy import inspect_observation,CALIBRATION
from rendered_input_report import file_hash
from visual_marker import detect_markers
from marker_extent import VARIANTS


def source_snapshot(entry='marker_extent_replay'):
    directory=Path(__file__).parent;pending=[entry];found={}
    while pending:
        name=pending.pop();path=directory/(name+'.py')
        if name in found or not path.is_file():continue
        found[name]=file_hash(path)
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            names=([alias.name for alias in node.names] if isinstance(node,ast.Import)
                else [node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
            pending.extend(module.split('.')[0] for module in names)
    return {name+'.py':found[name] for name in sorted(found)}


def evaluate_record(path):
    path=Path(path);raw_hash=file_hash(path);sources=source_snapshot();record=json.loads(path.read_text(encoding='utf-8-sig'))
    if record.get('evidence_kind')!='ue_game_world_rgb_marker_depth_target_search' or not 1<=len(record['runs'])<=200:
        raise ValueError('bounded saved visual observation record required')
    observations=[];seen=set()
    for run in record['runs']:
        if run['run_id'] in seen:raise ValueError('duplicate saved run')
        seen.add(run['run_id']);actions=run['execution']['actions']
        if len(actions)>12:raise ValueError('bounded saved observations required')
        for event in actions:
            if event['status']!='complete':
                observations.append(dict(run_id=run['run_id'],step_id=event['step_id'],status='incomplete_saved_observation',models={}))
                continue
            request=event['request'];directory=Path(request['input_dir'])
            if request['run_id']!=run['run_id'] or request['category']!=run['input']['category']:raise ValueError('changed saved category/run')
            views,hashes=inspect_observation(directory,request['t_sim_s'])
            color=cv2.imread(str(directory/'level/rgb.png'),cv2.IMREAD_COLOR);models={}
            for variant in VARIANTS:
                report=detect_markers(color,views[0][1],CALIBRATION,perception_variant=variant)
                if variant=='hsv_depth_v1' and report!=event['response']['detection_report']:
                    raise ValueError('saved baseline detector differs from current legacy pixel computation')
                targets=[d for d in report['detections'] if d['category']==request['category']]
                models[variant]=dict(selected=targets[0] if targets else None,detection_report=report)
            if any(file_hash(directory/name)!=sha for name,sha in hashes.items()):raise ValueError('source pixels changed during extent replay')
            observations.append(dict(run_id=run['run_id'],step_id=event['step_id'],status='pixel_comparison',
                category=request['category'],t_source_sim_s=request['t_sim_s'],input_files_sha256=hashes,models=models))
    if file_hash(path)!=raw_hash or source_snapshot()!=sources:raise ValueError('saved record or detector source changed during replay')
    summary=dict(observations=len(observations),incomplete_observations=sum(not row['models'] for row in observations))
    for variant in VARIANTS:
        rows=[row['models'][variant] for row in observations if row['models']]
        summary[variant]=dict(selected_observations=sum(row['selected'] is not None for row in rows),
            rejected_component_reasons=dict(Counter(item['reason'] for row in rows for item in row['detection_report']['rejected'])))
    return dict(schema_version=1,evidence_kind='offline_saved_observation_known_box_extent_comparison',
        source_execution_sha256=raw_hash,detector_source_sha256=sources,summary=summary,observations=observations,
        interpretation='same historical RGB-D observations only; no truth or world pose in detection, no new motion or counterfactual navigation success; source Game execution is not certified by this CPU replay')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execution',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    result=evaluate_record(args.execution)
    with args.out.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()
