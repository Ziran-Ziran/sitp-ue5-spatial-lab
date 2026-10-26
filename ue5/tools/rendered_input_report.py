"""Inspect isolated replay RGB-D inputs; contract readiness is not model quality."""
import hashlib
import json
import math
import os
from pathlib import Path
import cv2
import numpy as np
from rendered_replay import CALIBRATION,DEPTH,validate_plan,validate_render_settings
from semantic_memory import finite,position
from render_controls_report import validate_pair


def unlinked(path):
    for node in (path,*path.parents):
        if node.is_symlink() or (hasattr(os.path,'isjunction') and os.path.isjunction(node)):
            raise ValueError('linked replay inputs are not supported')


def file_hash(path):
    with path.open('rb') as source:return hashlib.file_digest(source,'sha256').hexdigest()


def inspect_sequence(directory):
    directory=Path(directory).absolute();unlinked(directory)
    if {path.name for path in directory.iterdir()}!={'sensor.json','timestamps.jsonl','frames','depth'}:
        raise ValueError('algorithm sequence accepts only sensor/timestamps/RGB/depth, never pose or evaluation files')
    for path in directory.iterdir():unlinked(path)
    sensor=json.loads((directory/'sensor.json').read_text(encoding='utf-8-sig'))
    validate_render_settings(sensor.get('render_settings',{}))
    expected=dict(schema_version=1,modality='rgbd_offline_pose_replay',calibration=CALIBRATION,
        depth=DEPTH,render_settings=sensor['render_settings'],time_source='replay_source_simulated_seconds',
        timestamp_interpretation='saved engine episode time; not physical sensor sampling or render throughput')
    if sensor!=expected:raise ValueError('unexpected or privileged replay sensor metadata')
    timestamps=[json.loads(line) for line in (directory/'timestamps.jsonl').read_text(encoding='utf-8-sig').splitlines()]
    last=-1
    for index,row in enumerate(timestamps):
        if (set(row)!={'frame_id','t_source_sim_s'} or isinstance(row['frame_id'],bool) or not isinstance(row['frame_id'],int)
                or row['frame_id']!=index or finite(row['t_source_sim_s'])<0
                or finite(row['t_source_sim_s'])<=last):raise ValueError('invalid replay source timestamps')
        last=row['t_source_sim_s']
    expected_files={f'frame_{index:06d}.png' for index in range(len(timestamps))}
    for modality in ('frames','depth'):
        if {path.name for path in (directory/modality).iterdir()}!=expected_files:
            raise ValueError('missing, unpaired or unexpected replay files')
    result=dict(evidence_kind='isolated_rendered_replay_input_contract',input_contract_ready=True,
                frames=len(timestamps),frames_with_valid_depth=0,frames_with_spatial_rgb_variation=0,frame_statistics=[])
    for index in range(len(timestamps)):
        name=f'frame_{index:06d}.png'
        color_path=directory/'frames'/name;depth_path=directory/'depth'/name
        unlinked(color_path);unlinked(depth_path)
        color=cv2.imread(str(color_path),cv2.IMREAD_COLOR)
        depth=cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED)
        validate_pair(color,depth)
        valid=(depth>0)&(depth<65535)
        fraction=float(valid.mean())
        variation=float(color.std(axis=(0,1)).mean())
        result['frames_with_valid_depth']+=bool(valid.any())
        result['frames_with_spatial_rgb_variation']+=variation>1e-6
        result['frame_statistics'].append(dict(frame_id=index,valid_depth_fraction=fraction,
            rgb_spatial_std=variation,white_clipped_fraction=float((color.min(axis=2)>250).mean()),
            dark_fraction=float((color.max(axis=2)<5).mean()),
            colored_fraction=float((np.ptp(color.astype(float),axis=2)>10).mean()),
            rgb_sha256=file_hash(color_path),depth_sha256=file_hash(depth_path)))
    result['interpretation']='structural input readiness only; empty depth/uniform RGB retained, not recognition or SLAM success'
    return result


def inspect_capture(capture):
    capture=Path(capture)
    index=json.loads((capture/'capture-index.json').read_text(encoding='utf-8-sig'))
    if not index.get('finished') or index.get('evidence_kind')!='rendered_static_pose_replay':
        raise ValueError('no completed static-pose replay index')
    plan=validate_plan(json.loads((capture/'evaluation/replay-plan.json').read_text(encoding='utf-8-sig')))
    if any(index.get(key)!=plan[key] for key in ('plan_sha256','source_attempt_id','source_stream_sha256','map','source_provenance','render_settings')):
        raise ValueError('capture index and frozen replay provenance mismatch')
    poses=[json.loads(line) for line in (capture/'evaluation/camera_poses.jsonl').read_text(encoding='utf-8-sig').splitlines()]
    if len(poses)!=sum(len(episode['frames']) for episode in plan['episodes']):raise ValueError('missing or extra recorded camera poses')
    if len(index['sequences'])!=len(plan['episodes']):raise ValueError('missing source episodes')
    expected={f'sequence_{number:03d}' for number in range(len(index['sequences']))}
    if {path.name for path in (capture/'input').iterdir()}!=expected:raise ValueError('missing or extra algorithm sequences')
    summaries=[]
    pose_index=0
    for number,sequence in enumerate(index['sequences']):
        name=f'sequence_{number:03d}'
        episode=plan['episodes'][number]
        if (sequence['sequence']!=name or sequence['run_id']!=episode['run_id']
                or sequence['source_status']!=episode['source_status'] or sequence['source_frames']!=episode['source_frames']
                or sequence['expected_frames']!=len(episode['frames'])):raise ValueError('sequence mapping mismatch')
        summary=inspect_sequence(capture/'input'/name)
        sensor=json.loads((capture/'input'/name/'sensor.json').read_text())
        if sensor['render_settings']!=plan['render_settings']:raise ValueError('input exposure metadata differs from plan')
        if summary['frames']!=sequence['expected_frames'] or len(sequence['frames'])!=sequence['expected_frames']:
            raise ValueError('capture frame count mismatch')
        timestamps=[json.loads(line) for line in (capture/'input'/name/'timestamps.jsonl').read_text().splitlines()]
        for frame,native,stamp,stats in zip(episode['frames'],sequence['frames'],timestamps,summary['frame_statistics']):
            mapping=poses[pose_index];pose_index+=1
            expected=dict(sequence=name,run_id=episode['run_id'],**frame)
            if any(mapping.get(key)!=value for key,value in expected.items()):raise ValueError('pose-to-source-frame mapping mismatch')
            if (native.get('captured') is not True or native.get('frame_id')!=frame['frame_id']
                    or native.get('width')!=320 or native.get('height')!=240
                    or native.get('fixed_exposure_ev100')!=plan['render_settings']['fixed_exposure_ev100']
                    or stamp!={key:frame[key] for key in ('frame_id','t_source_sim_s')}
                    or native.get('rgb_sha256')!=stats['rgb_sha256'] or native.get('depth_sha256')!=stats['depth_sha256']):
                raise ValueError('native capture, source time or image hash mismatch')
            filename=f"frame_{frame['frame_id']:06d}.png"
            if (native.get('color_file')!=f'input/{name}/frames/{filename}'
                    or native.get('depth_file')!=f'input/{name}/depth/{filename}'):
                raise ValueError('native image reference mismatch')
            if (math.dist(position(native['camera_position_cm']),frame['camera_position_cm'])>1e-3
                    or mapping['actual_camera_position_cm']!=native['camera_position_cm']
                    or mapping['actual_camera_rotation_deg']!=native['camera_rotation_deg']
                    or max(abs((a-b+180)%360-180) for a,b in zip(position(native['camera_rotation_deg']),frame['camera_rotation_deg']))>1e-3):
                raise ValueError('actual camera transform differs from frozen source replay')
        summaries.append(dict(sequence=name,**summary))
    return dict(evidence_kind='rendered_pose_replay_dataset_contract',input_contract_ready=True,
        plan_sha256=index['plan_sha256'],source_attempt_id=index['source_attempt_id'],sequences=summaries,
        frames=sum(row['frames'] for row in summaries),frames_with_valid_depth=sum(row['frames_with_valid_depth'] for row in summaries),
        frames_with_spatial_rgb_variation=sum(row['frames_with_spatial_rgb_variation'] for row in summaries))
