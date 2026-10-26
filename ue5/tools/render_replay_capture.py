"""UE-only static pose replay. Algorithm inputs and pose/reference truth stay apart."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import unreal
sys.path.insert(0,str(Path(__file__).resolve().parent))
from rendered_replay import algorithm_metadata,build_plan,validate_plan
from semantic_observation_stream import jsonl
from replay_source_binding import binding


def write(path,data):
    with path.open('x',encoding='utf-8') as output:json.dump(data,output,indent=2,allow_nan=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--observations',type=Path,required=True)
    parser.add_argument('--source-job',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    plan=validate_plan(json.loads(args.plan.read_text(encoding='utf-8-sig')))
    with args.observations.open('rb') as source:source_hash=hashlib.file_digest(source,'sha256').hexdigest()
    source_binding=binding(args.source_job,args.observations)
    if plan!=build_plan(jsonl(args.observations),source_hash,source_provenance=source_binding,
                       frames_per_episode=plan['frames_per_episode'],exposure_ev100=plan['render_settings']['fixed_exposure_ev100']):
        raise ValueError('source attempt/poses or frozen selection changed; refuse replay')
    if args.out.exists():raise RuntimeError('use a new replay output directory')
    args.out.mkdir(parents=True)
    evaluation=args.out/'evaluation';evaluation.mkdir()
    inputs=args.out/'input';inputs.mkdir()
    if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(plan['map']):
        raise RuntimeError('cannot load the frozen source map')
    world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    prior_mode=unreal.SystemLibrary.get_console_variable_int_value('r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange')
    unreal.SystemLibrary.execute_console_command(world,'r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange 1')
    report=dict(schema_version=1,evidence_kind='rendered_static_pose_replay',plan_sha256=plan['plan_sha256'],
        source_attempt_id=plan['source_attempt_id'],source_stream_sha256=source_hash,map=plan['map'],finished=False,
        source_provenance=source_binding,render_settings=plan['render_settings'],
        prior_luminance_mode=prior_mode,luminance_override_scope='render_process_only',
        interpretation='offline render at replayed engine eye positions and causal motion heading; not actual live camera or recognition',
        sequences=[])
    with (evaluation/'camera_poses.jsonl').open('x',encoding='utf-8') as poses:
        for sequence,episode in enumerate(plan['episodes']):
            name=f'sequence_{sequence:03d}'
            directory=inputs/name;directory.mkdir()
            (directory/'frames').mkdir();(directory/'depth').mkdir()
            write(directory/'sensor.json',algorithm_metadata(plan))
            row=dict(sequence=name,run_id=episode['run_id'],source_status=episode['source_status'],
                     source_frames=episode['source_frames'],expected_frames=len(episode['frames']),frames=[])
            report['sequences'].append(row)
            with (directory/'timestamps.jsonl').open('x',encoding='utf-8') as timestamps:
                for frame in episode['frames']:
                    index=frame['frame_id'];filename=f'frame_{index:06d}.png'
                    color=directory/'frames'/filename;depth=directory/'depth'/filename
                    pitch,yaw,roll=frame['camera_rotation_deg']
                    result=json.loads(unreal.SpatialLabRgbd.capture_pose(world,
                        unreal.Vector(*frame['camera_position_cm']),unreal.Rotator(pitch=pitch,yaw=yaw,roll=roll),str(color),str(depth),
                        plan['render_settings']['fixed_exposure_ev100']))
                    result.update(frame_id=index,color_file=f'input/{name}/frames/{filename}',depth_file=f'input/{name}/depth/{filename}')
                    row['frames'].append(result)
                    if result['captured']:
                        location_error=math.dist(frame['camera_position_cm'],result['camera_position_cm'])
                        angles=[abs((actual-request+180)%360-180) for actual,request in zip(result['camera_rotation_deg'],frame['camera_rotation_deg'])]
                        if location_error>1e-3 or max(angles)>1e-3:
                            raise RuntimeError('rendered camera transform does not match frozen replay pose')
                    else:
                        write(args.out/'capture-index.json',report)
                        raise RuntimeError('capture failed; incomplete frames and native diagnostics retained')
                    with color.open('rb') as image:result['rgb_sha256']=hashlib.file_digest(image,'sha256').hexdigest()
                    with depth.open('rb') as image:result['depth_sha256']=hashlib.file_digest(image,'sha256').hexdigest()
                    timestamps.write(json.dumps(dict(frame_id=index,t_source_sim_s=frame['t_source_sim_s']))+'\n')
                    poses.write(json.dumps(dict(sequence=name,run_id=episode['run_id'],**frame,
                                               actual_camera_position_cm=result['camera_position_cm'],
                                               actual_camera_rotation_deg=result['camera_rotation_deg']))+'\n')
                    timestamps.flush();poses.flush()
    report['finished']=True
    write(args.out/'capture-index.json',report)
    write(evaluation/'replay-plan.json',plan)
