"""UE-only new static pitch observations, separate from historical live cameras."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import unreal
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pitch_sweep_protocol import VIEWS,validate_sweep,expected_view_metadata,rotation_matches
from rendered_replay import build_plan
from semantic_observation_stream import jsonl
from replay_source_binding import binding,sha256


def write(path,value):
    with path.open('x',encoding='utf-8') as target:json.dump(value,target,indent=2,allow_nan=False)


def capture_sequence(world,root,name,run_id,frames,settings,poses):
    directory=root/'input'/name;directory.mkdir()
    record=dict(sequence=name,run_id=run_id,expected_frames=len(frames),views={})
    for view in VIEWS:
        view_name=view['name'];path=directory/view_name;path.mkdir()
        for subdir in ('frames','depth'):(path/subdir).mkdir()
        write(path/'sensor.json',expected_view_metadata(settings,view['pitch_down_deg']))
        record['views'][view_name]=[]
        with (path/'timestamps.jsonl').open('x',encoding='utf-8') as times:
            for source_frame in frames:
                frame=dict(source_frame,camera_rotation_deg=[-view['pitch_down_deg'],source_frame['camera_rotation_deg'][1],0])
                frame_id=frame['frame_id'];filename=f'frame_{frame_id:06d}.png'
                rgb=path/'frames'/filename;depth=path/'depth'/filename
                pitch,yaw,roll=frame['camera_rotation_deg']
                native=json.loads(unreal.SpatialLabRgbd.capture_pose(world,unreal.Vector(*frame['camera_position_cm']),
                    unreal.Rotator(pitch=pitch,yaw=yaw,roll=roll),str(rgb),str(depth),settings['fixed_exposure_ev100']))
                native.update(frame_id=frame_id,color_file=f'input/{name}/{view_name}/frames/{filename}',
                    depth_file=f'input/{name}/{view_name}/depth/{filename}')
                record['views'][view_name].append(native)
                if (not native['captured'] or math.dist(frame['camera_position_cm'],native['camera_position_cm'])>1e-3
                        or not rotation_matches(native['camera_rotation_deg'],frame['camera_rotation_deg'])):
                    write(root/'capture-failure.json',dict(expected=frame,native=native))
                    raise RuntimeError('native static pitch capture/pose failed; diagnostics retained')
                native.update(rgb_sha256=sha256(rgb),depth_sha256=sha256(depth))
                times.write(json.dumps({key:frame[key] for key in ('frame_id','t_source_sim_s')})+'\n');times.flush()
                poses.write(json.dumps(dict(sequence=name,run_id=run_id,view=view_name,**frame,
                    actual_camera_position_cm=native['camera_position_cm'],actual_camera_rotation_deg=native['camera_rotation_deg']))+'\n')
                poses.flush()
    return record


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('plan','observations','source-job','out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    plan=validate_sweep(json.loads(args.plan.read_text(encoding='utf-8-sig')));base=plan['base_plan']
    provenance=binding(args.source_job,args.observations)
    if base!=build_plan(jsonl(args.observations),sha256(args.observations),source_provenance=provenance,
            frames_per_episode=base['frames_per_episode'],exposure_ev100=base['render_settings']['fixed_exposure_ev100']):
        raise ValueError('source attempt, map bytes or frozen frame selection changed')
    if args.out.exists():raise ValueError('new sweep capture output required')
    args.out.mkdir(parents=True);(args.out/'input').mkdir();(args.out/'evaluation').mkdir()
    if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(base['map']):raise RuntimeError('map load failed')
    world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    prior=unreal.SystemLibrary.get_console_variable_int_value('r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange')
    unreal.SystemLibrary.execute_console_command(world,'r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange 1')
    report=dict(schema_version=1,evidence_kind='rendered_static_pitch_sweep',finished=False,sweep_sha256=plan['sweep_sha256'],
        **{key:base[key] for key in ('source_attempt_id','source_stream_sha256','map','source_provenance','render_settings')},
        prior_luminance_mode=prior,luminance_override_scope='render_process_only',sequences=[])
    with (args.out/'evaluation/camera_poses.jsonl').open('x',encoding='utf-8') as poses:
        for i,episode in enumerate(base['episodes']):
            row=capture_sequence(world,args.out,f'sequence_{i:03d}',episode['run_id'],episode['frames'],base['render_settings'],poses)
            row.update(source_frames=episode['source_frames'],source_status=episode['source_status'])
            report['sequences'].append(row)
    report['finished']=True
    write(args.out/'evaluation/sweep-plan.json',plan);write(args.out/'capture-index.json',report)
