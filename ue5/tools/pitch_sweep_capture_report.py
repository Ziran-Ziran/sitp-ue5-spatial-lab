"""Evaluation-side provenance checks, never imported by a geometry policy."""
import json
from pathlib import Path
from pitch_sweep_protocol import validate_sweep,VIEWS
from pitch_sweep_report import inspect_sweep_sequence
from rendered_input_report import validate_capture_frame,unlinked


def inspect_sweep_capture(capture):
    capture=Path(capture).absolute();unlinked(capture)
    plan=validate_sweep(json.loads((capture/'evaluation/sweep-plan.json').read_text(encoding='utf-8-sig')))
    base=plan['base_plan']
    index=json.loads((capture/'capture-index.json').read_text(encoding='utf-8-sig'))
    if (index.get('evidence_kind')!='rendered_static_pitch_sweep' or index.get('finished') is not True
            or index.get('sweep_sha256')!=plan['sweep_sha256']
            or any(index.get(key)!=base[key] for key in ('source_attempt_id','source_stream_sha256','map','source_provenance','render_settings'))):
        raise ValueError('incomplete or mismatched static pitch-sweep provenance')
    expected_names={f'sequence_{i:03d}' for i in range(len(base['episodes']))}
    if len(index['sequences'])!=len(base['episodes']) or {path.name for path in (capture/'input').iterdir()}!=expected_names:
        raise ValueError('missing source episodes')
    poses={}
    for line in (capture/'evaluation/camera_poses.jsonl').read_text(encoding='utf-8-sig').splitlines():
        mapping=json.loads(line);key=(mapping['sequence'],mapping['view'],mapping['frame_id'])
        if key in poses:raise ValueError('duplicate recorded view pose')
        poses[key]=mapping
    used=set();summaries=[]
    for i,(episode,record) in enumerate(zip(base['episodes'],index['sequences'])):
        name=f'sequence_{i:03d}'
        if (record['sequence']!=name or any(record[key]!=episode[key] for key in ('run_id','source_frames','source_status'))
                or record['expected_frames']!=len(episode['frames']) or set(record['views'])!={view['name'] for view in VIEWS}):
            raise ValueError('source sequence or fixed view mapping mismatch')
        intake=inspect_sweep_sequence(capture/'input'/name)
        if intake['frames']!=len(episode['frames']):raise ValueError('missing or extra sweep source frames')
        for view in VIEWS:
            view_name=view['name'];native_frames=record['views'][view_name]
            sensor=json.loads((capture/'input'/name/view_name/'sensor.json').read_text(encoding='utf-8-sig'))
            if sensor['render_settings']!=base['render_settings'] or len(native_frames)!=len(episode['frames']):
                raise ValueError('view exposure or count differs from plan')
            for frame,native,stamp,stats in zip(episode['frames'],native_frames,intake['timestamps'],
                    intake['views'][view_name]['frame_statistics']):
                expected=dict(frame,camera_rotation_deg=[-view['pitch_down_deg'],frame['camera_rotation_deg'][1],0])
                key=(name,view_name,frame['frame_id']);mapping=poses.get(key)
                if mapping is None or any(mapping.get(field)!=value for field,value in
                        dict(sequence=name,run_id=episode['run_id'],view=view_name,**expected).items()):
                    raise ValueError('view pose-to-source mapping mismatch')
                used.add(key)
                validate_capture_frame(native,expected,mapping,stamp,stats,f'input/{name}/{view_name}',base['render_settings']['fixed_exposure_ev100'])
        summaries.append(dict(sequence=name,source_frames=len(episode['frames']),image_pairs=3*len(episode['frames'])))
    if used!=set(poses):raise ValueError('extra recorded view poses')
    return dict(evidence_kind='static_pitch_sweep_capture_contract',input_contract_ready=True,
        sweep_sha256=plan['sweep_sha256'],source_attempt_id=base['source_attempt_id'],sequences=summaries,
        source_frames=sum(row['source_frames'] for row in summaries),image_pairs=sum(row['image_pairs'] for row in summaries))
