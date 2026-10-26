"""Independent fixed plane-depth and geometric path controls, no city truth."""
import json
from pathlib import Path
import cv2
import numpy as np
from local_depth_grid import GridConfig,depth_grid,multiview_grid
from depth_grid_replay import plan_grid
from pitch_sweep_protocol import VIEWS
from pitch_sweep_report import inspect_sweep_sequence
from rendered_replay import CALIBRATION
from rendered_input_report import validate_capture_frame


def evaluate_control_pixels(cases):
    if set(cases)!={'flat','wall','drop'} or any(set(views)!={0,45,90} for views in cases.values()):
        raise ValueError('all three pitch views and flat/wall/drop cases required')
    config=GridConfig();grids={};plans={}
    for name,views in cases.items():
        grids[name]=multiview_grid(list(views.items()),CALIBRATION,config)
        plans[name]=plan_grid(grids[name],(0,0),(0,4))
    level=depth_grid(cases['flat'][0],CALIBRATION,config)
    checks=dict(nadir_optical_floor_depth=abs(float(np.median(cases['flat'][90][118:122,158:162]))-1500)<=5,
        downward45_optical_floor_depth=abs(float(np.median(cases['flat'][45][118:122,158:162]))-1500*2**.5)<=30,
        level_near_start_unknown=level['states'][config.cell(0,0)]==-1,
        observed_sweep_start_free=bool(grids['flat']['free'][config.cell(0,0)]),
        flat_sweep_path_found=plans['flat']['status']=='path_found',
        wall_blocks_probe=plans['wall']['status']!='path_found',wall_observed=bool(np.any(grids['wall']['states']==1)),
        drop_blocks_probe=plans['drop']['status']!='path_found',drop_observed=bool(np.any(grids['drop']['states']==2)))
    checks={key:bool(value) for key,value in checks.items()}
    return dict(evidence_kind='synthetic_static_pitch_depth_controls',passed=all(checks.values()),checks=checks,
        plans=plans,interpretation='fixed primitive rendering and offline geometric probe controls; not executed navigation')


def inspect_sweep_controls(root):
    root=Path(root)
    record=json.loads((root/'controls.json').read_text(encoding='utf-8-sig'))
    if (record.get('evidence_kind')!='synthetic_calibrated_pitch_controls' or record.get('finished') is not True
            or [row['control'] for row in record['sequences']]!=['flat','wall','drop']):raise ValueError('incomplete primitive controls')
    if {path.name for path in (root/'input').iterdir()}!={f'sequence_{i:03d}' for i in range(3)}:raise ValueError('missing controls')
    poses={}
    for line in (root/'evaluation/camera_poses.jsonl').read_text(encoding='utf-8-sig').splitlines():
        row=json.loads(line);key=(row['sequence'],row['view'])
        if key in poses:raise ValueError('duplicate control pose')
        poses[key]=row
    if len(poses)!=9:raise ValueError('missing control poses')
    cases={}
    for i,row in enumerate(record['sequences']):
        name=f'sequence_{i:03d}';directory=root/'input'/name;intake=inspect_sweep_sequence(directory)
        if row['sequence']!=name or row['expected_frames']!=1 or intake['frames']!=1 or set(row['views'])!={v['name'] for v in VIEWS}:
            raise ValueError('control sequence shape mismatch')
        cases[row['control']]={}
        for view in VIEWS:
            vn=view['name'];pitch=view['pitch_down_deg'];frames=row['views'][vn]
            if len(frames)!=1:raise ValueError('control frame count mismatch')
            frame=dict(frame_id=0,source_frame_id=0,t_source_sim_s=0,camera_position_cm=[0,0,150],camera_rotation_deg=[-pitch,0,0])
            mapping=poses[(name,vn)]
            if any(mapping.get(key)!=value for key,value in dict(sequence=name,run_id=row['control'],view=vn,**frame).items()):
                raise ValueError('control source mapping mismatch')
            validate_capture_frame(frames[0],frame,mapping,intake['timestamps'][0],
                intake['views'][vn]['frame_statistics'][0],f'input/{name}/{vn}',1)
            cases[row['control']][pitch]=cv2.imread(str(directory/vn/'depth/frame_000000.png'),cv2.IMREAD_UNCHANGED)
    return evaluate_control_pixels(cases)
