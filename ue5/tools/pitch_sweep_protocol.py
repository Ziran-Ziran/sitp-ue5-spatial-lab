"""Fixed static relative-pitch replay schema; standard library for UE Python."""
import copy
import hashlib
import json
import math
from rendered_replay import validate_plan,CALIBRATION,DEPTH,validate_render_settings
from semantic_memory import position

VIEWS=[dict(name='level',pitch_down_deg=0),dict(name='down45',pitch_down_deg=45),dict(name='nadir',pitch_down_deg=90)]


def sweep_digest(plan):
    return hashlib.sha256(json.dumps({key:value for key,value in plan.items() if key!='sweep_sha256'},
        sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def build_sweep(base):
    validate_plan(base)
    result=dict(schema_version=1,evidence_kind='fixed_static_pitch_sweep_plan_not_live_sensor',
        base_plan=copy.deepcopy(base),views=copy.deepcopy(VIEWS))
    result['sweep_sha256']=sweep_digest(result)
    return validate_sweep(result)


def validate_sweep(plan):
    if (set(plan)!={'schema_version','evidence_kind','base_plan','views','sweep_sha256'}
            or plan['schema_version']!=1 or plan['evidence_kind']!='fixed_static_pitch_sweep_plan_not_live_sensor'
            or plan['views']!=VIEWS or plan['sweep_sha256']!=sweep_digest(plan)):
        raise ValueError('invalid fixed static pitch-sweep plan')
    validate_plan(plan['base_plan'])
    return plan


def expected_view_metadata(settings,pitch_down_deg):
    validate_render_settings(settings)
    if type(pitch_down_deg) not in (int,float) or pitch_down_deg not in (0,45,90):raise ValueError('unknown calibrated pitch')
    return dict(schema_version=1,modality='rgbd_offline_calibrated_pitch_view',
        calibration=copy.deepcopy(CALIBRATION),depth=copy.deepcopy(DEPTH),render_settings=copy.deepcopy(settings),
        time_source='replay_source_simulated_seconds',
        timestamp_interpretation='saved engine episode time; views are static at one pose, not physical actuation or sampling',
        camera_mount='coincident_eye_pure_pitch_relative_to_level_heading',relative_pitch_down_deg=pitch_down_deg)


def view_metadata(base,pitch_down_deg):
    validate_plan(base)
    return expected_view_metadata(base['render_settings'],pitch_down_deg)


def rotation_basis(rotation):
    # UE RotationTranslationMatrix.h basis rows, including the roll sign.
    pitch,yaw,roll=[math.radians(value) for value in position(rotation)]
    sp,cp=math.sin(pitch),math.cos(pitch);sy,cy=math.sin(yaw),math.cos(yaw);sr,cr=math.sin(roll),math.cos(roll)
    return [[cp*cy,cp*sy,sp],[sr*sp*cy-cr*sy,sr*sp*sy+cr*cy,-sr*cp],
            [-(cr*sp*cy+sr*sy),cy*sr-cr*sp*sy,cr*cp]]


def rotation_matches(actual,expected):
    return max(abs(a-b) for row_a,row_b in zip(rotation_basis(actual),rotation_basis(expected))
               for a,b in zip(row_a,row_b))<=2e-5
