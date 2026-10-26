"""Isolated depth -> fixed synthetic control plans -> separate pose bridge."""
import copy
from dataclasses import asdict
from pathlib import Path
import cv2
from local_depth_grid import GridConfig
from support_plane_grid import MODEL
from support_physics_geometry import plan_views
from marker_extent_replay import source_snapshot
from pitch_sweep_report import inspect_sweep_sequence
from rendered_input_report import file_hash
from support_physics_cases import cases
from rendered_replay import CALIBRATION
from depth_execution_plan import CONDITION,SETTINGS,digest_bundle,local_to_world,validate_bundle as validate_original

MODELS=('horizontal_v1',MODEL)
SCOPE='synthetic_observed_support_plane_physics_controls'


def source_hashes():return source_snapshot('support_physics_plan')


def read_plans(capture):
    capture=Path(capture);plans=[];hashes={}
    for index,case in enumerate(cases()):
        sequence=f'sequence_{index:03d}';directory=capture/'input'/sequence
        intake=inspect_sweep_sequence(directory)
        if intake['frames']!=1 or intake['timestamps'][0]['frame_id']!=0:raise ValueError('exact one fixed frame per synthetic case required')
        if intake['calibration']!=CALIBRATION:raise ValueError('original calibrated camera required')
        views=[(pitch,cv2.imread(str(directory/name/'depth/frame_000000.png'),cv2.IMREAD_UNCHANGED))
            for name,pitch in (('level',0),('down45',45),('nadir',90))]
        plans.append(plan_views(views));hashes.update({sequence+'/'+name:sha for name,sha in intake['source_files_sha256'].items()})
        if any(file_hash(directory/name)!=sha for name,sha in intake['source_files_sha256'].items()):raise ValueError('synthetic pixels changed during planning')
    if sorted(p.name for p in (capture/'input').iterdir())!=[f'sequence_{i:03d}' for i in range(len(cases()))]:
        raise ValueError('complete fixed synthetic case set required')
    return plans,hashes


def build_from_plans(map_path,map_sha256,plans,*,source_provenance,pixel_hashes):
    if len(plans)!=len(cases()):raise ValueError('all fixed synthetic cases required')
    rows=[];diagnostics=[];bindings=[]
    for index,(case,models) in enumerate(zip(cases(),plans)):
        if set(models)!=set(MODELS):raise ValueError('both fixed ground models required')
        binding=dict(sequence=f'sequence_{index:03d}',source_run_id=case['case_id'],frame_id=0,source_frame_id=0,
            t_source_sim_s=0,start_cm=case['start_cm'],heading_deg=0);bindings.append(binding)
        for model in (*MODELS,*(('supplied_reference',) if case['supplied_reference'] else ())):
            item=models[model] if model in MODELS else None
            gate=item['route']['continuous_gate'] if item else dict(status='approved',waypoints_right_forward_m=[[0,0],[0,1]])
            local=gate['waypoints_right_forward_m'];key=case['case_id']+'-'+model
            rows.append(dict(binding,run_id=key,mode=model,local_goal_m=[0,1],
                planning_status=item['route']['planning']['status'] if item else 'supplied_not_depth_estimate',
                gate_status=gate['status'],gate_detail=gate,local_waypoints_m=local,
                world_waypoints_cm=[local_to_world(p,binding['start_cm'],0) for p in local]))
            diagnostics.append(dict(run_id=key,source='observed_depth' if item else 'deliberately_supplied_reference',
                details={k:v for k,v in item.items() if k!='route'} if item else None))
    bundle=dict(schema_version=1,condition=CONDITION,execution_scope=SCOPE,map=map_path,map_sha256=map_sha256,
        settings=copy.deepcopy(SETTINGS),source_provenance=source_provenance,geometry_source_sha256=source_hashes(),
        source_files_sha256=pixel_hashes,grid_config=asdict(GridConfig()),frame_bindings=bindings,runs=rows,
        case_definitions=cases(),model_diagnostics=diagnostics)
    bundle['manifest_sha256']=digest_bundle(bundle);return validate_bundle(bundle)


def validate_bundle(bundle):
    extra={'case_definitions','model_diagnostics'}
    if (bundle.get('schema_version')!=1 or bundle.get('execution_scope')!=SCOPE or bundle.get('settings')!=SETTINGS
            or bundle.get('case_definitions')!=cases() or bundle.get('manifest_sha256')!=digest_bundle(bundle)
            or bundle.get('source_provenance',{}).get('kind')!='synthetic_observed_support_physics'
            or set(bundle.get('geometry_source_sha256',{}))!=set(source_hashes())):
        raise ValueError('explicit frozen observed-support synthetic controls required')
    normalized=copy.deepcopy(bundle)
    for key in extra:normalized.pop(key)
    # Reuse structural/pose/connector checks only. The actual bundle, native
    # input and report retain their own scope and stricter 2000cm path budget.
    normalized.update(execution_scope='synthetic_supplied_route_controls',settings=dict(SETTINGS,max_path_cm=5000),
        source_provenance={'kind':'synthetic_supplied_routes_not_depth_estimates'},geometry_source_sha256={},source_files_sha256={},frame_bindings=[])
    for row in normalized['runs']:row['mode']='control'
    normalized['manifest_sha256']=digest_bundle(normalized);validate_original(normalized)
    expected_bindings=[dict(sequence=f'sequence_{index:03d}',source_run_id=case['case_id'],frame_id=0,source_frame_id=0,
        t_source_sim_s=0,start_cm=case['start_cm'],heading_deg=0) for index,case in enumerate(cases())]
    if bundle['frame_bindings']!=expected_bindings:raise ValueError('fixed source-pose bridge required')
    expected=[(case,model) for case in cases() for model in (*MODELS,*(('supplied_reference',) if case['supplied_reference'] else ()))]
    if len(bundle['runs'])!=len(expected) or len(bundle['model_diagnostics'])!=len(expected) or len(bundle['frame_bindings'])!=len(cases()):
        raise ValueError('all twenty model rows and three separate references required')
    for row,diagnostic,(case,model) in zip(bundle['runs'],bundle['model_diagnostics'],expected):
        binding=expected_bindings[[c['case_id'] for c in cases()].index(case['case_id'])]
        if (any(row[key]!=value for key,value in binding.items())
                or row['run_id']!=case['case_id']+'-'+model or row['mode']!=model or row['start_cm']!=case['start_cm']
                or row['heading_deg']!=0 or row['local_goal_m']!=[0,1] or diagnostic.get('run_id')!=row['run_id']):
            raise ValueError('fixed case/model/query/bridge binding required')
        if model=='supplied_reference':
            if diagnostic!={'run_id':row['run_id'],'source':'deliberately_supplied_reference','details':None} or row['local_waypoints_m']!=[[0,0],[0,1]]:
                raise ValueError('supplied reference cannot masquerade as perception')
        elif diagnostic.get('source')!='observed_depth' or not isinstance(diagnostic.get('details'),dict):
            raise ValueError('missing observed ground diagnostics')
        elif model==MODEL:
            fit=diagnostic['details'].get('support_plane')
            if not isinstance(fit,dict) or fit.get('model')!=MODEL or fit.get('status')!='supported' and row['gate_status']=='approved':
                raise ValueError('unsupported plane cannot execute')
        elif diagnostic['details'].get('support_plane') is not None:raise ValueError('horizontal model cannot claim plane fitting')
    return bundle
