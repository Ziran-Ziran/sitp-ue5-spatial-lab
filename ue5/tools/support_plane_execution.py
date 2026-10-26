"""Explicit model-pair frozen transfer, with pose used only after pixel plans."""
import copy
from pathlib import Path
from depth_execution_plan import build_bundle as build_original,validate_bundle as validate_original,digest_bundle,local_to_world,GEOMETRY_FILES
from support_plane_replay import evaluate_sequence,geometry_sources
from support_plane_grid import MODEL
from rendered_input_report import file_hash

MODELS=('horizontal_v1',MODEL)
SCOPE='frozen_static_depth_support_plane_comparison'
PLANNER='continuous_square_shortcuts'


def source_hashes():
    names=set(geometry_sources())|set(GEOMETRY_FILES)|{'support_plane_execution.py'}
    return {name:file_hash(Path(__file__).parent/name) for name in sorted(names)}


def build_bundle(capture,*,source_job_binding):
    # Original input/pose provenance is validated independently. Its old routes
    # are discarded: BOTH models below use the same fixed continuous planner.
    bundle=build_original(capture,source_job_binding=source_job_binding)
    sources=source_hashes();runs=[];diagnostics=[];results={}
    for binding in bundle['frame_bindings']:
        sequence=binding['sequence']
        if sequence not in results:
            results[sequence]={frame['frame_id']:frame for frame in evaluate_sequence(Path(capture)/'input'/sequence)['frames']}
        frame=results[sequence][binding['frame_id']]
        for model in MODELS:
            item=frame[model];route=item['route'];gate=route['continuous_gate'];local=gate['waypoints_right_forward_m']
            key=f"{sequence}-frame_{binding['frame_id']:06d}-{model}"
            runs.append(dict(binding,run_id=key,mode=model,local_goal_m=[0,3],planning_status=route['planning']['status'],
                gate_status=gate['status'],gate_detail=gate,local_waypoints_m=local,
                world_waypoints_cm=[local_to_world(p,binding['start_cm'],binding['heading_deg']) for p in local]))
            diagnostics.append(dict(run_id=key,support_plane=item['support_plane'],
                observed_floor_cells=item['observed_floor_cells'],unknown_cells=item['unknown_cells'],
                obstacle_cells=item['obstacle_cells'],negative_cells=item['negative_cells']))
    if source_hashes()!=sources:raise ValueError('model-pair source changed during preparation')
    bundle.update(schema_version=2,execution_scope=SCOPE,ground_models=list(MODELS),planner_variant=PLANNER,
        geometry_source_sha256=sources,model_diagnostics=diagnostics,runs=runs)
    bundle['manifest_sha256']=digest_bundle(bundle)
    return validate_bundle(bundle)


def validate_bundle(bundle):
    extra={'ground_models','planner_variant','model_diagnostics'}
    if (bundle.get('schema_version')!=2 or bundle.get('execution_scope')!=SCOPE
            or bundle.get('ground_models')!=list(MODELS) or bundle.get('planner_variant')!=PLANNER
            or bundle.get('manifest_sha256')!=digest_bundle(bundle)
            or set(bundle.get('geometry_source_sha256',{}))!=set(source_hashes())):
        raise ValueError('explicit frozen model-pair bundle required')
    # Reuse the mature structural/actuation contract without relabelling the
    # actual bundle, native input, raw output, or reported model identities.
    normalized=copy.deepcopy(bundle)
    for key in extra:normalized.pop(key)
    normalized.update(schema_version=1,execution_scope='frozen_static_depth_transfer',
        geometry_source_sha256={name:bundle['geometry_source_sha256'][name] for name in GEOMETRY_FILES})
    for row in normalized['runs']:
        if row['mode'] not in MODELS:raise ValueError('unknown ground model')
        row['mode']=('level','sweep')[MODELS.index(row['mode'])]
        row['run_id']=f"{row['sequence']}-frame_{row['frame_id']:06d}-{row['mode']}"
    normalized['manifest_sha256']=digest_bundle(normalized);validate_original(normalized)
    diagnostic_keys={'run_id','support_plane','observed_floor_cells','unknown_cells','obstacle_cells','negative_cells'}
    if len(bundle['model_diagnostics'])!=len(bundle['runs']):raise ValueError('missing model diagnostics')
    for row,diagnostic in zip(bundle['runs'],bundle['model_diagnostics']):
        if (set(diagnostic)!=diagnostic_keys or diagnostic['run_id']!=row['run_id']
                or row['run_id']!=f"{row['sequence']}-frame_{row['frame_id']:06d}-{row['mode']}"):
            raise ValueError('changed model identity or ordered diagnostics')
        fit=diagnostic['support_plane']
        if (row['mode']=='horizontal_v1' and fit is not None
                or row['mode']==MODEL and (not isinstance(fit,dict) or fit.get('model')!=MODEL
                    or fit.get('status')!='supported' and row['gate_status']=='approved')):
            raise ValueError('unsupported ground model cannot execute')
    return bundle
