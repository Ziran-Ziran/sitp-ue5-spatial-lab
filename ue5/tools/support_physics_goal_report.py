"""Posthoc endpoint proximity to known synthetic floors; never inference/success."""
from collections import Counter
import math
from depth_execution_plan import SETTINGS
from depth_execution_report import evaluate as native_evaluate
from semantic_memory import position
from support_physics_cases import cases
from support_physics_plan import validate_bundle


def surface_proximity(case_id,endpoint):
    matching=[case for case in cases() if case['case_id']==case_id]
    if len(matching)!=1:raise ValueError('one frozen synthetic construction case required')
    case=matching[0];point=position(endpoint);start=case['start_cm']
    flat=[start[0]+100,start[1],start[2]]
    surface=[flat[0],flat[1],flat[2]+100*math.tan(math.radians(case['slope_deg']))]
    xy=math.hypot(point[0]-flat[0],point[1]-flat[1]);flat_z=abs(point[2]-flat[2]);ground_z=abs(point[2]-surface[2])
    if not all(math.isfinite(v) for v in (xy,flat_z,ground_z)):raise ValueError('finite endpoint errors required')
    return dict(original_flat_goal_cm=flat,constructed_ground_goal_cm=surface,xy_error_cm=xy,
        flat_goal_z_error_cm=flat_z,constructed_ground_z_error_cm=ground_z,
        xy_tolerance_cm=SETTINGS['final_xy_tolerance_cm'],z_tolerance_cm=SETTINGS['final_z_tolerance_cm'],
        endpoint_near_original_flat_goal=xy<=SETTINGS['final_xy_tolerance_cm'] and flat_z<=SETTINGS['final_z_tolerance_cm'],
        endpoint_near_constructed_ground_goal=xy<=SETTINGS['final_xy_tolerance_cm'] and ground_z<=SETTINGS['final_z_tolerance_cm'])


def summarize(rows):
    measured=[row for row in rows if row['measurement_valid']]
    return dict(full_rows=len(rows),measured_rows=len(measured),unmeasured_rows=len(rows)-len(measured),
        native_contract_successes=sum(row['native_success'] for row in rows),
        native_failure_counts=dict(Counter(row['native_failure_reason'] for row in rows if not row['native_success'])),
        posthoc_endpoints_near_constructed_ground=sum(row['proximity']['endpoint_near_constructed_ground_goal'] for row in measured),
        native_failed_but_endpoint_near_ground=sum(not row['native_success'] and row['proximity']['endpoint_near_constructed_ground_goal'] for row in measured))


def evaluate(bundle,record):
    native=native_evaluate(bundle,record,bundle_validator=validate_bundle)
    if not native['finished'] or any(row['failure_reason']=='invalid_execution' for row in native['runs']):raise ValueError('complete independently valid actual execution required')
    raw={row['run_id']:row['execution'] for row in record['runs']};tasks={row['run_id']:row for row in bundle['runs']};rows=[]
    for score in native['runs']:
        task=tasks[score['run_id']];execution=raw[score['run_id']]
        rows.append(dict(run_id=score['run_id'],case_id=task['source_run_id'],mode=score['mode'],
            native_success=score['success'],native_failure_reason=score['failure_reason'],measurement_valid=score['measurement_valid'],
            native_contacts=score['collisions'],proximity=surface_proximity(task['source_run_id'],execution['final_position_cm']) if score['measurement_valid'] else None))
    return dict(schema_version=1,evidence_kind='posthoc_synthetic_floor_endpoint_contract_diagnostics',
        native_manifest_sha256=bundle['manifest_sha256'],original_native_overall=native['overall'],
        original_native_by_mode=native['by_mode'],all_rows=summarize(rows),
        by_mode={mode:summarize([row for row in rows if row['mode']==mode]) for mode in sorted({row['mode'] for row in rows})},runs=rows,
        threshold_source='unchanged original final_xy_tolerance_cm/final_z_tolerance_cm, not fitted after outcomes',
        query_convention='fixed horizontal local forward +1m, not one meter of slope arclength',
        ground_truth_source='verified synthetic construction floor equation, evaluation only; not fitted support plane',
        interpretation='posthoc exploratory endpoint proximity, not navigation success, new execution, a replacement score or confirmatory improvement; native failures/contact/unmeasured rows unchanged')
