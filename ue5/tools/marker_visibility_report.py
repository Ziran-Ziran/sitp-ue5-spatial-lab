"""Posthoc perception stages on saved pixels and privileged fixture box truth."""
from collections import Counter
import math
from pathlib import Path
import cv2
import numpy as np
from online_depth_policy import inspect_observation,CALIBRATION
from marker_extent import VARIANTS
from marker_visibility import depth_consistent_box_masks
from visual_marker import detect_markers
from visual_marker_report import local_to_world,associate_selections
from rendered_input_report import file_hash


def color_masks(color):
    hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV);h,s,v=cv2.split(hsv);bright=(s>=120)&(v>=50)
    return {'red_marker':bright&((h<=10)|(h>=170)),'blue_marker':bright&(h>=100)&(h<=130)}


def control_acceptance(report):
    rows=report['observations'];summary=report['summary'];original=report['original_navigation_overall']
    positives=[f'start_{index:02d}-{category}' for index in range(4) for category in ('red_marker','blue_marker')]+['immediate']
    checks=dict(full_fixed_controls=summary['task_runs']==summary['saved_runs']==original['runs']==12,
        all_saved_observations_accounted=summary['observations']==summary['complete_observations']==original['captures'])
    for key in positives:
        checks[key+'_has_depth_and_accepted_color_support']=any(any(entity['requested'] and entity['depth_consistent_pixels']>=9
            and entity['models']['hsv_depth_v1']['accepted_supported_pixels']>=9 for entity in row['entities']) for row in rows if row['run_id']==key)
    for key in ('missing','occluded'):
        selected=[row for row in rows if row['run_id']==key]
        checks[key+'_has_no_requested_box_depth_support']=bool(selected) and all(not any(entity['requested'] and entity['depth_consistent_pixels']>=9
            for entity in row['entities']) for row in selected)
    checks['blocked_spawn_has_no_observation']=not any(row['run_id']=='spawn' for row in rows)
    return dict(evidence_kind='saved_neutral_fixture_depth_projection_proxy_controls',passed=all(checks.values()),checks=checks,
        interpretation='same-pose depth projection and controlled color/hidden/occluder diagnostics; not exact instance masks or new physical policy trials')


def analyze_record(tasks,record,truth):
    expected={run['run_id']:run for run in tasks['runs']}
    if not 1<=len(expected)<=200 or len(expected)!=len(tasks['runs']):raise ValueError('bounded complete task identities required')
    rows=[];seen=set()
    for run in record['runs']:
        key=run['run_id']
        if key not in expected or key in seen or run['input']!=expected[key]:raise ValueError('changed/duplicate diagnostic task')
        seen.add(key);task=expected[key];actions=run['execution']['actions']
        if len(actions)>12:raise ValueError('bounded saved observations required')
        # Privileged truth affects evaluation only. Detectors below receive only
        # RGB, depth, intrinsics and the declared counterfactual size variant.
        active=[entity for entity in truth['entities'] if not (task['environment_case']=='red_hidden' and entity['category']=='red_marker')]
        for event in actions:
            row=dict(run_id=key,step_id=event['step_id'],category=task['category'],status=event['status'],entities=[],models={})
            rows.append(row)
            if event['status']!='complete':continue
            request=event['request'];directory=Path(request['input_dir']);views,hashes=inspect_observation(directory,request['t_sim_s'])
            if request['run_id']!=key or request['category']!=task['category']:raise ValueError('diagnostic request changes category/run')
            color=cv2.imread(str(directory/'level/rgb.png'),cv2.IMREAD_COLOR);masks=color_masks(color)
            labels={category:cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity=8)[1] for category,mask in masks.items()}
            reports={variant:detect_markers(color,views[0][1],CALIBRATION,perception_variant=variant) for variant in VARIANTS}
            if reports['hsv_depth_v1']!=event['response']['detection_report']:raise ValueError('diagnostic baseline differs from original detector response')
            level=[capture for capture in event['captures'] if capture['view']=='level']
            if len(level)!=1:raise ValueError('one actual level camera required')
            rotation=level[0]['camera_rotation_deg'];heading=event['heading_deg']
            if abs(rotation[0])>1e-8 or abs(rotation[2])>1e-8 or abs(math.remainder(rotation[1]-heading,360))>1e-8:
                raise ValueError('yaw-only proxy requires actual level camera')
            proxy=depth_consistent_box_masks(views[0][1],CALIBRATION,camera_position_cm=level[0]['camera_position_cm'],heading_deg=heading,entities=active)
            row.update(input_files_sha256=hashes,ambiguous_pixels=proxy['ambiguous_depth_consistent_pixels'])
            for variant,report in reports.items():
                targets=[item for item in report['detections'] if item['category']==task['category']];selected=targets[0] if targets else None
                associations=[]
                if selected is not None:
                    point=local_to_world(selected['position_right_forward_up_m'],event['feet_cm'],heading)
                    associations=associate_selections([dict(step_id=event['step_id'],category=task['category'],position_cm=point)],active)
                row['models'][variant]=dict(selected=selected,selection_associations=associations,
                    components_total=report['components_total'],components_processed=report['components_processed'],
                    component_budget_exhausted=report['component_budget_exhausted'],
                    rejected_component_reasons=dict(Counter(item['reason'] for item in report['rejected'])))
            for entity in proxy['entities']:
                category=entity['category'];mask=entity['depth_consistent_mask']
                item={name:entity[name] for name in ('entity_id','category','status','projected_pixels','depth_consistent_pixels')}
                item.update(requested=category==task['category'],hsv_supported_pixels=int((mask&masks[category]).sum()),models={})
                # One label histogram replaces per-component full-image scans.
                # Label 0 is non-category background and never a detection.
                counts=np.bincount(labels[category][mask],minlength=int(labels[category].max())+1)
                overlapping=set(np.flatnonzero(counts).tolist())-{0}
                for variant,report in reports.items():
                    accepted=[d for d in report['detections'] if d['category']==category]
                    ids=[d['component_id'] for d in accepted]
                    selected=row['models'][variant]['selected']
                    selected_pixels=int(counts[selected['component_id']]) if selected and selected['category']==category else 0
                    processed={d['component_id'] for d in report['detections']+report['rejected'] if d['category']==category}
                    reasons=Counter(d['reason'] for d in report['rejected'] if d['category']==category and counts[d['component_id']]>0)
                    item['models'][variant]=dict(accepted_supported_pixels=int(counts[ids].sum()),selected_supported_pixels=selected_pixels,
                        overlapping_rejected_component_reasons=dict(reasons),unprocessed_overlapping_component_ids=sorted(overlapping-processed))
                row['entities'].append(item)
            if any(file_hash(directory/name)!=sha for name,sha in hashes.items()):raise ValueError('diagnostic source pixels changed')
    complete=[row for row in rows if row['models']];requested=[entity for row in complete for entity in row['entities'] if entity['requested']]
    eligible=[entity for entity in requested if entity['depth_consistent_pixels']>=9]
    summary=dict(task_runs=len(expected),saved_runs=len(seen),observations=len(rows),complete_observations=len(complete),
        observations_with_requested_proxy_target=sum(any(e['requested'] and e['depth_consistent_pixels']>=9 for e in row['entities']) for row in complete),
        requested_proxy_eligible_entity_observation_pairs=len(eligible),requested_projected_pixels=sum(e['projected_pixels'] for e in requested),
        requested_proxy_eligible_pairs_with_color_pixels=sum(e['hsv_supported_pixels']>=9 for e in eligible),
        requested_proxy_pixels=sum(e['depth_consistent_pixels'] for e in requested),requested_proxy_hsv_pixels=sum(e['hsv_supported_pixels'] for e in requested))
    for variant in VARIANTS:
        associations=[a for row in complete for a in row['models'][variant]['selection_associations']]
        summary[variant]=dict(selected_observations=sum(row['models'][variant]['selected'] is not None for row in complete),
            true_point_associations=sum(a['matched'] for a in associations),nonmatching_point_associations=sum(not a['matched'] for a in associations),
            proxy_eligible_pairs_with_accepted_pixels=sum(e['models'][variant]['accepted_supported_pixels']>=9 for e in eligible),
            proxy_eligible_pairs_with_selected_pixels=sum(e['models'][variant]['selected_supported_pixels']>=9 for e in eligible),
            proxy_eligible_pairs_with_unprocessed_color_components=sum(bool(e['models'][variant]['unprocessed_overlapping_component_ids']) for e in eligible),
            overlapping_rejected_component_reasons=dict(sum((Counter(e['models'][variant]['overlapping_rejected_component_reasons']) for e in eligible),Counter())))
    return dict(schema_version=1,evidence_kind='posthoc_depth_consistent_known_box_perception_diagnostics',summary=summary,observations=rows,
        proxy_method='level-camera optical front-depth consistency with known axis-aligned20x20x60cmfixtureboxes;2mmdepthbound;ambiguousboxesexcluded',
        interpretation='privileged geometric visibility proxy and saved-observation counterfactual detector stages, not exact instance/material masks, visible-target recall, calibrated confidence, independent trials, or new navigation outcomes')
