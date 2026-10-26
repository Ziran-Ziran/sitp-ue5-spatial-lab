"""Posthoc engine-ray diagnosis at recorded endpoints; never a policy input.

Uses privileged target IDs/geometry after execution. Does not change sensor
parameters, routes, task definitions, raw streams or scores.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import unreal


def xyz(value):
    return [float(value.x),float(value.y),float(value.z)]


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execution',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists(): raise RuntimeError('use a new diagnostic output')
    execution=json.loads(args.execution.read_text(encoding='utf-8-sig'))
    if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(execution['map']):
        raise RuntimeError('failed loading recorded map')
    world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    actors={a.get_name():a for a in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()}
    report=dict(evidence_kind='posthoc_engine_visibility_diagnostic',map=execution['map'],runs=[])
    for run in execution['runs']:
        state=run['execution']
        row=dict(run_id=run['run_id'],status=state['status'])
        if state['status']!='arrived':
            report['runs'].append(dict(row,diagnostic='no_arrival; retained without endpoint visibility query'))
            continue
        actor=actors[state['entity_id']]
        feet=state['final_position_cm']
        eye=unreal.Vector(feet[0],feet[1],feet[2]+150)
        center,extent=actor.get_actor_bounds(False)
        row.update(target_id=actor.get_name(),category=run['task']['category'],engine_label=actor.get_actor_label(),
                   target_root_cm=xyz(actor.get_actor_location()),bounds_center_cm=xyz(center),bounds_extent_cm=xyz(extent),
                   observer_feet_cm=feet,eye_cm=xyz(eye),root_distance_cm=math.dist(xyz(eye),xyz(actor.get_actor_location())),rays=[])
        for offset in (0,.5,-.5):
            sample=unreal.Vector(center.x,center.y,center.z+offset*extent.z)
            hit=unreal.SystemLibrary.line_trace_single(world,eye,sample,unreal.TraceTypeQuery.ECC_VISIBILITY,
                                                      True,[],unreal.DrawDebugTrace.NONE)
            hit_actor=hit.to_tuple()[9] if hit else None
            outcome='target' if hit_actor==actor else 'other' if hit_actor else 'no_hit'
            row['rays'].append(dict(offset_z_fraction=offset,sample_cm=xyz(sample),outcome=outcome,
                                   first_hit=hit_actor.get_name() if hit_actor else None,
                                   first_hit_label=hit_actor.get_actor_label() if hit_actor else None))
        row['outcomes']=dict(Counter(ray['outcome'] for ray in row['rays']))
        report['runs'].append(row)
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
