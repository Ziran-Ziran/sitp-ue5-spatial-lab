"""Validate synthetic UE Game-world controls, including deliberate failures."""
import argparse
import json
import math
from pathlib import Path


def validate_synthetic(report):
    errors=[]
    rows=report.get('runs',[])
    names=[r.get('run_id') for r in rows]
    expected={'detour','spawn-blocked','unreachable','direct-blocked'}
    if not report.get('finished') or report.get('expected_runs')!=4 or len(names)!=4 or set(names)!=expected:
        errors.append('incomplete_or_duplicate_runs')
    if (report.get('evidence_kind')!='ue_game_world_character_movement'
            or report.get('units')!='centimeters' or report.get('position_reference')!='capsule_feet'):
        errors.append('incorrect_runtime_protocol')
    if not errors:
        cases={r['run_id']:r['execution'] for r in rows}
        detour=cases['detour']
        direct=cases['direct-blocked']
        if (detour.get('status')!='arrived' or not 1500<detour.get('continuous_path_length_cm',0)<2500
                or math.dist(detour.get('final_position_cm',[1e9]*3),[700,0,0])>50
                or detour.get('collisions')!=0 or not 0<detour.get('sim_s',0)<=20):
            errors.append('detour_control_failed')
        if cases['spawn-blocked'].get('status') not in ('spawn_not_on_navmesh','spawn_blocked'):
            errors.append('invalid_spawn_not_rejected')
        if cases['unreachable'].get('status')!='runtime_path_rejected':
            errors.append('unreachable_goal_not_rejected')
        if (direct.get('status') not in ('movement_blocked','timeout')
                or direct.get('collisions') is None or direct['collisions']<=0
                or direct.get('final_position_cm',[1e9])[0]>=0):
            errors.append('wall_collision_control_failed')
    return dict(schema_version=1,evidence_kind='synthetic_ue_game_world_integration',passed=not errors,
                expected_failures=3,errors=errors,
                scope='built-in floor and wall only; not city scene or semantic recognition acceptance')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('execution',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=validate_synthetic(json.loads(args.execution.read_text(encoding='utf-8-sig')))
    with args.out.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
