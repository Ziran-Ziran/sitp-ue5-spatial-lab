"""UE editor collision/NavMesh probe, explicitly privileged engine evidence.

Creates a navigation volume in memory. Saving requires an explicit --save-map;
the imported map is never overwritten. No claim of Play/character acceptance.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
import traceback

import unreal


def vec(value):
    return [float(value.x), float(value.y), float(value.z)]


def v(values):
    return unreal.Vector(*values)


def point_probe(world, x, y, reference_z, nav_data):
    hit = unreal.SystemLibrary.line_trace_single(world, v([x,y,reference_z + 200]),
                                                v([x,y,reference_z - 300]),
                                                unreal.TraceTypeQuery.ECC_VISIBILITY, False, [],
                                                unreal.DrawDebugTrace.NONE)
    if hit is None:
        return dict(query_xy_cm=[x,y], reference_z_cm=reference_z, status='no_floor')
    broken = hit.to_tuple()
    # Generated UE signature is also retained in report.api for version checks.
    blocking, initial, hit_time, distance, location, impact, normal, impact_normal = broken[:8]
    actor = broken[9]
    floor = vec(impact)
    row = dict(query_xy_cm=[x,y], reference_z_cm=reference_z, floor_cm=floor,
               normal=vec(impact_normal), floor_actor=actor.get_actor_label() if actor else None)
    if not blocking or impact_normal.z < math.cos(math.radians(45)):
        return dict(row, status='steep_or_nonblocking')
    center = [floor[0], floor[1], floor[2]+90]
    obstruction = unreal.SystemLibrary.capsule_trace_single(
        world, v(center), v([center[0],center[1],center[2]+.1]), 34, 88,
        unreal.TraceTypeQuery.ECC_VISIBILITY, False, [], unreal.DrawDebugTrace.NONE)
    row['capsule_center_cm'] = center
    if obstruction is not None:
        return dict(row, status='capsule_blocked')
    projected = unreal.NavigationSystemV1.project_point_to_navigation(
        world, v(floor), nav_data, None, v([75,75,150]))
    row['nav_projected_cm'] = vec(projected) if projected else None
    return dict(row, status='ready' if projected else 'not_on_navmesh')


def probe(args):
    subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if not subsystem.load_level(args.map):
        raise RuntimeError('failed loading requested map')
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    # Native bridge does not assume collision/nav validation from asset settings.
    navigation = json.loads(unreal.SpatialLabNavigation.build_oracle_navigation(
        world, v([-1300,1500,-1600]), v([22000,27000,8500]), 34, 176))
    nav_data = next((actor for actor in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
                     if isinstance(actor, unreal.RecastNavMesh)), None)
    report = dict(schema_version=1, condition='engine_navmesh_and_collision_oracle',
                  map=args.map, units='centimeters', coordinate_system='UE left-handed X-forward Y-right Z-up',
                  agent=dict(radius_cm=34, half_height_cm=88, spawn_floor_offset_cm=90),
                  navigation=navigation, api=dict(hit_result_to_tuple=str(unreal.HitResult.to_tuple.__doc__)),
                  play_tested=False, targets=[], probes=[])
    for actor in sorted(actors, key=lambda a:a.get_actor_label()):
        label = actor.get_actor_label()
        category = ('chair' if label.startswith('Area_MdProps_Chair') else
                    'lamp' if label.startswith('Area_MdProps_Lamp13_') else None)
        if category:
            origin, extent = actor.get_actor_bounds(False)
            reference = actor.get_actor_location()
            target = dict(entity_id=actor.get_name(), label=label, category=category,
                          source='engine_actor_name_oracle', location_cm=vec(reference),
                          bounds_origin_cm=vec(origin), bounds_extent_cm=vec(extent), approaches=[])
            for angle in range(0,360,45):
                radians = math.radians(angle)
                result = point_probe(world, reference.x+250*math.cos(radians),
                                     reference.y+250*math.sin(radians), reference.z, nav_data)
                target['approaches'].append(result)
            report['targets'].append(target)
    # Fixed exploration grid, chosen from scene bounds, not success outcomes.
    for y in range(-16000,18001,1000):
        for x in range(-16000,14001,1000):
            floor = unreal.SystemLibrary.line_trace_single(world, v([x,y,7000]), v([x,y,-10000]),
                     unreal.TraceTypeQuery.ECC_VISIBILITY, False, [], unreal.DrawDebugTrace.NONE)
            if floor is None:
                report['probes'].append(dict(query_xy_cm=[x,y],status='no_floor'))
            else:
                impact = floor.to_tuple()[5]
                report['probes'].append(point_probe(world, x,y,impact.z,nav_data))
    report['probe_counts'] = dict(Counter(p['status'] for p in report['probes']))
    report['approach_counts'] = dict(Counter(p['status'] for t in report['targets'] for p in t['approaches']))
    if args.save_map:
        if navigation.get('populated_tiles',0) <= 0:
            report['acceptance_error']='empty_navmesh'
            return report
        if args.save_map == args.map or not args.save_map.startswith('/Game/SpatialLab/'):
            raise ValueError('save only to a separate /Game/SpatialLab experiment map')
        if unreal.EditorAssetLibrary.does_asset_exist(args.save_map):
            raise ValueError('experiment map already exists; refusing overwrite')
        world.get_world_settings().set_editor_property('default_game_mode',unreal.SpatialLabGameMode)
        if not unreal.EditorLoadingAndSavingUtils.save_map(world, args.save_map):
            raise RuntimeError('failed saving experiment map')
        report['saved_map'] = args.save_map
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default='/Game/MondstadtCity/Mondstadt_City_Detailed')
    parser.add_argument('--out', required=True)
    parser.add_argument('--save-map')
    args = parser.parse_args()
    out = Path(args.out)
    if out.exists():
        raise RuntimeError('output exists; use a fresh report path')
    started = time.perf_counter()
    try:
        result = probe(args)
        result['probe_wall_s'] = time.perf_counter()-started
    except Exception:
        result = dict(error=traceback.format_exc())
        out.write_text(json.dumps(result,indent=2),encoding='utf-8')
        raise
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    if result.get('acceptance_error') or result['navigation'].get('error'):
        raise RuntimeError('navigation acceptance failed; see preserved report')
    unreal.log('SITP_NAVIGATION_PROBE_COMPLETE ' + str(result['probe_counts']))


if __name__ == '__main__':
    main()
