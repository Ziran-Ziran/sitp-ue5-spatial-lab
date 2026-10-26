"""Read-only UE editor audit. Run via -run=pythonscript -script="... --out ...".

Reports contain privileged engine metadata and local assets, stay outside Git.
No save, actor spawning, collision repair or simulated movement is performed.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import traceback

import unreal


def vector(value):
    return [float(value.x), float(value.y), float(value.z)]


def property_or_error(obj, name):
    try:
        return str(obj.get_editor_property(name))
    except Exception as error:
        return {'error': str(error)}


def audit(map_path):
    level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if not level.load_level(map_path):
        raise RuntimeError('failed loading requested map')
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    rows = []
    meshes = {}
    for actor in actors:
        origin, extent = actor.get_actor_bounds(False)
        row = dict(name=actor.get_name(), label=actor.get_actor_label(),
                   actor_class=actor.get_class().get_name(), location_cm=vector(actor.get_actor_location()),
                   scale=vector(actor.get_actor_scale3d()), bounds_origin_cm=vector(origin), bounds_extent_cm=vector(extent),
                   tags=[str(x) for x in actor.tags])
        components = actor.get_components_by_class(unreal.StaticMeshComponent)
        row['meshes'] = []
        for component in components:
            mesh = component.static_mesh
            if mesh is None:
                continue
            path = mesh.get_path_name()
            row['meshes'].append(dict(asset=path, collision_profile=str(component.get_collision_profile_name()),
                                      collision_enabled=str(component.get_collision_enabled()),
                                      world_scale=vector(component.get_world_scale())))
            if path not in meshes:
                body = mesh.get_editor_property('body_setup')
                body_info = None
                if body:
                    geom = body.get_editor_property('agg_geom')
                    body_info = dict(collision_trace_flag=property_or_error(body, 'collision_trace_flag'),
                                     simple_counts={key: len(geom.get_editor_property(key))
                                                    for key in ('box_elems', 'sphere_elems', 'sphyl_elems', 'convex_elems')})
                meshes[path] = dict(bounds_origin_cm=vector(mesh.get_bounds().origin),
                                    bounds_extent_cm=vector(mesh.get_bounds().box_extent), body=body_info)
        rows.append(row)
    api = {}
    for name in ('NavigationSystemV1', 'NavigationPath', 'SystemLibrary', 'EditorLevelLibrary',
                 'LevelEditorSubsystem', 'StaticMeshEditorSubsystem', 'RecastNavMesh',
                 'NavMeshBoundsVolume', 'Character', 'CharacterMovementComponent', 'AIController'):
        cls = getattr(unreal, name, None)
        if cls:
            api[name] = {method: str(getattr(cls, method).__doc__)
                         for method in dir(cls) if any(token in method for token in (
                             'navigation', 'path', 'trace', 'collision', 'play', 'simulation', 'tick',
                             'bounds', 'move_to', 'movement', 'simple_move', 'world'))}
    return dict(schema_version=1, condition='engine_metadata_oracle_audit', map=map_path,
                engine_version=unreal.SystemLibrary.get_engine_version(), units='UE centimeters',
                coordinate_system='UE left-handed X-forward Y-right Z-up',
                actor_count=len(rows), actor_classes=dict(Counter(x['actor_class'] for x in rows)),
                actors=rows, meshes=meshes, api=api,
                acceptance=dict(collision=False, navmesh=False, play=False, navigation=False,
                                note='structural read-only audit; all runtime checks remain unverified'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default='/Game/MondstadtCity/Mondstadt_City_Detailed')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    output = Path(args.out)
    if output.exists():
        raise RuntimeError('output exists; choose a new audit path')
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        report = audit(args.map)
        output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        unreal.log('SITP_AUDIT_COMPLETE actors=' + str(report['actor_count']))
    except Exception:
        output.write_text(json.dumps(dict(error=traceback.format_exc()), indent=2), encoding='utf-8')
        raise


if __name__ == '__main__':
    main()
