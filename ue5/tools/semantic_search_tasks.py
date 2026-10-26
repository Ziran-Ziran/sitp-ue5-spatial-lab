"""Freeze observed-only missions separately from posthoc annotation manifests."""
import copy
import hashlib
import json
from navigation_tasks import digest_manifest
from semantic_memory import CONDITION
from semantic_search_policy import POLICIES,validate_mission


BUNDLE_KEYS={'schema_version','condition','evidence_kind','map','units','geometry_source',
             'evaluation_manifest_sha256','missions','manifest_sha256'}


def digest_bundle(bundle):
    body={key:value for key,value in bundle.items() if key!='manifest_sha256'}
    return hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def export_start_queries(source):
    return dict(schema_version=1,map=source['map'],queries=[
        dict(episode_id=episode['episode_id'],start_cm=copy.deepcopy(episode['start_cm']))
        for episode in source['episodes']])


def build_bundle(source,anchors):
    if source.get('condition')!='engine_oracle':raise ValueError('expected independent engine annotation manifest')
    episodes=source['episodes']
    if {episode['episode_id'] for episode in episodes}!=set(anchors):
        raise ValueError('anchor episodes must match every frozen task')
    missions=[]
    for episode in episodes:
        for policy in POLICIES:
            mission={key:copy.deepcopy(episode[key]) for key in ['category','start_cm','max_sim_s',
                'max_path_cm','arrival_tolerance_cm','success_radius_cm']}
            mission.update(run_id=episode['episode_id']+'-'+policy,policy=policy,
                           exploration_points_cm=copy.deepcopy(anchors[episode['episode_id']]))
            missions.append(validate_mission(mission))
    result=dict(schema_version=1,condition=CONDITION,evidence_kind='observed_only_search_missions',
        map=source['map'],units='centimeters',geometry_source='engine_navmesh_fixed_grid_or_synthetic_anchors',
        evaluation_manifest_sha256=digest_manifest(source),missions=missions)
    result['manifest_sha256']=digest_bundle(result)
    return validate_bundle(result)


def validate_bundle(bundle):
    if (set(bundle)!=BUNDLE_KEYS or bundle['schema_version']!=1 or bundle['condition']!=CONDITION
            or bundle['evidence_kind']!='observed_only_search_missions' or bundle['units']!='centimeters'
            or bundle['geometry_source']!='engine_navmesh_fixed_grid_or_synthetic_anchors'
            or not bundle['map'].startswith('/Game/SpatialLab/')
            or bundle['manifest_sha256']!=digest_bundle(bundle)):
        raise ValueError('invalid or changed observed-only mission bundle')
    sha=bundle['evaluation_manifest_sha256']
    if not isinstance(sha,str) or len(sha)!=64 or any(c not in '0123456789abcdef' for c in sha):
        raise ValueError('invalid independent evaluation manifest reference')
    missions=bundle['missions']
    if not isinstance(missions,list) or not 1<=len(missions)<=100:raise ValueError('invalid bounded mission count')
    for mission in missions:validate_mission(mission)
    if len({m['run_id'] for m in missions})!=len(missions):raise ValueError('duplicate mission runs')
    return bundle
