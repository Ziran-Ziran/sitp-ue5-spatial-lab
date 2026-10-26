"""Add controlled beacons to a fresh owned city map; oracle construction only."""
import argparse
import json
import math
from pathlib import Path
import re
import sys
import unreal
sys.path.insert(0,str(Path(__file__).resolve().parent))
from navigation_probe import point_probe,vec


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--map',required=True)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--truth',type=Path,required=True);args=parser.parse_args()
    plan=json.loads(args.plan.read_text(encoding='utf-8-sig'));rule=plan['rule']
    expected=dict(candidate_radius_cm=400,candidate_turn_offsets_deg=[0,90,180,270],category_angle_deg=dict(red_marker=-30,blue_marker=30),
        maximum_height_change_cm=100,marker_center_height_cm=100,marker_scale=[.2,.2,.6],projection_extent_cm=[75,75,150],
        capsule_radius_cm=34,capsule_half_height_cm=88,max_slope_deg=45,query_initialization='existing_native_bounded_oracle_navigation_setup')
    if (set(plan)!={'schema_version','evidence_kind','base_map','base_map_sha256','source_manifest_sha256','fixed_exposure_ev100',
            'emission_multiplier','anchors','rule'} or plan['schema_version']!=2
            or plan['evidence_kind']!='oracle_constructed_controlled_city_color_beacons' or rule!=expected
            or not 1<=len(plan['anchors'])<=12 or not -10<=plan['fixed_exposure_ev100']<=20
            or plan['emission_multiplier']!=2**(plan['fixed_exposure_ev100']-1)
            or any(not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',p) for p in (args.map,plan['base_map']))
            or args.map==plan['base_map']):raise ValueError('fixed bounded own-city beacon plan required')
    if args.out.exists() or args.truth.exists() or unreal.EditorAssetLibrary.does_asset_exist(args.map):raise ValueError('fresh overlay map and outputs required')
    if not unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(plan['base_map']):raise RuntimeError('owned base map load failed')
    world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    # A Python commandlet does not pump normal editor compilation/tickers.
    # The existing bounded setup initializes that lifecycle before collision
    # queries, and rebuilds NavMesh only for explicit fixture construction.
    navigation=json.loads(unreal.SpatialLabNavigation.build_oracle_navigation(
        world,unreal.Vector(-1300,1500,-1600),unreal.Vector(22000,27000,8500),34,176))
    if navigation.get('populated_tiles',0)<=0:raise RuntimeError('fixture query initialization has empty NavMesh')
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    existing=actors.get_all_level_actors();nav=next((a for a in existing if isinstance(a,unreal.RecastNavMesh)),None)
    if nav is None:raise RuntimeError('fixture construction requires prior validated base NavMesh')
    if any('SpatialLabMarkerRed' in [str(t) for t in a.tags] or 'SpatialLabMarkerBlue' in [str(t) for t in a.tags] for a in existing):
        raise ValueError('base city already contains controlled beacons')
    cube=unreal.load_asset('/Engine/BasicShapes/Cube');assets=unreal.AssetToolsHelpers.get_asset_tools();materials={}
    for name,rgb in (('Red',[.8,.015,.015]),('Blue',[.015,.015,.8])):
        material=assets.create_asset(name,args.map+'_Materials',unreal.Material,unreal.MaterialFactoryNew())
        material.set_editor_property('shading_model',unreal.MaterialShadingModel.MSM_UNLIT)
        expression=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector)
        expression.set_editor_property('constant',unreal.LinearColor(*(v*plan['emission_multiplier'] for v in rgb),1))
        if not unreal.MaterialEditingLibrary.connect_material_property(expression,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR):raise RuntimeError('beacon emission connection failed')
        unreal.MaterialEditingLibrary.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material);materials[name]=material
    truth=[];placements=[]
    for anchor_index,anchor in enumerate(plan['anchors']):
        if set(anchor)!={'start_cm','heading_deg'} or len(anchor['start_cm'])!=3 or not all(math.isfinite(v) for v in [*anchor['start_cm'],anchor['heading_deg']]):
            raise ValueError('finite original anchor only')
        for category in ('red_marker','blue_marker'):
            queries=[];selected=None
            for turn in rule['candidate_turn_offsets_deg']:
                angle=math.radians(anchor['heading_deg']+rule['category_angle_deg'][category]+turn)
                x=anchor['start_cm'][0]+400*math.cos(angle);y=anchor['start_cm'][1]+400*math.sin(angle)
                query=point_probe(world,x,y,anchor['start_cm'][2],nav);queries.append(dict(turn_deg=turn,**query))
                if query['status']=='ready' and abs(query['floor_cm'][2]-anchor['start_cm'][2])<=100:
                    selected=query['floor_cm'];break
                if query['status']=='ready':queries[-1]['status']='height_rejected'
            row=dict(anchor_index=anchor_index,category=category,queries=queries,status='no_valid_placement')
            if selected is not None:
                center=[selected[0],selected[1],selected[2]+100];actor=actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*center))
                actor.set_actor_label(f'SpatialLabBeacon{len(truth):03d}');actor.static_mesh_component.set_static_mesh(cube)
                actor.set_actor_scale3d(unreal.Vector(.2,.2,.6));actor.static_mesh_component.set_collision_profile_name('BlockAll')
                actor.static_mesh_component.set_material(0,materials['Red' if category=='red_marker' else 'Blue'])
                actor.tags=['SpatialLabMarkerRed' if category=='red_marker' else 'SpatialLabMarkerBlue']
                actual=vec(actor.get_actor_location());entity_id=f'fixture_{len(truth):03d}'
                truth.append(dict(entity_id=entity_id,category=category,position_cm=actual,source='owned_fixture_material_palette_ground_truth'))
                row.update(status='placed',entity_id=entity_id,position_cm=actual)
            placements.append(row)
    world.get_world_settings().set_editor_property('default_game_mode',unreal.SpatialLabGameMode)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,args.map):raise RuntimeError('owned overlay SaveAs failed')
    with args.truth.open('x',encoding='utf-8') as stream:
        json.dump(dict(schema_version=1,evidence_kind='independent_owned_marker_fixture_truth',map=args.map,entities=truth),stream,indent=2)
    with args.out.open('x',encoding='utf-8') as stream:
        json.dump(dict(schema_version=1,evidence_kind='owned_city_controlled_color_beacon_overlay',map=args.map,base_map=plan['base_map'],
            placement_source='engine_floor_capsule_and_navmesh_fixture_construction_only',navmesh_build_called=True,navigation_preparation=navigation,
            original_actor_count=len(existing),added_markers=len(truth),requested_placements=len(placements),placements=placements,
            fixed_exposure_ev100=plan['fixed_exposure_ev100'],emission_multiplier=plan['emission_multiplier'],
            truth_scope='post_run_evaluation_only_not_native_or_policy_input'),stream,indent=2,allow_nan=False)
