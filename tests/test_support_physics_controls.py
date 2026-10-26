import copy
import math
import json
import tempfile
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from support_physics_cases import cases,floor_transform,ceiling_transform
from support_physics_plan import plan_views,build_from_plans,validate_bundle
from depth_execution_plan import digest_bundle,SETTINGS


def tilted_views(slope=0,ceiling=None):
    rows,cols=np.indices((240,320));ky=(rows-119.5)/160
    result=[]
    for pitch in (0,45,90):
        theta=math.radians(pitch);forward=math.cos(theta)-ky*math.sin(theta)
        up=-math.sin(theta)-ky*math.cos(theta);den=slope*forward-up
        depth=np.divide(1.5,den,out=np.full_like(den,np.inf),where=den>0)
        if ceiling is not None:
            roof=np.divide(ceiling-1.5,up,out=np.full_like(up,np.inf),where=up>0)
            depth=np.minimum(depth,roof)
        depth[~np.isfinite(depth)|(depth<=0)|(depth>30)]=0
        result.append((pitch,np.rint(depth*1000).astype(np.uint16)))
    return result


class SupportPhysicsTests(unittest.TestCase):
    def test_fixed_cases_include_upright_sloped_ceiling_and_contract_boundaries(self):
        frozen=cases();self.assertEqual(len(frozen),10)
        self.assertEqual([c['slope_deg'] for c in frozen[:7]],[0,5,-5,10,-10,15,-15])
        self.assertEqual([c['ceiling_vertical_gap_cm'] for c in frozen[7:]],[170,220,176.5])
        self.assertEqual(frozen[-1]['slope_deg'],10)
        self.assertEqual(sum(c['supplied_reference'] for c in frozen),3)
        frozen[0]['start_cm'][1]=999;self.assertEqual(cases()[0]['start_cm'],[0,0,0])

    def test_geometry_static_closure_cannot_read_fixture_or_pose_bridge(self):
        from marker_extent_replay import source_snapshot
        sources=source_snapshot('support_physics_geometry')
        self.assertIn('support_plane_grid.py',sources)
        for name in ('support_physics_cases.py','support_physics_plan.py','support_physics_fixture.py'):
            self.assertNotIn(name,sources)

    def test_saved_standard_pitch_metadata_and_all_pixels_are_accepted(self):
        import cv2
        from support_physics_plan import read_plans
        from pitch_sweep_protocol import expected_view_metadata
        from rendered_replay import RENDER_SETTINGS
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);(root/'input').mkdir()
            for index in range(10):
                for name,(pitch,depth) in zip(('level','down45','nadir'),tilted_views()):
                    directory=root/'input'/f'sequence_{index:03d}'/name
                    (directory/'frames').mkdir(parents=True);(directory/'depth').mkdir()
                    (directory/'sensor.json').write_text(json.dumps(expected_view_metadata(RENDER_SETTINGS,pitch)),encoding='utf-8')
                    (directory/'timestamps.jsonl').write_text(json.dumps(dict(frame_id=0,t_source_sim_s=0))+'\n',encoding='utf-8')
                    cv2.imwrite(str(directory/'frames/frame_000000.png'),np.zeros((240,320,3),dtype=np.uint8))
                    cv2.imwrite(str(directory/'depth/frame_000000.png'),depth)
            plans,hashes=read_plans(root)
            self.assertEqual(len(plans),10);self.assertEqual(len(hashes),120)

    def test_transforms_make_floor_origin_and_exact_parallel_roof_gap(self):
        for case in cases():
            center,scale,pitch=floor_transform(case)
            # UE positive Pitch lifts its local +X direction.
            normal=np.array([-math.sin(math.radians(pitch)),0,math.cos(math.radians(pitch))])
            top=np.asarray(center)+10*normal
            origin=np.asarray(case['start_cm'])
            self.assertAlmostEqual(float(normal@(top-origin)),0,places=9)
            if case['ceiling_vertical_gap_cm'] is not None:
                center,scale,pitch=ceiling_transform(case)
                bottom=np.asarray(center)-10*normal
                self.assertAlmostEqual(float(normal@(bottom-origin)),case['ceiling_vertical_gap_cm']*normal[2],places=9)

    def test_independent_quantized_tilted_pixels_keep_plane_and_height_contract(self):
        for angle in (-10,-5,0,5,10):
            views=tilted_views(math.tan(math.radians(angle)))
            result=plan_views(views)
            plane=result['observed_support_plane_v3']
            self.assertEqual(plane['support_plane']['status'],'supported')
            self.assertAlmostEqual(plane['support_plane']['slope_deg'],abs(angle),delta=.05)
            self.assertEqual(plane['route']['continuous_gate']['status'],'approved')
        self.assertEqual(SETTINGS['final_z_tolerance_cm'],20)

    def test_low_roof_cannot_be_approved_and_high_roof_does_not_fill_unknown(self):
        low=plan_views(tilted_views(ceiling=1.70));high=plan_views(tilted_views(ceiling=2.2))
        for model in low:
            self.assertNotEqual(low[model]['route']['continuous_gate']['status'],'approved')
            self.assertEqual(high[model]['route']['continuous_gate']['status'],'approved')
        empty=[(p,np.zeros((240,320),dtype=np.uint16)) for p in (0,45,90)]
        self.assertNotEqual(plan_views(empty)['observed_support_plane_v3']['route']['continuous_gate']['status'],'approved')

    def bundle(self):
        plans=[plan_views(tilted_views(math.tan(math.radians(c['slope_deg'])),
            None if c['ceiling_vertical_gap_cm'] is None else c['ceiling_vertical_gap_cm']/100)) for c in cases()]
        return build_from_plans('/Game/SpatialLab/SupportPhysics','a'*64,plans,
            source_provenance={'kind':'synthetic_observed_support_physics','capture_binding':{'sha256':'b'*64}},pixel_hashes={'input.png':'c'*64})

    def test_full23_rows_preserve_real_scope_model_query_and_constant_world_z(self):
        bundle=self.bundle();self.assertEqual(validate_bundle(bundle),bundle)
        self.assertEqual(len(bundle['runs']),23);self.assertEqual(bundle['settings'],SETTINGS)
        self.assertEqual(sum(r['mode']=='supplied_reference' for r in bundle['runs']),3)
        for row in bundle['runs']:
            self.assertEqual(row['local_goal_m'],[0,1])
            self.assertTrue(all(p[2]==row['start_cm'][2] for p in row['world_waypoints_cm']))

    def test_changed_query_model_missing_failure_or_truth_cannot_pass_manifest_only(self):
        original=self.bundle()
        for mutate in (lambda b:b['runs'][0].update(local_goal_m=[0,3]),lambda b:b['runs'][0].update(mode='oracle'),
                lambda b:b['runs'].pop(),lambda b:b['settings'].update(final_z_tolerance_cm=50),lambda b:b.update(truth={}),
                lambda b:b['frame_bindings'][0].update(start_cm=[999,0,0]),lambda b:b['runs'][0].update(source_frame_id=1)):
            value=copy.deepcopy(original);mutate(value);value['manifest_sha256']=digest_bundle(value)
            with self.assertRaises(ValueError):validate_bundle(value)

    def test_acceptance_never_turns_rejected_rows_into_zero_contact_measurements(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
        from support_physics_experiment import acceptance
        from depth_execution_report import evaluate
        from test_depth_execution_report import execution
        bundle=self.bundle();runs=[]
        for row in bundle['runs']:
            status='blocked_no_progress' if row['run_id'] in ('low170-supplied_reference','slope10_low1765-supplied_reference') else 'arrived'
            runs.append(dict(run_id=row['run_id'],input=copy.deepcopy(row),execution=execution(row,status)))
        record=dict(schema_version=1,condition=bundle['condition'],execution_scope=bundle['execution_scope'],
            evidence_kind='ue_game_world_supplied_depth_waypoint_character_movement',executor='CharacterMovement_AddMovementInput',
            position_feedback_source='engine_ground_truth_for_actuation',position_reference='capsule_feet_cm',map=bundle['map'],
            manifest_sha256=bundle['manifest_sha256'],attempt_id='12345678-1234-4234-8234-123456789abc',expected_runs=23,finished=True,runs=runs)
        report=evaluate(bundle,record,bundle_validator=validate_bundle)
        self.assertTrue(acceptance(report,record)['passed'])
        changed=copy.deepcopy(report)
        next(row for row in changed['runs'] if row['run_id']=='slope10_low1765-supplied_reference').update(success=True,failure_reason=None,collisions=0)
        self.assertFalse(acceptance(changed,record)['passed'])
        rejected=[r for r in report['runs'] if r['gate_status']!='approved']
        self.assertTrue(rejected);self.assertTrue(all(r['collisions'] is None and not r['measurement_valid'] for r in rejected))

    def test_original_producer_must_guard_required_dependency_and_fixed_tick_arguments(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
        from support_physics_experiment import frozen_inputs,verify_game_arguments
        from rendered_input_report import file_hash
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);entry=root/'entry.py';helper=root/'helper.py'
            entry.write_text('import helper',encoding='utf-8');helper.write_text('VALUE=1',encoding='utf-8')
            required={entry:file_hash(entry),helper:file_hash(helper)}
            with self.assertRaises(ValueError):frozen_inputs({'input_sha256':{str(entry):file_hash(entry)}},required)
            self.assertEqual(frozen_inputs({'input_sha256':{str(p):v for p,v in required.items()}},required),required)
            project=root/'Control.uproject';bundle={'map':'/Game/SpatialLab/Control'}
            command=['UnrealEditor-Cmd.exe',str(project).replace('\\','/'),bundle['map'],'-game',
                '-SITPDepthPlan='+str(root/'tasks.json').replace('\\','/'),'-SITPResult='+str(root/'execution.json').replace('\\','/'),
                '-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic']
            verify_game_arguments(command,project,bundle,root)
            for flag in ('-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'):
                with self.assertRaises(ValueError):verify_game_arguments([v for v in command if v!=flag],project,bundle,root)
            with self.assertRaises(ValueError):verify_game_arguments(command+['-FPS=30'],project,bundle,root)


if __name__=='__main__':unittest.main()
