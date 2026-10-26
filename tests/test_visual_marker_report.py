import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
from test_online_depth_policy import analytic_views
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from online_depth_policy import write_observation
from visual_marker_depth_policy import decide
from visual_marker_tasks import build_tasks,CONDITION,POSE_SOURCE
from visual_marker_report import audit_execution,evaluate


class VisualMarkerReportTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup);self.root=Path(temporary.name)
        self.tasks=build_tasks('/Game/SpatialLab/Markers','a'*64);self.task=self.tasks['runs'][8];self.attempt='12345678-1234-4234-8234-123456789abc'
        self.truth=dict(schema_version=1,evidence_kind='independent_owned_marker_fixture_truth',map=self.tasks['map'],
            entities=[dict(entity_id='fixture_000',category='red_marker',position_cm=[300,-200,100],source='owned_fixture_material_palette_ground_truth'),
                      dict(entity_id='fixture_001',category='blue_marker',position_cm=[300,200,100],source='owned_fixture_material_palette_ground_truth')])
        folder=self.root/'run_008/step_000';pixels=folder/'input';pixels.mkdir(parents=True)
        for name,depth in analytic_views().items():
            (pixels/name).mkdir();rgb=np.zeros((240,320,3),dtype=np.uint8)
            if name=='level':rgb[200:206,156:162]=(0,0,255);depth[200:206,156:162]=700
            cv2.imwrite(str(pixels/name/'rgb.png'),rgb);cv2.imwrite(str(pixels/name/'depth.png'),depth)
        write_observation(pixels,0)
        request=dict(schema_version=1,attempt_id=self.attempt,run_id='immediate',step_id=0,t_sim_s=0,
            input_dir=str(pixels),category='red_marker',consecutive_empty_views=0)
        response=decide(request);self.assertEqual(response['action'],'candidate_reached')
        for name,value in [('request.json',request),('response.json',response)]: (folder/name).write_text(json.dumps(value))
        captures=[dict(view=name,captured=True,width=320,height=240,fixed_exposure_ev100=1,
            camera_position_cm=[230,-200,152],camera_rotation_deg=[-pitch,0,0]) for name,pitch in (('level',0),('down45',45),('nadir',90))]
        self.raw=dict(status='candidate_reached',agent_spawned=True,measurement_started=True,initial_position_cm=[230,-200,2],final_position_cm=[230,-200,2],
            trajectory_cm=[[230,-200,2]],actions=[dict(step_id=0,t_sim_s=0,feet_cm=[230,-200,2],heading_deg=0,folder=str(folder),captures=captures,
                capture_wall_s=.05,bridge_wall_s=.1,request=request,response=response,status='complete')],waypoint_events=[],capture_count=1,
            continuous_path_length_cm=0,collisions=0,sim_s=0,wall_s=.2,reset_wall_s=.01,reset_xy_cm=0,reset_z_cm=2,
            capture_wall_s=.05,policy_bridge_wall_s=.1,max_tracking_error_cm=0,engine_path_requests=0,intervention_sim_s=-1)
        self.record=dict(schema_version=1,condition=CONDITION,evidence_kind='ue_game_world_rgb_marker_depth_target_search',
            executor='CharacterMovement_AddMovementInput',position_reference='capsule_feet_cm',pose_source=POSE_SOURCE,
            attempt_id=self.attempt,map=self.tasks['map'],scope=self.tasks['scope'],manifest_sha256=self.tasks['manifest_sha256'],
            expected_runs=12,finished=True,runs=[dict(run_id='immediate',input=self.task,execution=self.raw)])

    def audit(self):return audit_execution(self.task,self.raw,self.root,self.attempt,8)

    def test_fresh_pixel_candidate_is_checked_against_independent_target_truth(self):
        self.assertTrue(self.audit()['candidate_reached'])
        report=evaluate(self.tasks,self.record,self.root,self.truth)
        self.assertEqual(report['overall']['runs'],12);self.assertEqual(report['overall']['successes'],1)
        self.assertEqual(report['perception_associations']['selected_observations'],1)
        self.assertEqual(report['perception_associations']['true_beacon_associations'],1)
        self.assertFalse(report['finished']);self.assertEqual(report['runs'][0]['failure_reason'],'missing_execution')
        truth=copy.deepcopy(self.truth);truth['entities'][0]['category']='blue_marker'
        self.assertEqual(evaluate(self.tasks,self.record,self.root,truth)['overall']['successes'],0)
        self.assertEqual(evaluate(self.tasks,self.record,self.root,truth)['perception_associations']['nonmatching_color_selections'],1)

    def test_fake_candidate_motion_camera_or_engine_path_cannot_pass(self):
        original=copy.deepcopy(self.raw)
        for mutate in (lambda r:r.update(engine_path_requests=1),
                lambda r:r['actions'][0]['captures'][0].update(camera_position_cm=[0,0,152]),
                lambda r:r.update(trajectory_cm=[[230,-200,2],[235,-200,2]],final_position_cm=[235,-200,2],continuous_path_length_cm=5,sim_s=1/60)):
            self.raw=copy.deepcopy(original);mutate(self.raw)
            with self.assertRaises(ValueError):self.audit()

    def test_far_true_target_does_not_become_navigation_success(self):
        truth=copy.deepcopy(self.truth);truth['entities'][0]['position_cm']=[3000,-200,100]
        self.assertEqual(evaluate(self.tasks,self.record,self.root,truth)['overall']['successes'],0)

    def test_version_two_candidate_binds_request_variant_to_task(self):
        self.tasks=build_tasks('/Game/SpatialLab/Markers','a'*64,planner_variant='continuous_square')
        self.task=self.tasks['runs'][8];event=self.raw['actions'][0]
        event['request'].update(schema_version=2,planner_variant='continuous_square');event['response']=decide(event['request'])
        folder=Path(event['folder'])
        for name,value in [('request.json',event['request']),('response.json',event['response'])]:(folder/name).write_text(json.dumps(value))
        self.assertTrue(self.audit()['candidate_reached'])
        self.task=dict(self.task,planner_variant='legacy_centers')
        with self.assertRaises(ValueError):self.audit()

    def test_wall_budget_can_preempt_a_fresh_candidate_without_fabricating_success(self):
        self.raw.update(status='wall_timeout',wall_s=60.001)
        score=self.audit()
        self.assertTrue(score['measurement_valid']);self.assertFalse(score['candidate_reached'])
        self.assertEqual(evaluate(self.tasks,self.record,self.root,self.truth)['overall']['successes'],0)
        self.raw['wall_s']=1
        with self.assertRaises(ValueError):self.audit()

    def analytic_episode(self,*,empty=False):
        self.task=self.tasks['runs'][0];feet=[-100,0,2];heading=0;empty_views=0
        path=[list(feet)];actions=[];events=[];status=None
        for step in range(12):
            timestamp=(len(path)-1)/60;folder=self.root/f'run_000/step_{step:03d}';pixels=folder/'input';pixels.mkdir(parents=True)
            for name,depth in analytic_views().items():
                (pixels/name).mkdir();rgb=np.zeros((240,320,3),dtype=np.uint8)
                if name=='level' and not empty:
                    # Independent analytic observation of a fixed true marker.
                    forward=(300-feet[0])/100;right=(-200-feet[1])/100;up=(100-feet[2])/100
                    x=round(159.5+160*right/forward);y=round(119.5+160*(1.5-up)/forward)
                    rgb[y-2:y+4,x-2:x+4]=(0,0,255);depth[y-2:y+4,x-2:x+4]=round(forward*1000)
                cv2.imwrite(str(pixels/name/'rgb.png'),rgb);cv2.imwrite(str(pixels/name/'depth.png'),depth)
            write_observation(pixels,timestamp)
            request=dict(schema_version=1,attempt_id=self.attempt,run_id=self.task['run_id'],step_id=step,t_sim_s=timestamp,
                input_dir=str(pixels),category='red_marker',consecutive_empty_views=empty_views)
            response=decide(request)
            for name,value in [('request.json',request),('response.json',response)]:(folder/name).write_text(json.dumps(value))
            captures=[dict(view=name,captured=True,width=320,height=240,fixed_exposure_ev100=1,
                camera_position_cm=[feet[0],feet[1],feet[2]+150],camera_rotation_deg=[-pitch,heading,0])
                for name,pitch in (('level',0),('down45',45),('nadir',90))]
            actions.append(dict(step_id=step,t_sim_s=timestamp,feet_cm=list(feet),heading_deg=heading,folder=str(folder),captures=captures,
                capture_wall_s=.02,bridge_wall_s=.05,request=request,response=response,status='complete'))
            if response['action'] in ('candidate_reached','stop'):
                status='candidate_reached' if response['action']=='candidate_reached' else response['reason'];break
            if response['action']=='rotate':heading+=90;empty_views+=1
            else:
                self.assertEqual(response['action'],'move');empty_views=0;origin=list(feet)
                for index,(right,forward) in enumerate(response['waypoints_right_forward_m']):
                    target=[origin[0]+100*forward,origin[1]+100*right,origin[2]];start=path[-1];count=math.ceil(math.dist(start,target)/5)
                    for tick in range(1,count+1):path.append([a+(b-a)*tick/count for a,b in zip(start,target)])
                    events.append(dict(step_id=step,waypoint_index=index,t_sim_s=(len(path)-1)/60,position_cm=list(target)))
                feet=list(path[-1])
            path.append(list(feet))  # Native next-tick stopped observation, including yaw scans.
        self.assertIsNotNone(status)
        self.raw.update(status=status,initial_position_cm=path[0],final_position_cm=path[-1],trajectory_cm=path,
            actions=actions,waypoint_events=events,capture_count=len(actions),sim_s=(len(path)-1)/60,
            continuous_path_length_cm=sum(math.dist(a,b) for a,b in zip(path,path[1:])),wall_s=len(actions)*.2,
            capture_wall_s=len(actions)*.02,policy_bridge_wall_s=len(actions)*.05)
        return audit_execution(self.task,self.raw,self.root,self.attempt,0)

    def test_multiple_pixel_updates_and_route_prefixes_reach_an_independent_marker(self):
        result=self.analytic_episode();self.assertTrue(result['candidate_reached']);self.assertGreaterEqual(result['captures'],3)
        self.assertGreater(result['path_length_cm'],100)
        self.record['runs']=[dict(run_id=self.task['run_id'],input=self.task,execution=self.raw)]
        self.assertEqual(evaluate(self.tasks,self.record,self.root,self.truth)['overall']['successes'],1)

    def test_four_empty_headings_stop_and_wrong_scan_heading_fails_audit(self):
        result=self.analytic_episode(empty=True)
        self.assertFalse(result['candidate_reached']);self.assertEqual(result['scans'],3);self.assertEqual(result['captures'],4)
        self.raw['actions'][1]['heading_deg']+=90
        with self.assertRaises(ValueError):audit_execution(self.task,self.raw,self.root,self.attempt,0)
