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
from online_depth_policy import decide,write_observation
from online_depth_tasks import build_controls
from online_depth_report import audit_execution


class OnlineDepthReportTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup);self.root=Path(temporary.name)
        self.task=build_controls('/Game/SpatialLab/Controls','a'*64)['runs'][0]
        self.attempt='12345678-1234-4234-8234-123456789abc'
        folder=self.root/'run_000/step_000';pixels=folder/'input';pixels.mkdir(parents=True)
        for name,depth in analytic_views().items():
            (pixels/name).mkdir();cv2.imwrite(str(pixels/name/'depth.png'),depth)
            cv2.imwrite(str(pixels/name/'rgb.png'),np.zeros((240,320,3),dtype=np.uint8))
        write_observation(pixels,0)
        request=dict(schema_version=1,attempt_id=self.attempt,run_id=self.task['run_id'],step_id=0,t_sim_s=0,
            input_dir=str(pixels),goal_right_forward_m=[0,3],mode=self.task['mode'])
        answer=decide(request)
        (folder/'request.json').write_text(json.dumps(request));(folder/'response.json').write_text(json.dumps(answer))
        trajectory=[[0,0,2]];events=[]
        for i,point in enumerate(answer['waypoints_right_forward_m']):
            target=[point[1]*100,point[0]*100,2];start=trajectory[-1];steps=math.ceil(math.dist(start,target)/5)
            for step in range(1,steps+1):trajectory.append([a+(b-a)*step/steps for a,b in zip(start,target)])
            events.append(dict(step_id=0,waypoint_index=i,t_sim_s=(len(trajectory)-1)/60,position_cm=target))
        captures=[dict(view=name,captured=True,width=320,height=240,fixed_exposure_ev100=1,
            camera_position_cm=[0,0,152],camera_rotation_deg=[-pitch,0,0]) for name,pitch in (('level',0),('down45',45),('nadir',90))]
        self.raw=dict(status='arrived',agent_spawned=True,measurement_started=True,initial_position_cm=trajectory[0],final_position_cm=trajectory[-1],
            trajectory_cm=trajectory,actions=[dict(step_id=0,t_sim_s=0,feet_cm=trajectory[0],folder=str(folder),captures=captures,
                capture_wall_s=.05,bridge_wall_s=.1,request=request,response=answer,status='complete')],waypoint_events=events,capture_count=1,
            continuous_path_length_cm=sum(math.dist(a,b) for a,b in zip(trajectory,trajectory[1:])),collisions=0,
            sim_s=(len(trajectory)-1)/60,wall_s=.2,reset_wall_s=.01,reset_xy_cm=0,reset_z_cm=2,
            capture_wall_s=.05,policy_bridge_wall_s=.1,max_tracking_error_cm=0,engine_path_requests=0,intervention_sim_s=-1)

    def audit(self):return audit_execution(self.task,self.raw,self.root,self.attempt,0,exposure=1)

    def test_actual_capture_decision_and_route_feedback_are_reproducible(self):
        self.assertTrue(self.audit()['success'])

    def test_version_two_policy_is_bound_even_when_routes_happen_to_match(self):
        self.task=dict(self.task,query_cell_policy='boundary_snap_1e-9_cell');event=self.raw['actions'][0]
        event['request'].update(schema_version=2,query_cell_policy='boundary_snap_1e-9_cell')
        event['response']=decide(event['request']);folder=Path(event['folder'])
        for name,value in [('request.json',event['request']),('response.json',event['response'])]:(folder/name).write_text(json.dumps(value))
        self.assertTrue(self.audit()['success'])
        self.task['query_cell_policy']='legacy_floor'
        with self.assertRaises(ValueError):self.audit()

    def test_fake_goal_camera_time_or_route_is_rejected(self):
        original=copy.deepcopy(self.raw)
        for mutate in (lambda r:r['actions'][0]['request'].update(goal_right_forward_m=[0,2]),
                       lambda r:r['actions'][0]['captures'][1].update(camera_position_cm=[1,0,152]),
                       lambda r:r['actions'][0].update(t_sim_s=1),
                       lambda r:r['waypoint_events'].pop()):
            self.raw=copy.deepcopy(original);mutate(self.raw)
            with self.assertRaises(ValueError):self.audit()

    def test_changed_pixels_false_distance_or_navmesh_cannot_pass(self):
        for key,value in [('continuous_path_length_cm',0),('engine_path_requests',1),('wall_s',61)]:
            original=self.raw[key];self.raw[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.audit()
            self.raw[key]=original
        path=Path(self.raw['actions'][0]['request']['input_dir'])/'nadir/depth.png'
        cv2.imwrite(str(path),np.zeros((240,320),dtype=np.uint16))
        with self.assertRaises(ValueError):self.audit()

    def test_geometry_rejection_cannot_hide_subsequent_uncommanded_motion(self):
        event=self.raw['actions'][0];pixels=Path(event['request']['input_dir'])
        for name,depth in analytic_views(wall=1.8).items():cv2.imwrite(str(pixels/name/'depth.png'),depth)
        (pixels/'observation.json').unlink();write_observation(pixels,0)
        answer=decide(event['request']);self.assertEqual(answer['action'],'reject')
        event['response']=answer;(Path(event['folder'])/'response.json').write_text(json.dumps(answer))
        self.raw.update(status='geometry_'+answer['reason'],waypoint_events=[],trajectory_cm=[[0,0,2]],final_position_cm=[0,0,2],
            continuous_path_length_cm=0,sim_s=0)
        self.assertTrue(self.audit()['measurement_valid'])
        self.raw.update(trajectory_cm=[[0,0,2],[0,5,2],[0,10,2]],final_position_cm=[0,10,2],continuous_path_length_cm=10,sim_s=2/60)
        with self.assertRaises(ValueError):self.audit()

    def test_other_failures_still_need_pixel_supported_motion(self):
        self.raw.update(status='blocked_no_progress',actions=[],capture_count=0,capture_wall_s=0,policy_bridge_wall_s=0,waypoint_events=[])
        with self.assertRaises(ValueError):self.audit()

    def test_wall_budget_preempts_geometry_rejection_at_the_same_stopped_pose(self):
        event=self.raw['actions'][0];pixels=Path(event['request']['input_dir'])
        for name,depth in analytic_views(wall=1.8).items():cv2.imwrite(str(pixels/name/'depth.png'),depth)
        (pixels/'observation.json').unlink();write_observation(pixels,0)
        response=decide(event['request']);self.assertEqual(response['action'],'reject')
        event['response']=response;(Path(event['folder'])/'response.json').write_text(json.dumps(response))
        self.raw.update(status='wall_timeout',waypoint_events=[],trajectory_cm=[[0,0,2]],final_position_cm=[0,0,2],
            continuous_path_length_cm=0,sim_s=0,wall_s=60.001)
        score=self.audit();self.assertTrue(score['measurement_valid']);self.assertFalse(score['success'])
        self.raw['wall_s']=1
        with self.assertRaises(ValueError):self.audit()
