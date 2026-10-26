import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from online_depth_policy import decide,route_prefix,world_goal_to_local,write_observation


def analytic_views(wall=None):
    y,x=np.indices((240,320));down=(y-119.5)/160
    result={}
    for name,pitch in (('level',0),('down45',45),('nadir',90)):
        theta=math.radians(pitch);forward=math.cos(theta)-down*math.sin(theta)
        up=-math.sin(theta)-down*math.cos(theta)
        with np.errstate(divide='ignore',invalid='ignore'):
            depth=np.where(up<0,-1.5/up,np.inf)
            if wall is not None:
                hit=wall/forward;height=1.5+up*hit
                depth=np.minimum(depth,np.where((forward>0)&(height>=0)&(height<=2),hit,np.inf))
        result[name]=np.where((depth>0)&(depth<=30),np.round(depth*1000),0).astype(np.uint16)
    return result


class OnlineDepthPolicyTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name)/'input';self.root.mkdir()
        for name,depth in analytic_views().items():
            view=self.root/name;view.mkdir()
            cv2.imwrite(str(view/'rgb.png'),np.zeros((240,320,3),dtype=np.uint8))
            cv2.imwrite(str(view/'depth.png'),depth)
        write_observation(self.root,0)
        self.request=dict(schema_version=1,attempt_id='12345678-1234-4234-8234-123456789abc',run_id='floor',step_id=0,
            input_dir=str(self.root),goal_right_forward_m=[0,3],mode='receding_sweep',t_sim_s=0)

    def test_floor_produces_bounded_safe_prefix_and_full_initial_route(self):
        answer=decide(self.request)
        self.assertEqual(answer['action'],'move');self.assertEqual(answer['waypoints_right_forward_m'][0],[0,0])
        points=answer['waypoints_right_forward_m']
        self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(points,points[1:])),1)
        request=dict(self.request,mode='initial_sweep_once')
        self.assertEqual(decide(request)['waypoints_right_forward_m'][-1],[0,3])

    def test_new_pixels_change_decision_and_old_hash_cannot_be_reused(self):
        for name,depth in analytic_views(wall=1.8).items():cv2.imwrite(str(self.root/name/'depth.png'),depth)
        with self.assertRaises(ValueError):decide(self.request)
        (self.root/'observation.json').unlink();write_observation(self.root,1)
        self.request['step_id']=1
        self.request['t_sim_s']=1
        answer=decide(self.request)
        self.assertEqual(answer['action'],'reject')
        self.assertIn(answer['reason'],('planning_no_path','planning_goal_obstacle','planning_goal_clearance_blocked','planning_goal_unobserved'))

    def test_unknown_floor_is_failure_and_privileged_fields_are_rejected(self):
        for name in analytic_views():cv2.imwrite(str(self.root/name/'depth.png'),np.zeros((240,320),dtype=np.uint16))
        (self.root/'observation.json').unlink();write_observation(self.root,0)
        self.assertEqual(decide(self.request)['reason'],'planning_start_unobserved')
        for key in ('map','pose','entities','future_frames'):
            request=copy.deepcopy(self.request);request[key]={}
            with self.subTest(key=key),self.assertRaises(ValueError):decide(request)

    def test_prefix_interpolates_without_shortcutting_a_corner(self):
        prefix=route_prefix([[0,0],[0,.6],[.8,.6]],1)
        self.assertEqual(prefix[:2],[[0,0],[0,.6]])
        self.assertAlmostEqual(prefix[-1][0],.4);self.assertEqual(prefix[-1][1],.6)
        self.assertEqual(route_prefix([[0,0]],1),[[0,0]])

    def test_supervised_relative_goal_rotation_is_separate_from_pixel_geometry(self):
        self.assertEqual(world_goal_to_local([100,200,0],[100,500,0],90),[0,3])
        actual=world_goal_to_local([100,200,0],[400,200,0],90)
        self.assertAlmostEqual(actual[0],-3);self.assertAlmostEqual(actual[1],0)

    def test_noncanonical_identity_and_extra_view_files_are_rejected(self):
        for change in (dict(attempt_id='old'),dict(step_id=True),dict(goal_right_forward_m=[0,float('nan')])):
            with self.subTest(change=change),self.assertRaises(ValueError):decide(dict(self.request,**change))
        (self.root/'nadir/poses.jsonl').write_text('{}')
        with self.assertRaises(ValueError):decide(self.request)

    def test_version_two_binds_index_policy_without_changing_raw_goal(self):
        goal=[-4.218847493575595e-15,3.0000000000002798]
        for policy,expected in [('legacy_floor',[20,19]),('boundary_snap_1e-9_cell',[20,20])]:
            request=dict(self.request,schema_version=2,query_cell_policy=policy,goal_right_forward_m=goal,mode='initial_sweep_once')
            answer=decide(request)
            self.assertEqual(answer['schema_version'],2);self.assertEqual(answer['query_cell_policy'],policy)
            self.assertEqual(answer['planning']['query_indexing']['selected_cells'][1],expected)
            self.assertEqual(answer['continuous_gate']['waypoints_right_forward_m'][-1],goal)
        with self.assertRaises(ValueError):decide(dict(self.request,schema_version=2))
        with self.assertRaises(ValueError):decide(dict(self.request,query_cell_policy='legacy_floor'))
