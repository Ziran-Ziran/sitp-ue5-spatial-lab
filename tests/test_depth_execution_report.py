import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from depth_execution_plan import build_controls,CONDITION
from depth_execution_report import evaluate,control_acceptance


def execution(row,status='arrived'):
    if row['gate_status']!='approved' or status=='spawn_blocked':
        return dict(status='not_executed_'+row['gate_status'] if row['gate_status']!='approved' else status,
            agent_spawned=False,measurement_started=False,initial_position_cm=None,final_position_cm=None,
            trajectory_cm=[],waypoint_events=[],continuous_path_length_cm=None,collisions=None,sim_s=None,wall_s=None,
            reset_wall_s=None,reset_xy_cm=None,reset_z_cm=None,max_tracking_error_cm=None,engine_path_requests=0)
    points=row['world_waypoints_cm'];trajectory=[list(points[0])];events=[]
    for index,target in enumerate(points):
        start=trajectory[-1];steps=math.ceil(math.dist(start,target)/5)
        for step in range(1,steps+1):trajectory.append([a+(b-a)*step/steps for a,b in zip(start,target)])
        events.append(dict(waypoint_index=index,t_sim_s=(len(trajectory)-1)/60,position_cm=list(target)))
    if status=='blocked_no_progress':
        trajectory=[list(points[0]) for _ in range(121)];events=events[:1]
    return dict(status=status,agent_spawned=True,measurement_started=True,initial_position_cm=trajectory[0],
        final_position_cm=trajectory[-1],trajectory_cm=trajectory,waypoint_events=events,
        continuous_path_length_cm=sum(math.dist(a,b) for a,b in zip(trajectory,trajectory[1:])),
        collisions=1 if status=='blocked_no_progress' else 0,sim_s=(len(trajectory)-1)/60,wall_s=.1,
        reset_wall_s=.01,reset_xy_cm=0,reset_z_cm=0,max_tracking_error_cm=0,engine_path_requests=0)


class DepthExecutionReportTests(unittest.TestCase):
    def setUp(self):
        self.bundle=build_controls('/Game/SpatialLab/Controls','a'*64)
        self.record=dict(schema_version=1,condition=CONDITION,execution_scope=self.bundle['execution_scope'],
            evidence_kind='ue_game_world_supplied_depth_waypoint_character_movement',executor='CharacterMovement_AddMovementInput',
            position_feedback_source='engine_ground_truth_for_actuation',position_reference='capsule_feet_cm',
            map=self.bundle['map'],manifest_sha256=self.bundle['manifest_sha256'],attempt_id='12345678-1234-4234-8234-123456789abc',
            expected_runs=5,finished=True,runs=[])
        for row in self.bundle['runs']:
            status={'direct-blocked':'blocked_no_progress','spawn-blocked':'spawn_blocked'}.get(row['run_id'],'arrived')
            self.record['runs'].append(dict(run_id=row['run_id'],input=copy.deepcopy(row),execution=execution(row,status)))

    def test_control_successes_and_expected_failures_are_separate(self):
        report=evaluate(self.bundle,self.record)
        self.assertTrue(report['finished']);self.assertEqual(report['overall']['successes'],2)
        self.assertTrue(control_acceptance(report,self.record)['passed'])
        self.assertEqual(report['overall']['collision_observed_runs'],3)

    def test_claimed_arrival_without_actual_trajectory_or_waypoint_feedback_fails(self):
        self.record['runs'][0]['execution']['waypoint_events']=[]
        report=evaluate(self.bundle,self.record)
        self.assertFalse(report['runs'][0]['success'])
        self.assertEqual(report['runs'][0]['failure_reason'],'invalid_execution')

    def test_missing_attempt_is_retained_in_denominator(self):
        self.record['runs'].pop(0)
        report=evaluate(self.bundle,self.record)
        self.assertFalse(report['finished']);self.assertEqual(report['overall']['runs'],5)
        self.assertEqual(report['runs'][0]['failure_reason'],'missing_execution')

    def test_unmeasured_conditions_have_unknown_aggregate_motion_and_contacts(self):
        self.record['runs']=[]
        overall=evaluate(self.bundle,self.record)['overall']
        self.assertEqual(overall['measured_runs'],0)
        self.assertIsNone(overall['collision_events'])
        self.assertIsNone(overall['total_path_length_cm'])

    def test_wrong_evidence_pose_source_or_manifest_is_rejected(self):
        for key,value in [('evidence_kind','ue_game_world_character_movement'),('position_feedback_source','visual_estimate'),('manifest_sha256','b'*64)]:
            record=copy.deepcopy(self.record);record[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):evaluate(self.bundle,record)

    def test_false_distance_and_navmesh_requests_cannot_be_scored_as_success(self):
        self.record['runs'][0]['execution']['continuous_path_length_cm']=0
        self.record['runs'][-1]['execution']['engine_path_requests']=1
        report=evaluate(self.bundle,self.record)
        self.assertFalse(report['runs'][0]['success']);self.assertFalse(report['runs'][-1]['success'])

    def test_native_arrival_must_satisfy_wall_tracking_and_vertical_budgets(self):
        for key,value in [('wall_s',31),('max_tracking_error_cm',100)]:
            record=copy.deepcopy(self.record);record['runs'][0]['execution'][key]=value
            with self.subTest(key=key):
                self.assertEqual(evaluate(self.bundle,record)['runs'][0]['failure_reason'],'invalid_execution')
        record=copy.deepcopy(self.record);raw=record['runs'][0]['execution']
        raw['trajectory_cm'][20][2]+=60
        raw['continuous_path_length_cm']=sum(math.dist(a,b) for a,b in zip(raw['trajectory_cm'],raw['trajectory_cm'][1:]))
        self.assertEqual(evaluate(self.bundle,record)['runs'][0]['failure_reason'],'invalid_execution')
