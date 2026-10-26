import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from online_depth_tasks import build_tasks as source_tasks,digest
from city_marker_tasks import build_tasks,build_placement_plan,validate_city_tasks
from visual_marker_tasks import validate_tasks


class CityMarkerTasksTests(unittest.TestCase):
    def setUp(self):
        frames=[dict(sequence=f'sequence_{i//4:03d}',source_run_id='source',frame_id=i%4,source_frame_id=i%4,t_source_sim_s=i%4,
            start_cm=[100*(i//4),0,0],heading_deg=0) for i in range(8)]
        self.source=source_tasks('/Game/SpatialLab/City','a'*64,frames,exposure=13,source_binding={})
        self.binding=dict(base_map_sha256='a'*64,fixture_plan_sha256='b'*64,fixture_report_sha256='c'*64)

    def task(self,variant='legacy_centers'):
        return build_tasks('/Game/SpatialLab/CityMarkers','d'*64,self.source,self.binding,variant)

    def test_every_original_pose_has_both_categories_and_no_target_catalog(self):
        tasks=self.task();self.assertEqual(len(tasks['runs']),16);self.assertEqual(validate_tasks(tasks),tasks)
        self.assertEqual(tasks['fixed_exposure_ev100'],13);self.assertEqual(tasks['schema_version'],2)
        for row in tasks['runs']:
            self.assertNotIn('goal_cm',row);self.assertNotIn('entities',row)
        self.assertEqual({r['category'] for r in tasks['runs']},{'red_marker','blue_marker'})

    def test_removed_pose_or_privileged_field_rejected_after_rehash(self):
        for mutate in (lambda t:t['runs'].pop(),lambda t:t['runs'][0].update(goal_cm=[0,0,0]),lambda t:t['frame_bindings'].pop(),
                lambda t:t['frame_bindings'][0].update(entities=[])):
            tasks=self.task();mutate(tasks);tasks['manifest_sha256']=digest(tasks)
            with self.assertRaises(ValueError):validate_city_tasks(tasks)

    def test_source_descriptor_and_only_predeclared_variants_are_bound(self):
        tasks=self.task('continuous_square_shortcuts');self.assertEqual({r['planner_variant'] for r in tasks['runs']},{'continuous_square_shortcuts'})
        tasks['source_tasks']['manifest_sha256']='0'*64;tasks['manifest_sha256']=digest(tasks)
        with self.assertRaises(ValueError):validate_city_tasks(tasks)
        with self.assertRaises(ValueError):self.task('continuous_square')

    def test_placement_plan_is_predeclared_bounded_and_exposure_compensated(self):
        plan=build_placement_plan(self.source);self.assertEqual(plan,build_placement_plan(copy.deepcopy(self.source)))
        self.assertEqual(len(plan['anchors']),2);self.assertEqual(plan['rule']['candidate_radius_cm'],400)
        self.assertEqual(plan['rule']['candidate_turn_offsets_deg'],[0,90,180,270]);self.assertEqual(plan['emission_multiplier'],4096)
