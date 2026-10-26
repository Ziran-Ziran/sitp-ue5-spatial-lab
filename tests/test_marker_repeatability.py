from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from marker_repeatability import initial_groups


def run(name,category='red_marker',action='reject',pixel='a',empty=0):
    hashes={v+'/'+kind+'.png':pixel*64 for v in ('level','down45','nadir') for kind in ('rgb','depth')}
    request=dict(schema_version=3,attempt_id=name,run_id=name,step_id=0,t_sim_s=0,
        category=category,consecutive_empty_views=empty,planner_variant='legacy_centers',goal_horizon_policy='fixed_three_meters')
    response=dict(request,input_files_sha256=dict(hashes,**{'observation.json':name}),action=action,geometry=None,perception_and_planning_wall_s=1)
    return dict(run_id=name,execution={'actions':[dict(step_id=0,status='complete',request=request,response=response)]})


class MarkerRepeatabilityTests(unittest.TestCase):
    def test_duplicate_pixels_are_not_independent_categories_or_episodes(self):
        record={'runs':[run('one'),run('two'),run('three',category='blue_marker')]}
        result=initial_groups(record)
        self.assertEqual(result['full_runs'],3);self.assertEqual(result['distinct_initial_six_png_groups'],1)
        self.assertEqual(result['distinct_category_input_groups'],2);self.assertEqual(result['conflicting_same_input_responses'],[])

    def test_changed_decision_with_identical_inputs_is_reported_not_hidden(self):
        result=initial_groups({'runs':[run('one'),run('two',action='move')]})
        self.assertEqual(len(result['conflicting_same_input_responses']),1)

    def test_pixels_scan_state_and_category_separate_groups_missing_keeps_denominator(self):
        record={'runs':[run('one'),run('two',pixel='b'),run('three',empty=1),dict(run_id='missing',execution={'actions':[]})]}
        result=initial_groups(record)
        self.assertEqual(result['full_runs'],4);self.assertEqual(result['distinct_initial_six_png_groups'],2)
        self.assertEqual(result['distinct_category_input_groups'],3);self.assertEqual(result['missing_initial_observations'],['missing'])

    def test_duplicates_and_forged_identity_or_incomplete_pixel_hashes_are_rejected(self):
        with self.assertRaises(ValueError):initial_groups({'runs':[run('same'),run('same')]})
        for mutate in (lambda r:r['execution']['actions'][0]['response'].update(run_id='other'),
                lambda r:r['execution']['actions'][0]['response']['input_files_sha256'].pop('level/depth.png')):
            value=run('one');mutate(value)
            with self.assertRaises(ValueError):initial_groups({'runs':[value]})


if __name__=='__main__':unittest.main()
