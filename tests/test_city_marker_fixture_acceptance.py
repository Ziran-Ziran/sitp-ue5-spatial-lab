import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from city_marker_experiment import positive_fixture


class CityMarkerFixtureAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.plan=dict(anchors=[{},{}]);self.record=dict(requested_placements=4,navmesh_build_called=True,
            navigation_preparation=dict(populated_tiles=3199),placements=[dict(category=c,status='placed',queries=[{}])
            for _ in range(2) for c in ('red_marker','blue_marker')])

    def test_validated_two_category_construction_can_enter_navigation(self):
        positive_fixture(self.record,self.plan)

    def test_missing_categories_empty_navigation_or_unbounded_queries_cannot(self):
        for mutate in (lambda r:[p.update(status='no_valid_placement') for p in r['placements']],
                lambda r:r.update(navigation_preparation=dict(populated_tiles=0)),
                lambda r:r['placements'][0].update(queries=[{}]*5)):
            record=copy.deepcopy(self.record);mutate(record)
            with self.assertRaises(ValueError):positive_fixture(record,self.plan)
