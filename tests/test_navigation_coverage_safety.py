import json
from pathlib import Path
import tempfile
import unittest

from test_navigation_integration import open_grid

from suat_mission.automation.mission import Mission, MissionError
from suat_mission.contracts import ActionResult, Outcome
from suat_mission.navigation.map_identity import map_identity
from suat_mission.navigation.pickup_keepout import PickupKeepout
from suat_mission.navigation.station_coverage import coverage
from suat_mission.navigation.stations import (
    COLLECTION, SORTING, candidate_inventory, map_image_digest, mission_stations, save_candidate,
)


def keepout_for(grid):
    return PickupKeepout(dict(schema=1,map_identity=grid,center_map_m=[2.,2.],
                              yaw_map_rad=0.,side_m=.8,boundary_padding_m=.1,
                              rasterize_brick_array_only=True),grid)


def save_formal(path, grid, poses):
    path.write_text(json.dumps({'schema':1, 'frame_id':'map', 'map_identity':map_identity(grid),
                               'waypoints':poses}),encoding='utf-8')


class CompetitionCoverageTests(unittest.TestCase):
    def test_all_directed_pairs_show_missing_without_synthetic_poses(self):
        grid = open_grid()
        report = coverage(grid,{},.22,keepout_for(grid))
        def pairs(rows):
            return {(r['start'],r['goal']) for r in rows}
        self.assertEqual(len(report['routes']),37)
        self.assertEqual(pairs(report['routes']),{
            ('Start_A','Classification_area'),
            *(('Classification_area',s) for s in SORTING),
            *((s,c) for s in SORTING for c in COLLECTION),
            *((c,s) for c in COLLECTION for s in SORTING),
        })
        self.assertEqual(len(report['fallback_routes']),12)
        self.assertEqual(pairs(report['fallback_routes']),
                         {(s,t) for s in SORTING for t in SORTING if s!=t})
        self.assertEqual(len(report['direct_start_routes']),4)
        for row in report['routes']+report['fallback_routes']+report['direct_start_routes']:
            self.assertEqual(row['status'],'MISSING_RECORD')
            self.assertEqual(row['missing_stations'],[row['start'],row['goal']])

    def test_partial_pickup_resource_does_not_backfill_legacy_point(self):
        grid = open_grid()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            save_formal(root/'base.json',grid,{'Sorting_S':[1.,1.,0.],'Collection_A':[3.,3.,0.]})
            save_formal(root/'pickup.json',grid,{'Sorting_W':[1.,3.,0.]})
            config = {'recorded_route_file':'base.json','pickup_stations_file':'pickup.json'}
            stations = mission_stations(root,config,grid,inventory=True)
            self.assertNotIn('Sorting_S',stations)
            with self.assertRaisesRegex(ValueError,'pickup approaches'):
                mission_stations(root,config,grid)
            report = coverage(grid,stations,.22,keepout_for(grid))
            self.assertEqual(report['stations']['Sorting_S']['validation_status'],'MISSING_RECORD')
            self.assertEqual(report['stations']['Sorting_W']['pose'],[1.,3.,0.])
            missing_base = mission_stations(root,{'recorded_route_file':'missing.json'},grid,inventory=True)
            self.assertEqual(missing_base,{})

    def test_map_identity_mismatch_fails_closed(self):
        grid = open_grid()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            save_formal(root/'wrong.json',dict(grid,alignment_version='changed'),{'Sorting_W':[1.,1.,0.]})
            with self.assertRaisesRegex(ValueError,'map identity'):
                mission_stations(root,{'recorded_route_file':'wrong.json'},grid,inventory=True)

    def test_candidate_overlay_is_isolated_from_formal_station(self):
        grid = open_grid()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/'map.yaml').write_text('image: map.pgm\n',encoding='utf-8')
            (root/'map.pgm').write_bytes(b'fixture-image')
            grid['map_yaml'] = str(root/'map.yaml')
            save_formal(root/'base.json',grid,{'Sorting_W':[1.,1.,0.]})
            config = {'recorded_route_file':'base.json'}
            previous = (root/'base.json').read_bytes()
            candidate = save_candidate(root/'session','Sorting_W',[1.,1.2,.2],grid,.25,100)
            original = mission_stations(root,config,grid)
            reviewed = candidate_inventory(root/'session',original,grid,map_image_digest(grid))
            self.assertEqual(reviewed['Sorting_W']['pose'],[1.,1.2,.2])
            self.assertIs(reviewed['Sorting_W']['formal_resource'],False)
            self.assertEqual(mission_stations(root,config,grid),original)
            self.assertEqual((root/'base.json').read_bytes(),previous)
            with self.assertRaisesRegex(ValueError,'not a formal station resource'):
                mission_stations(root,{'recorded_route_file':str(candidate)},grid)
            with self.assertRaisesRegex(ValueError,'map identity'):
                candidate_inventory(root/'session',original,dict(grid,layout_version='changed'),map_image_digest(grid))

    def test_positive_but_millimetric_clearance_requires_field_review(self):
        grid = open_grid()
        report = coverage(grid,{'Sorting_W':{'pose':[2.731,2.,0.]},
                               'Collection_A':{'pose':[3.4,2.,.4]}},.22,keepout_for(grid))
        rows = [r for r in report['routes'] if (r['start'],r['goal']) == ('Sorting_W','Collection_A')]
        self.assertEqual(len(rows),1)
        row = rows[0]
        self.assertTrue(row['static_plan'])
        self.assertEqual(row['static_status'],'STATIC_PASS')
        self.assertEqual(row['status'],'FIELD_REVIEW_REQUIRED')
        self.assertGreater(row['min_clearance_lower_bound_m'],0.)
        self.assertLess(row['min_clearance_lower_bound_m'],report['field_review_floor_m'])

    def test_board_alternative_checks_approach_and_four_outgoing_legs(self):
        grid = open_grid()
        report = coverage(grid,{'Start_A':{'pose':[.7,.7,0.]},
                                'Classification_area':{'pose':[.04,1.3,0.]},
                                'Sorting_N':{'pose':[.7,2.9,.5]}},.22,keepout_for(grid),
                          board_candidates=[{'id':'latest_recording','pose':[.8,1.4,0.]}])
        self.assertEqual(report['routes'][0]['status'],'BLOCKED')
        alt = report['board_station_candidates'][0]
        self.assertEqual(len(alt['routes']),5)
        self.assertTrue(alt['routes'][0]['static_plan'])
        self.assertEqual(alt['routes'][0]['goal'],'Classification_area__latest_recording')
        self.assertEqual(sum(x['status']=='MISSING_RECORD' for x in alt['routes']),3)

    def test_formal_board_retries_preserve_navigation_deadline(self):
        class Robot:
            def __init__(self):
                self.now = 10.
                self.deadlines = []

            def log(self,text):
                pass

            def is_stopped(self):
                return False

            def board_stations(self):
                return ['first','second']

            def navigate(self,name,remaining,context):
                self.deadlines.append(context.deadline)
                self.now += 10.
                return ActionResult(context,Outcome.UNREACHABLE,'fixture')

        for competition_end, expected in ((300.,130.),(55.,55.)):
            with self.subTest(competition_end=competition_end):
                robot = Robot()
                mission = Mission(robot,clock=lambda:robot.now,deadline=competition_end)
                mission.end_time = mission.original_deadline
                with self.assertRaisesRegex(MissionError,'no board station'):
                    mission._read_board()
                self.assertEqual(robot.deadlines,[expected,expected])
                self.assertEqual(mission.end_time,competition_end)


if __name__=='__main__':
    unittest.main()
