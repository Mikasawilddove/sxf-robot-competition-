import contextlib
import io
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/suat_mission/src'))

from suat_mission.contracts import ActionContext
from suat_mission.navigation.field_navigation import main
from suat_mission.navigation.field_routes import StaticPlanner
from suat_mission.navigation.navigation_budget import NavigationBudget
from suat_mission.navigation.navigation_gate import GOAL_PREFILTER_RADIUS_M, goal_check_radius
from suat_mission.navigation.navigation_profiles import local_planner_settings
from suat_mission.navigation.pickup_keepout import PickupKeepout
from suat_mission.navigation.station_coverage import coverage
from suat_mission.navigation.stations import load_candidate, map_image_digest, save_candidate


def open_grid():
    return dict(resolution=.05,width=80,height=80,origin=[0.,0.],origin_yaw=0.,
                negate=0,free_thresh=.19,occupied_thresh=.65,mode='trinary',
                frame_id='map',layout_version='fixture',alignment_version='fixture-alignment',data=[0]*6400)


class NavigationIntegrationTests(unittest.TestCase):
    def test_help_and_goal_radius_offline(self):
        output=io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(SystemExit) as raised:
            main(['--help'])
        self.assertEqual(raised.exception.code,0)
        self.assertIn('Sorting_S',output.getvalue())
        self.assertEqual(goal_check_radius(.22),GOAL_PREFILTER_RADIUS_M)
        self.assertAlmostEqual(goal_check_radius(.22,True),.23)
        self.assertEqual(goal_check_radius(.31),.31)

    def test_phase_budgets_do_not_spend_alignment_or_extend_judge(self):
        budget=NavigationBudget()
        end=budget.deadline(100.,185.)
        context=ActionContext('run','goal','navigate','',0,end)
        departure=budget.stage(context,'departure',100.)
        cruise=budget.stage(context,'cruise',departure.deadline)
        alignment=budget.stage(context,'align',cruise.deadline)
        self.assertEqual((departure.deadline,cruise.deadline,alignment.deadline),(130.,160.,185.))
        self.assertEqual(context.deadline,185.)
        self.assertEqual(budget.deadline(100.,600.),220.)

    def test_same_comparison_parameters_except_planner_algorithm(self):
        for approach in (False,True):
            for front in (False,True):
                rollout=local_planner_settings('rollout',approach,comparison=True,track_front=front)
                dwa=local_planner_settings('dwa',approach,comparison=True,track_front=front)
                self.assertIs(rollout.pop('use_dwa'),False)
                self.assertIs(dwa.pop('use_dwa'),True)
                self.assertEqual(rollout,dwa)

    def test_candidates_are_exclusive_and_image_bound(self):
        grid=open_grid()
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            image=root/'map.pgm'
            image.write_bytes(b'map-image')
            selected=root/'map.yaml'
            selected.write_text('image: map.pgm\n',encoding='utf-8')
            grid['map_yaml']=str(selected)
            candidate=save_candidate(root,'Sorting_S',[.8,1.,-.7],grid,.25,789)
            pose=load_candidate(candidate,'Sorting_S',grid,map_image_digest(grid))['pose']
            self.assertEqual(pose,[.8,1.,-.7])
            with self.assertRaises(FileExistsError):
                save_candidate(root,'Sorting_S',[.9,1.,-.7],grid,.25,790)
            image.write_bytes(b'changed-map-image')
            with self.assertRaisesRegex(ValueError,'mismatched'):
                load_candidate(candidate,'Sorting_S',grid,map_image_digest(grid))
            with self.assertRaisesRegex(ValueError,'blocked'):
                save_candidate(root,'Sorting_E',[.02,1.,0.],grid,.25,789)

    def test_routes_detour_around_keepout_and_missing_rows_remain(self):
        grid=open_grid()
        keepout=PickupKeepout(dict(schema=1,map_identity=grid,center_map_m=[2.,2.],
                                  yaw_map_rad=0.,side_m=.8,boundary_padding_m=.05),grid)
        start=[1.,2.,.2]
        end=[3.,2.,-.3]
        path=StaticPlanner(keepout.mask(grid),.22,keepout).plan(start,end)
        self.assertGreater(len(path),2)
        self.assertTrue(all(keepout.segment_clear(a,b,.22) for a,b in zip(path,path[1:])))
        report=coverage(grid,{'Sorting_W':{'pose':start},'Collection_C':{'pose':end}},.22,keepout)
        self.assertEqual(len(report['routes']),20)
        self.assertEqual(report['physical_navigation'],'NOT_RUN')
        row=next(x for x in report['routes'] if (x['start'],x['goal'])==('Sorting_W','Collection_C'))
        self.assertEqual(row['status'],'STATIC_PASS')
        self.assertEqual((row['start_yaw_rad'],row['goal_yaw_rad']),(.2,-.3))
        self.assertEqual(sum(x['status']=='MISSING_RECORD' for x in report['routes']),19)


if __name__=='__main__':
    unittest.main()
