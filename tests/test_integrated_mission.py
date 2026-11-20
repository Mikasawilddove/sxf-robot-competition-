"""Exercise the formal adapter with simulated camera, navigation and serial I/O."""
import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from test_formal_startup import Message, Port, module, StartupFixture
from test_navigation_integration import open_grid
from suat_mission.navigation.map_identity import map_identity
from suat_mission.navigation.stations import SORTING, STATIONS
from suat_mission.vision.perception import Detection


class SwiftProPort(Port):
    def __init__(self, path, commands):
        super().__init__(path, commands)
        self.pose = [200., 0., 150.]
        self.pump_on = False

    def write(self, wire):
        super().write(wire)
        command = self.commands[-1]
        if command.startswith('G0 '):
            self.pose = [float(field[1:]) for field in command.split()[1:4]]
        elif command.startswith('M2231 V'):
            self.pump_on = command.endswith('1')
        elif command.startswith(('M2221 ', 'M2222 ', 'P2231')):
            prefix = self.reply.split(b'ok', 1)[0]
            if command.startswith('M2221 '):
                fields = ' X%g Y%g Z%g' % tuple(self.pose)
            elif command.startswith('M2222 '):
                fields = ' V1'
            else:
                fields = ' V%d' % self.pump_on
            self.reply = prefix + ('ok' + fields + '\r\n').encode()


class IntegratedMission(StartupFixture):
    port_type = SwiftProPort
    isolated_startup = False
    cake = 'cake-tracked-1'
    frame = 'fixture_camera_optical_frame'
    calibration = 'fixture-dlt'
    zones = ['B', 'C', 'D']

    def write_json(self, name, value):
        path = self.folder / name
        path.write_text(json.dumps(value), encoding='utf-8')
        return str(path)

    def setUp(self):
        super().setUp()
        self.version = 0
        self.active = {}
        self.adjusted = False
        self.hold_visible = self.release_visible = True
        self.depth_visible = True
        self.use_pickup_fallback = False
        self.landing_x = 250.
        self.stop_while_carrying = False
        self.legs = []
        self.declined = []
        self.input_rejections = []
        self.image = np.zeros((120, 300, 3), dtype=np.uint8)
        self.grid = open_grid()
        identity = map_identity(self.grid)
        self.stations = dict(zip(STATIONS, (
            [.4, .4, 0.], [1.5, .5, .2], [1., 1.7, 0.], [1.7, 1., 1.57],
            [2.5, 1.7, 3.14], [1.7, 2.5, -1.57], [.6, .6, .3],
            [2.7, .6, 1.], [2.7, 2.7, 2.], [.6, 2.7, -1.],
        )))
        self.config.update(
            recorded_route_file=self.write_json('route.json', dict(
                frame_id='map', map_identity=identity, waypoints=self.stations)),
            board_geometry_file=self.write_json('board.json', dict(
                map_identity=identity, geometry_id='fixture-board', image_size=[300, 120],
                normalized_size=[300, 120], corners=[[0, 0], [300, 0], [300, 120], [0, 120]],
                zones=self.zones, direction_confirmed=True, direction_reference='fixture')),
            pickup_geometry_file=self.write_json('pickup.yaml', dict(
                calibration_id=self.calibration, arm_bounds_mm=[150, 370, -120, 120, -150, 260],
                reliable_work_area_mm=[160, 285, -110, 110], support_z_mm=-232, cup_offset_mm=82,
                cup_radius_mm=12, empty_travel_pose_mm=[200, 0, 150],
                observation_pose_mm=[190, 45, 150], board_observation_pose_mm=[200, 0, 150],
                transport_pose_mm=[190, -20, 115],
                object_width_mm={'cake': 100}, object_height_mm={'cake': 100},
                placement_mode='visual_red_zone', require_visual_alignment=True,
                reachable_top_selection=True, cake_forward_limit_mm=285, arm_yaw_in_base_rad=0.)),
            sorting_points=['Sorting_W'], use_latest_board_geometry=False,
            navigation_keepout_file='simulated-navigator-keepout.json', max_base_alignments=1,
            navigation_timeout=120., action_timeout=60., observation_timeout=2., feedback_timeout=2.,
            startup_delay=0., stop_margin=1., judge_clock_already_running=False)
        self.node.gate.clock = self.clock.monotonic
        real_mission = self.adapter.Mission
        self.stack.enter_context(patch.object(self.adapter, 'Mission', side_effect=
            lambda *a, **k: real_mission(*a, clock=self.clock.monotonic, **k)))
        self.stack.enter_context(patch.object(self.runtime, 'load_binding', return_value=self.grid))
        self.stack.enter_context(patch.object(self.adapter, 'load_binding', return_value=self.grid))
        self.stub_cameras()

        navigation = self.adapter.Navigator
        self.navigator = Mock(spec=navigation)
        self.navigator.keepout = None
        self.navigator.grid.return_value = self.grid
        self.navigator.pose.return_value = self.stations['Start_A']
        self.navigator.plan.side_effect = self.plan
        self.navigator.execute_leg.side_effect = self.execute_leg
        self.navigator.can_try_next_station.return_value = False
        self.stack.enter_context(patch.object(self.adapter, 'Navigator', return_value=self.navigator))
        self.stack.enter_context(patch.object(self.adapter, 'RosNavigationTransport',
            return_value=SimpleNamespace(settle=lambda deadline: True)))
        serial_scene = self.adapter.SerialRosRobot._serial_scene

        def feed_latest_scene(robot, after, deadline):
            self.send_scene(robot)
            return serial_scene(robot, after, deadline)

        self.stack.enter_context(patch.object(self.adapter.SerialRosRobot, '_serial_scene', feed_latest_scene))

    def stub_cameras(self):
        def stamp():
            seconds = self.clock.monotonic()
            return SimpleNamespace(to_nsec=lambda: round(seconds * 1e9), to_sec=lambda: seconds)

        self.ros.Time = SimpleNamespace(now=stamp)
        self.ros.wait_for_message = lambda *a, **k: SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(), frame_id=self.frame))
        self.ros.ROSException = RuntimeError
        self.ros.logwarn_throttle = Mock()

        def publisher(topic, *args, **kwargs):
            pub = Mock()
            if topic == '/suat/manipulation/active':
                pub.publish.side_effect = lambda msg: self.active.update(json.loads(msg.data))
            return pub

        self.ros.Publisher.side_effect = publisher
        cv = module('cv2', error=RuntimeError, imwrite=lambda *args: True,
                    imread=lambda *_: self.image, warpPerspective=lambda image, *a, **k: image)
        try:
            import cv2
        except ImportError:
            # The test scenes use axis-aligned rectangles. Use signed edge
            # distance for those fixtures when OpenCV is absent on the host.
            def polygon_distance(contour, point, measure):
                low, high = contour.reshape(-1, 2).min(axis=0), contour.reshape(-1, 2).max(axis=0)
                return float(min(*(point-low), *(high-point)))
            cv.pointPolygonTest = polygon_distance
        else:
            cv.pointPolygonTest = cv2.pointPolygonTest
        self.stack.enter_context(patch.dict(sys.modules, {
            'cv2': cv, 'yaml': module('yaml', safe_load=json.loads),
            'cv_bridge': module('cv_bridge', CvBridge=lambda: SimpleNamespace(imgmsg_to_cv2=lambda *_: self.image)),
            'sensor_msgs.msg': module('sensor_msgs.msg', Image=object),
        }))
        self.stack.enter_context(patch('suat_mission.vision.mission_overlay.MissionOverlay'))
        self.stack.enter_context(patch('suat_mission.vision.board_station_view.observe_zone_markers',
            return_value=[{'zone': zone, 'source': 'mock_marker_decoder'} for zone in self.zones]))
        detections = [Detection(label, .95, x, 60., 25., 25.)
                      for label, x in zip(('clock', 'teddybear', 'banana'), (50., 150., 250.))]
        self.stack.enter_context(patch('suat_mission.vision.template_matcher.TemplateMatcher',
            return_value=SimpleNamespace(detect_board=lambda _: {'detections': detections})))

    def send_scene(self, robot):
        self.version += 1
        phase = self.active.get('phase')
        pose = self.ports[-1].pose
        cup = [*pose[:2], pose[2] - 82.]
        top = [270. if self.adjusted else 310., 0., -32.]
        hand = {'state': 'unknown'}
        if phase in ('lift', 'transport', 'carrying', 'observe_placement') and self.hold_visible:
            top = cup
        if phase == 'verify_placement':
            top = [self.landing_x, 0., -132.]
            if self.release_visible:
                hand = dict(state='empty', evidence='synthetic free depth rays', object_id=self.cake)
        manipulation = dict(tool=dict(state='observed', cup_point_arm_mm=cup), gripper=hand)
        if phase == 'observe_placement':
            manipulation['placement'] = dict(zone='A', support_z_mm=-232.,
                polygon_arm_mm=[[150,-110],[350,-110],[350,110],[150,110]],
                candidates_arm_mm=[[250., 0.]])
        surface = dict(top_face=True, calibration_id=self.calibration, arm_xyz_mm=top,
                       normal_arm=[0., 0., 1.], plane_normal_arm=[0., 0., 1.],
                       plane_offset_arm_mm=top[2], support_z_mm=-232., edge_margin_mm=35.)
        if not self.depth_visible:
            surface = {}
            hand.update(state='empty', evidence='synthetic free depth rays', object_id=self.cake)
        payload = dict(scene_id='fixture-scenes', version=self.version, frame_id=self.frame,
                       calibration_id=self.calibration, stamp=self.ros.Time.now().to_sec(), support_frames=3,
                       objects=[dict(label='cake', score=.98, x=150., y=60., width=40., height=40.,
                                     object_id=self.cake, surface=surface, graspable=True)],
                       manipulation=manipulation)
        if self.version == 1:
            for failure in (dict(calibration_id='other-calibration'),
                            dict(stamp=payload['stamp']+30.), dict(stamp=payload['stamp']-30.)):
                robot._instances(Message(json.dumps(dict(payload, **failure))))
                self.input_rejections.append(robot.scene_seq)
        robot._instances(Message(json.dumps(payload)))

    def plan(self, start, goal, grid, **kwargs):
        if self.use_pickup_fallback and goal == self.stations['Sorting_W']:
            self.declined.append(self.node.robot.active_context)
            raise self.adapter.NavigationPlanError('no complete live plan', {'goal': goal})
        return {'timeout_s': 80.}

    def execute_leg(self, item, plan, *, context, **options):
        self.legs.append((item['name'], context, options))
        self.navigator.pose.return_value = item['pose']
        self.send_scene(self.node.robot)
        self.node.robot._ensure_running()
        if item['name'] == 'pickup_align':
            self.adjusted = True
        if item['name'] == 'Collection_A' and self.stop_while_carrying:
            self.command('stop')
        return dict(passed=True, arrival_confirmed=True, target=item, stop_confirmed=True,
                    output_lease_released=True)

    def execute_mission(self):
        self.prepare()
        self.assertEqual(self.input_rejections, [0, 0, 0])
        self.assertNotIn('M17', self.commands)
        self.assertEqual(self.node.config['waypoints'], self.stations)
        self.assertFalse(any(command.startswith(('G0 ', 'M2231 ')) for command in self.commands))
        self.command('start', run_id=self.node.gate.run_id)
        self.join()
        report, final_state = self.recorder.finish.call_args.args
        self.assertEqual(report['run_id'], self.node.gate.run_id)
        self.assertEqual(self.commands.count('M17'), 1)
        move = next(i for i, command in enumerate(self.commands) if command.startswith('G0 '))
        self.assertIn('P2200', self.commands[self.commands.index('M17')+1:move])
        self.assertIn('M2000', self.commands)
        self.assertEqual(len(self.ports), 1)
        return report, final_state

    def test_one_formal_run_uses_same_cake_from_board_to_confirmed_landing(self):
        report, state = self.execute_mission()
        self.assertEqual(report['status'], 'completed', report['error'])
        self.assertEqual(report['board'], dict(clock='B', teddybear='C', banana='D'))
        self.assertEqual(report['board_evidence']['direction_source'], 'station_zone_markers')
        self.assertEqual(report['board_evidence']['run_id'], report['run_id'])
        self.assertEqual([name for name, _, _ in self.legs],
                         ['Classification_area', 'Sorting_W', 'pickup_align', 'Collection_A'])
        self.assertEqual(self.legs[2][1].object_id, self.cake)
        self.assertEqual(self.legs[3][1].object_id, self.cake)
        self.assertEqual([name for name in self.commands if name.startswith('M2231 ')], ['M2231 V1', 'M2231 V0'])
        self.assertEqual(report['delivered'], dict(cake=1, clock=0, teddybear=0, banana=0))
        landed, = report['objects']
        self.assertEqual((landed['object_id'], landed['state'], landed['zone']),
                         (self.cake, 'placed_confirmed', 'A'))
        events = json.loads((self.folder/'manipulation/events.json').read_text(encoding='utf-8'))
        self.assertEqual(next(e for e in events if e['phase']=='grasp_projection')['selected_source'], 'dlt')
        self.assertTrue(any(e['phase']=='visual_alignment' for e in events))
        self.assertTrue(any(e['phase']=='holding_confirmed' for e in events))
        self.assertEqual(sum(e['phase']=='placement_confirmed' for e in events), 1)
        self.assertIsNone(state['serial_manipulation']['held_or_possible_id'])
        self.assertTrue(self.ports[0].closed)

    def test_candidate_station_retry_uses_the_same_navigation_deadline(self):
        self.use_pickup_fallback = True
        self.config['sorting_points'] = SORTING
        report, _ = self.execute_mission()
        self.assertEqual(report['status'], 'completed', report['error'])
        self.assertEqual(report['pickup_station'], 'Sorting_S')
        self.assertEqual(self.declined[0].deadline, self.legs[1][1].deadline)

    def test_missing_top_depth_blocks_pick_before_pump_on(self):
        self.depth_visible = False
        self.config['max_pick_retries'] = 0
        report, _ = self.execute_mission()
        self.assertEqual(report['status'], 'aborted')
        self.assertEqual(report['delivered']['cake'], 0)
        self.assertNotIn('M2231 V1', self.commands)
        self.assertNotIn('Collection_A', [name for name, _, _ in self.legs])

    def test_wrong_map_refuses_preparation_before_serial_acquisition(self):
        self.grid['layout_version'] = 'other-layout'
        self.command('prepare')
        self.join()
        self.assertFalse(self.node.prepared)
        self.assertIn('different complete map identity', self.node.error)
        self.assertEqual(self.node.gate.state, 'stopped')
        self.assertEqual(self.ports, [])

    def test_unknown_lift_preserves_suction_and_never_transports(self):
        self.hold_visible = False
        report, state = self.execute_mission()
        self.assertEqual(report['status'], 'aborted')
        self.assertIn('lift not visually confirmed', report['error'])
        self.assertEqual(report['delivered']['cake'], 0)
        self.assertNotIn('Collection_A', [name for name, _, _ in self.legs])
        self.assertNotIn('M2231 V0', self.commands)
        self.assertTrue(state['serial_manipulation']['pump_may_be_on'])
        self.assertTrue(self.node.robot.recovery_required)
        self.assertFalse(self.ports[0].closed)
        self.assertTrue(self.node.release_for_recovery(None).success)

    def test_wrong_landing_is_not_counted_even_when_pump_is_off(self):
        self.landing_x = 345.
        self.assert_unverified_release()

    def test_in_zone_without_independent_detachment_is_not_counted(self):
        self.release_visible = False
        self.assert_unverified_release()

    def assert_unverified_release(self):
        report, state = self.execute_mission()
        self.assertEqual(report['status'], 'aborted')
        self.assertIn('correct final region not observed', report['error'])
        self.assertEqual(report['delivered']['cake'], 0)
        self.assertEqual(report['objects'][0]['state'], 'held_uncertain')
        self.assertFalse(state['serial_manipulation']['pump_may_be_on'])
        self.assertTrue(self.node.robot.recovery_required)
        self.assertTrue(self.node.release_for_recovery(None).success)

    def test_stop_during_transport_prevents_destination_release(self):
        self.stop_while_carrying = True
        report, state = self.execute_mission()
        self.assertEqual(report['status'], 'aborted')
        self.assertEqual(report['delivered']['cake'], 0)
        self.assertEqual(report['objects'][0]['state'], 'held_uncertain')
        self.navigator.cancel.assert_called()
        self.assertNotIn('M2231 V0', self.commands)
        self.assertTrue(state['serial_manipulation']['pump_may_be_on'])
        self.assertEqual(self.node.gate.state, 'stopped')
        self.assertTrue(self.node.release_for_recovery(None).success)
