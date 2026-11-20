#!/usr/bin/env python3
"""ROS 1 adapter. Motion is locked until configuration and feedback are verified."""
import math
import json
import base64
import io
from dataclasses import asdict
import os
from pathlib import Path
import threading
import time
import uuid
from urllib.parse import quote, urlencode

import actionlib
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult
from swiftpro.msg import position, status
from vision_msgs.msg import Detection2DArray

from suat_mission.manipulation.calibration import PlanarCalibration
from suat_mission.vision.board_recognition import BoardReader
from suat_mission.automation.mission import Mission, MissionConfig, MissionError
from suat_mission.vision.perception import COCO_IDS, Detection, OBJECTS
from suat_mission.contracts import ActionResult, Outcome
from suat_mission.observations import decode_scene, decode_placement
from suat_mission.automation.run_gate import RunGate
from suat_mission.navigation.navigation_ros import Navigator, NavigationPlanError, RosNavigationTransport
from suat_mission.navigation.navigation_gate import goal_is_free
from suat_mission.automation.scope import scope_from_config
from suat_mission.automation.readiness import inspect_config, validate_board_configuration
from suat_mission.automation.run_record import RunRecorder, file_identity
from suat_mission.navigation.navigation_map import load_binding
from suat_mission.automation.resource_binding import validate_waypoint_binding
from suat_mission.navigation.map_identity import map_identity


class RosRobot:
    def __init__(self, permitted=lambda: True, config=None):
        self.config = config
        self.stop_event = threading.Event()
        self.command_lock = threading.RLock()
        self.nav_active = False
        self.permitted = permitted
        self.cv = threading.Condition()
        self.frames = []
        self.frame_seq = 0
        self.last_stamp = None
        self.board_observations = []
        self.observation_seq = 0
        self.feedback = {"arrived": (None, 0, 0.0), "attached": (None, 0, 0.0)}
        self.deadline = float('inf')
        self.holding = False
        self.latest_scene = None
        self.selected_scene = None
        self.scene_seq = 0
        self.placement_pending = None
        self.placement_result = None
        self.active_context = None
        self._load_config()
        direct = self._param('execution_backend') == 'serial_rgbd'
        self.arm = None if direct else rospy.Publisher('/position_write_topic', position, queue_size=1)
        self.pump = None if direct else rospy.Publisher('/pump_topic', status, queue_size=1)
        self._navigator = None
        self.placement_request = rospy.Publisher('/suat/placement/request', String, queue_size=1, latch=False)
        self.subs = [rospy.Subscriber('/objects', Detection2DArray, self._detections, queue_size=1),
                     rospy.Subscriber(self._param('instances_topic','/suat/perception/instances'), String, self._instances, queue_size=1),
                     rospy.Subscriber('/suat/placement/result', String, self._placement, queue_size=10),
                     rospy.Subscriber(self._param('arm_arrived_topic'), Bool, self._feedback, 'arrived', queue_size=10),
                     rospy.Subscriber(self._param('object_attached_topic'), Bool, self._feedback, 'attached', queue_size=10)]

    def _param(self, name, default=None):
        config = getattr(self, 'config', None)
        return config.get(name, default) if config is not None else rospy.get_param('~' + name, default)

    def _load_config(self):
        if self._param('hardware_ready', False) is not True:
            raise ValueError('hardware_ready is false: complete and verify the on-site configuration')
        if self._param('board_slots_confirmed', False) is not True:
            raise ValueError('camera orientation and board slots have not been confirmed')
        if self._param('calibration_verified', False) is not True:
            raise ValueError('calibration has not been verified')
        for gate in ('identity_source_verified', 'feedback_source_verified', 'placement_source_verified',
                     'control_chain_verified'):
            if self._param(gate, False) is not True:
                raise ValueError(gate + ' is false: independent on-site verification required')
        self.calibration_id = self._param('calibration_id', '')
        if not isinstance(self.calibration_id, str) or not self.calibration_id.strip():
            raise ValueError('missing calibration_id')
        zones = self._param('board_slots')
        if len(zones) != 3 or set(zones) != {'B', 'C', 'D'}:
            raise ValueError('invalid board slots')
        if self._param('board_axis', 'x') not in ('x', 'y'):
            raise ValueError('board_axis must be x or y')
        validate_board_configuration(self.config if self.config is not None else rospy.get_param('~', {}))
        self.map_binding = load_binding(Path(__file__).resolve().parents[5])
        self.waypoint_map_identity = validate_waypoint_binding(
            self.config if self.config is not None else rospy.get_param('~', {}), self.map_binding)
        self.scope = scope_from_config({'mission_scope': self._param('mission_scope', 'full'),
                                        'sorting_points': self._param('sorting_points', ['Sorting_N', 'Sorting_W'])})
        self.points = self._param('waypoints', {})
        for name in self.scope.required_points:
            p = self.points.get(name)
            if not isinstance(p, (list, tuple)) or len(p) != 3 or not all(math.isfinite(float(v)) for v in p):
                raise ValueError('missing or invalid waypoint: ' + name)
        paths = self._param('calibration_files', {})
        self.calibrations = {}
        for label in self.scope.labels:
            if not paths.get(label):
                raise ValueError('missing verified calibration file for ' + label)
            with open(os.path.expanduser(paths[label]), 'r') as f:
                self.calibrations[label] = PlanarCalibration.from_values(f.read().split())
        self.bounds = self._bounds('arm_bounds', 4)
        self.z_bounds = self._bounds('arm_z_bounds', 2)
        if self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3] or self.z_bounds[0] >= self.z_bounds[1]:
            raise ValueError('invalid arm bounds')
        self.grasp_z = self._param('grasp_z', {})
        if any(k not in self.grasp_z for k in self.scope.labels):
            raise ValueError('each block class requires a verified grasp height')
        self.approach = float(self._param('approach_offset', 0))
        self.lift = float(self._param('lift_offset', 0))
        if not all(math.isfinite(v) and v > 0 for v in (self.approach, self.lift)):
            raise ValueError('approach and lift offsets must be positive and verified')
        self.drops = self._param('drop_poses', {})
        for zone in self.scope.delivery_zones:
            if zone not in self.drops:
                raise ValueError('missing drop pose for zone ' + zone)
            self._check_pose(self.drops[zone])
        self.nav_timeout = float(self._param('navigation_timeout', 60.0))
        self.feedback_timeout = float(self._param('feedback_timeout', 15.0))
        self.observation_timeout = float(self._param('observation_timeout', 5.0))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.nav_timeout, self.feedback_timeout, self.observation_timeout)):
            raise ValueError('timeouts must be positive')

    def _bounds(self, name, size):
        value = self._param(name, [])
        if len(value) != size or not all(math.isfinite(float(x)) for x in value):
            raise ValueError('invalid parameter ' + name)
        return tuple(float(x) for x in value)

    def _check_pose(self, xyz):
        if len(xyz) != 3 or not all(math.isfinite(float(x)) for x in xyz):
            raise ValueError('invalid arm pose')
        x, y, z = map(float, xyz)
        if not (self.bounds[0] <= x <= self.bounds[1] and self.bounds[2] <= y <= self.bounds[3]
                and self.z_bounds[0] <= z <= self.z_bounds[1]):
            raise ValueError('arm pose outside configured limits')

    def _feedback(self, msg, name):
        with self.cv:
            self.feedback[name] = (bool(msg.data), self.feedback[name][1] + 1, time.monotonic())
            self.cv.notify_all()

    def set_deadline(self, value):
        self.deadline = value

    def _get_navigator(self):
        if self._navigator is None:
            # Use the same canonical workspace as preparation and map recording.
            root = Path(__file__).resolve().parents[5]
            self._navigator = Navigator(root, self._param('navigation_speed', 'slow'),
                                        owner='/suat_mission', health_check=self._ensure_running,
                                        planner_mode=self._param('navigation_planner_mode') or None,
                                        keepout_file=self._param('navigation_keepout_file',
                                            'data/navigation/central_pickup_keepout.json'))
        return self._navigator

    def prepare_navigation(self):
        """Read map/action/gate readiness before Enter, without opening output."""
        navigator = self._get_navigator()
        navigator.grid()
        navigator.connect_control()

    def preflight(self, preparing=False):
        """Require live independent feedback and connected output drivers."""
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            if preparing:
                if (self.stop_event.is_set() or rospy.is_shutdown()
                        or not getattr(self, 'preparation_permitted', lambda: True)()):
                    raise MissionError('preparation was stopped')
            else:
                self._ensure_running()
            with self.cv:
                arrived = self.feedback['arrived']
                attached = self.feedback['attached']
            fresh = lambda item: item[0] is not None and time.monotonic() - item[2] < 2.0
            if (fresh(arrived) and fresh(attached) and arrived[0] is True and attached[0] is False
                    and self.arm.get_num_connections() and self.pump.get_num_connections()):
                return
            time.sleep(0.1)
        raise MissionError('preflight requires live independent arm-arrived and empty-gripper feedback plus connected drivers')

    def _detections(self, msg):
        # Preserve source-clock and ROS connection facts; reception is not a new image stamp.
        stamp = msg.header.stamp.to_nsec()
        received = rospy.Time.now().to_nsec()
        header = getattr(msg, '_connection_header', {}) or {}
        image_topic = self._param('board_image_topic', '')
        image_uri = ('ros-image://' + quote(image_topic, safe='/') + '?' + urlencode(
            {'stamp_ns': stamp, 'frame_id': msg.header.frame_id})) if image_topic else ''
        detected = []
        for obj in msg.detections:
            for hypothesis in obj.results:
                label = COCO_IDS.get(hypothesis.id)
                if label:
                    detected.append(Detection(label, hypothesis.score, obj.bbox.center.x, obj.bbox.center.y,
                                              obj.bbox.size_x, obj.bbox.size_y))
        with self.cv:
            self.observation_seq += 1
            self.board_observations.append((self.observation_seq, dict(
                detections=detected, stamp_ns=stamp, received_ros_ns=received,
                frame_id=msg.header.frame_id, publisher=header.get('callerid'),
                detection_topic=header.get('topic', '/objects'), image_source_uri=image_uri,
                image_source_basis='configured_image_topic_and_original_image_header', image_archived=False,
                declared_detector_source=self._param('board_detector_source', ''),
                declared_score_semantics=self._param('board_score_semantics', ''))))
            self.board_observations = self.board_observations[-100:]
            age = (received-stamp) / 1.e9
            if (stamp > 0 and 0 <= age <= float(self._param('observation_max_age', 2.0))
                    and (self.last_stamp is None or stamp > self.last_stamp)):
                self.last_stamp = stamp
                self.frame_seq += 1
                self.frames.append((self.frame_seq, detected))
                self.frames = self.frames[-100:]
            self.cv.notify_all()

    def _collect(self, count, timeout):
        if count <= 0:
            raise ValueError('frame count must be positive')
        with self.cv:
            after = self.frame_seq
            deadline = time.monotonic() + timeout
            while True:
                self._ensure_running()
                frames = [f for seq, f in self.frames if seq > after]
                if len(frames) >= count:
                    return frames[:count]
                remain = deadline - time.monotonic()
                if remain <= 0:
                    raise MissionError('timed out waiting for fresh detection frames')
                self.cv.wait(min(remain, 0.1))

    def _ensure_running(self, check_attachment=True):
        if self.is_stopped():
            raise MissionError('stop requested')
        if time.monotonic() >= self.deadline:
            raise MissionError('competition deadline reached')
        if self.active_context is not None and time.monotonic() >= self.active_context.deadline:
            raise MissionError('action deadline reached')
        if self.holding and check_attachment:
            with self.cv:
                attached, _, observed_at = self.feedback['attached']
            if attached is not True or time.monotonic() - observed_at > 2.0:
                raise MissionError('object attachment lost or feedback is stale')

    @staticmethod
    def _crop(frame, roi):
        if not roi:
            return frame
        if len(roi) != 4 or roi[0] >= roi[2] or roi[1] >= roi[3]:
            raise ValueError('invalid image ROI')
        return [d for d in frame if roi[0] <= d.x <= roi[2] and roi[1] <= d.y <= roi[3]]

    def read_board_frames(self):
        frames = self._collect(int(self._param('board_frames', 8)), float(self._param('board_timeout', 12.0)))
        roi = self._param('board_roi', [])
        return [self._crop(frame, roi) for frame in frames]

    def read_board_mapping(self, *, run_id):
        """Read this run's board through the geometry/direction/freshness decision seam."""
        config = self.config if self.config is not None else rospy.get_param('~', {})
        count = self._param('board_frames', 8)
        timeout = float(self._param('board_timeout', 12.0))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('board_timeout must be finite and positive')
        with self.cv:
            after = self.observation_seq
            started_ns = rospy.Time.now().to_nsec()
        reader = BoardReader.from_config(run_id, config, started_ns=started_ns)
        transport, interrupted = [], None
        end = time.monotonic() + timeout
        try:
            while len(transport) < count:
                with self.cv:
                    self._ensure_running()
                    frames = [(seq, frame) for seq, frame in self.board_observations if seq > after]
                    if not frames:
                        remaining = end-time.monotonic()
                        if remaining <= 0:
                            break
                        self.cv.wait(min(remaining, .1))
                        continue
                for seq, frame in frames[:count-len(transport)]:
                    after = seq
                    publisher_matches = frame['publisher'] == self._param('board_detector_publisher')
                    reader.observe(frame['detections'], run_id=run_id, stamp_ns=frame['stamp_ns'],
                        now_ns=rospy.Time.now().to_nsec(), frame_id=frame['frame_id'],
                        image_id=frame['image_source_uri'],
                        source=frame['declared_detector_source'] if publisher_matches else 'unverified_publisher',
                        score_semantics=frame['declared_score_semantics'])
                    transport.append(dict({key: value for key, value in frame.items() if key != 'detections'},
                                          publisher_matches=publisher_matches))
        except MissionError as exc:
            interrupted = exc
        result = reader.result(now_ns=rospy.Time.now().to_nsec())
        result.evidence.update(transport_frames=transport,
            collection_complete=len(transport) == count,
            collection_interruption=str(interrupted) if interrupted else None,
            original_image_archive='SOURCE_URI_ONLY_NO_ORIGINAL_IMAGE_PIXELS_SAVED')
        sink = getattr(self, 'board_evidence_sink', None)
        if sink is not None:
            sink(asdict(result))
        if interrupted is not None:
            raise interrupted
        return result

    def _instances(self, msg):
        try:
            scene = decode_scene(msg.data, time.monotonic(), rospy.Time.now().to_sec(),
                                 float(self._param('observation_max_age', 2.0)), self.calibration_id,
                                 int(self._param('target_min_frames', 3)))
            with self.cv:
                if (self.latest_scene is not None and scene.scene_id == self.latest_scene.scene_id
                        and (scene.version <= self.latest_scene.version or
                             scene.source_stamp <= self.latest_scene.source_stamp)):
                    return
                self.latest_scene = scene
                self.scene_seq += 1
                self.cv.notify_all()
        except (ValueError, TypeError, KeyError) as exc:
            rospy.logwarn_throttle(5.0, 'Rejected instance observation: %s', exc)

    def detect_objects(self):
        # Class/proximity tracks from /objects cannot stand in for physical identity.
        with self.cv:
            after = self.scene_seq
            end = time.monotonic() + self.observation_timeout
            while time.monotonic() < end:
                self._ensure_running()
                if self.scene_seq > after:
                    self.selected_scene = self.latest_scene
                    return self.latest_scene
                self.cv.wait(min(.1, max(0.0, end - time.monotonic())))
        raise MissionError('no fresh physical-instance snapshot; identity producer is not implemented/verified')

    def _placement(self, msg):
        with self.cv:
            if self.placement_pending is None or self.placement_result is not None:
                return
            context, zone = self.placement_pending
            try:
                self.placement_result = decode_placement(msg.data, context, zone)
                self.cv.notify_all()
            except (ValueError, TypeError, KeyError):
                return

    def _output(self, function, *args):
        # Stop and command emission share a lock: an old worker cannot emit after stop returns.
        with self.command_lock:
            self._ensure_running()
            function(*args)

    def _execute(self, context, action, function):
        if context.action != action:
            raise MissionError('incorrect action context')
        self.active_context = context
        try:
            self._ensure_running()
            result = function()
            self._ensure_running()
            if isinstance(result, ActionResult):
                return result
            return ActionResult(context, Outcome.CONFIRMED if result else Outcome.UNKNOWN,
                                'adapter: ' + action + ' independent feedback transition verified'
                                if result else 'adapter completion was not verified')
        finally:
            self.active_context = None

    def _wait_feedback(self, name, expected, after, timeout, require_transition=False):
        deadline = time.monotonic() + timeout
        saw_opposite = not require_transition
        with self.cv:
            while time.monotonic() < deadline:
                self._ensure_running(check_attachment=not (name == 'attached' and expected is False))
                value, seq, _ = self.feedback[name]
                if seq > after:
                    if value != expected:
                        saw_opposite = True
                    elif saw_opposite:
                        return True
                self.cv.wait(min(0.1, max(0.0, deadline - time.monotonic())))
        return False

    def _move_arm(self, xyz):
        self._ensure_running()
        self._check_pose(xyz)
        with self.cv:
            seq = self.feedback['arrived'][1]
        self._output(self.arm.publish, position(*map(float, xyz)))
        # An arrival topic must show movement (false) and then fresh arrival (true).
        if not self._wait_feedback('arrived', True, seq, self.feedback_timeout, True):
            raise MissionError('arm motion was not independently verified')

    def pick(self, target, label, context):
        if not target.object_id or target.object_id != context.object_id:
            raise MissionError('pick target identity mismatch')
        scene = self.selected_scene
        if (scene is None or scene.version != context.scene_version or target not in scene.detections
                or not 0 <= time.monotonic() - scene.observed_at <=
                float(self._param('observation_max_age', 2.0))):
            raise MissionError('pick target scene is missing, mismatched or stale')
        return self._execute(context, 'pick', lambda: self._pick(target, label))

    def _pick(self, target, label):
        if self.holding:
            raise MissionError('already holding a block')
        with self.cv:
            attached, _, observed_at = self.feedback['attached']
        if attached is not False or time.monotonic() - observed_at > 2.0:
            raise MissionError('fresh independent no-object feedback required before grasp')
        x, y = self.calibrations[label].project(target.x, target.y, self.bounds)
        z = float(self.grasp_z[label])
        for pose in ((x, y, z + self.approach), (x, y, z), (x, y, z + self.lift)):
            self._check_pose(pose)
        self._move_arm((x, y, z + self.approach))
        self._move_arm((x, y, z))
        with self.cv:
            seq = self.feedback['attached'][1]
        self._ensure_running()
        self._output(self.pump.publish, status(1))
        if not self._wait_feedback('attached', True, seq, self.feedback_timeout):
            raise MissionError('suction was not independently verified')
        self.holding = True
        self._move_arm((x, y, z + self.lift))
        self._ensure_running()
        return True

    def drop(self, zone, label, context):
        return self._execute(context, 'place', lambda: self._drop(zone, label, context))

    def _drop(self, zone, label, context):
        if not self.holding:
            raise MissionError('no verified object to release')
        self._move_arm(self.drops[zone])
        with self.cv:
            seq = self.feedback['attached'][1]
        self._ensure_running()
        self._output(self.pump.publish, status(0))
        if not self._wait_feedback('attached', False, seq, self.feedback_timeout):
            raise MissionError('release was not independently verified')
        self.holding = False
        # Empty suction alone does not show where the block landed.
        with self.cv:
            self.placement_pending = (context, zone)
            self.placement_result = None
        request = dict(asdict(context), zone=zone)
        try:
            self._output(self.placement_request.publish, String(data=json.dumps(request)))
            with self.cv:
                end = min(context.deadline, time.monotonic() + self.observation_timeout)
                while time.monotonic() < end:
                    self._ensure_running()
                    if self.placement_result is not None:
                        return self.placement_result
                    self.cv.wait(min(.1, max(0.0, end - time.monotonic())))
            return ActionResult(context, Outcome.UNKNOWN, 'release seen, but final placement was not observed')
        finally:
            with self.cv:
                self.placement_pending = None

    def navigate(self, name, remaining, context):
        return self._execute(context, 'navigate', lambda: self._navigate(name, remaining))

    def _before_navigation(self, name):
        pass

    def _navigate(self, name, remaining):
        from suat_mission.navigation.pickup_keepout import is_pickup_station
        from suat_mission.navigation.navigation_budget import NavigationBudget
        started = time.monotonic()
        budget = NavigationBudget(self.nav_timeout)
        self._ensure_running()
        navigator = self._get_navigator()
        item = {'name': name, 'pose': list(self.points[name])}
        candidate = (name == 'Classification_area' or name in getattr(self, 'board_candidates', {})
                     or name in self.scope.sorting_points)
        def make_plan(previous_attempts=None):
            actual_pose = None
            try:
                actual_pose = navigator.pose()
                return navigator.plan(actual_pose, item['pose'], navigator.grid(),
                                      pickup_station=is_pickup_station(navigator.keepout,name))
            except NavigationPlanError as exc:
                report={'passed':False,'phase':'planning','target':item,'first_failure_stage':'planning',
                        'actual_pose':actual_pose,'leg_elapsed_s':time.monotonic()-started,
                        'error':str(exc),'rejected_plan':exc.plan,
                        'motion_goal_sent':False,'output_lease_opened':False}
                if previous_attempts:
                    report['previous_execution_attempts']=previous_attempts
                self.last_navigation_result=report
                sink=getattr(self,'navigation_evidence_sink',None)
                if sink is not None:sink(report)
                raise
        try:
            plan = make_plan()
        except NavigationPlanError as exc:
            if candidate and str(exc) in NavigationPlanError.STATION_ERRORS:
                return ActionResult(self.active_context, Outcome.UNREACHABLE,
                                    'no motion issued; candidate station plan refused: '+str(exc))
            raise MissionError('navigate '+name+': '+str(exc)) from exc
        self._before_navigation(name)
        plan['timeout_s'] = min(plan['timeout_s'], self.nav_timeout, float(remaining))
        with self.command_lock:
            self._ensure_running()
            self.nav_active = True
        try:
            report = navigator.execute_leg(item, plan, context=self.active_context,station_approach=True,budget=budget)
            attempts=[report]
            # Only an idle, released navigation failure may replan once. A stop,
            # stale sensor, ownership fault or uncertain payload never retries.
            retryable=(report.get('action_terminal_state')==4 or
                       report.get('error')=='no navigation progress for 20 seconds')
            if retryable and navigator.can_try_next_station(report,self.active_context):
                self._ensure_running()
                if time.monotonic()+navigator.executor_stop_budget()<self.active_context.deadline:
                    try:
                        plan=make_plan(attempts)
                    except NavigationPlanError as exc:
                        if not candidate or str(exc) not in NavigationPlanError.STATION_ERRORS:
                            raise
                        report=dict(report,error=str(exc),rejected_plan=exc.plan,
                                    candidate_unreachable=True)
                    else:
                        plan['timeout_s']=min(plan['timeout_s'],self.nav_timeout,
                                              self.active_context.deadline-time.monotonic())
                        report=navigator.execute_leg(item,plan,context=self.active_context,station_approach=True,
                                                     budget=budget)
                    attempts.append(dict(report))
                    report['attempts']=attempts
            report['action_elapsed_s'] = time.monotonic()-started
            if attempts[0].get('first_failure_stage'):
                report['first_failure_stage']=attempts[0]['first_failure_stage']
            self.last_navigation_result = report
            sink = getattr(self, 'navigation_evidence_sink', None)
            if sink is not None:
                sink(report)
            if candidate and navigator.can_try_next_station(report,self.active_context):
                self._ensure_running()
                navigator.connect_control()
                return ActionResult(self.active_context, Outcome.UNREACHABLE, json.dumps(report))
            outcome = Outcome.CONFIRMED if report['passed'] else Outcome.FAILED
            return ActionResult(self.active_context, outcome, json.dumps(report, ensure_ascii=True))
        finally:
            with self.command_lock:
                self.nav_active = False

    def is_stopped(self):
        return (self.stop_event.is_set() or not self.permitted() or rospy.is_shutdown()
                or time.monotonic() >= self.deadline)

    def stop_motion(self):
        with self.command_lock:
            self.stop_event.set()
            navigator = getattr(self, '_navigator', None)
            if navigator is not None:
                navigator.cancel()
            self.nav_active = False
        # No second /cmd_vel publisher; the shared owner stops its own output.
        # Do not command the pump or arm blindly if a block might be held.

    @staticmethod
    def log(message):
        rospy.loginfo('%s', message)

    def close(self):
        for sub in self.subs:
            sub.unregister()
        if self._navigator is not None:
            self._navigator.close()
        for pub in (self.arm, self.pump, self.placement_request):
            if pub is not None:
                pub.unregister()


class SerialRosRobot(RosRobot):
    """The same competition protocol, backed by measured serial and RGB-D scenes."""
    def _load_config(self):
        from suat_mission.automation.serial_mission import SerialMissionRuntime
        from suat_mission.manipulation.pickup_geometry import PickupGeometry
        root=Path(__file__).resolve().parents[5]
        geometry=PickupGeometry.load(self._param('pickup_geometry_file'))
        self.calibration_id=geometry.calibration_id
        self.scope=scope_from_config(self.config)
        self.points=dict(self._param('waypoints',{}))
        self.board_candidates = {'Classification_area__'+item['id']: item
                                 for item in self.config.get('board_station_candidates', ())}
        self.points.update((name, item['pose']) for name, item in self.board_candidates.items())
        self.bounds=geometry.bounds[:4] if geometry.bounds else None
        self.z_bounds=geometry.bounds[4:] if geometry.bounds else None
        self.map_binding=load_binding(root)
        self.nav_timeout=float(self._param('navigation_timeout',60.))
        self.feedback_timeout=float(self._param('feedback_timeout',15.))
        self.observation_timeout=float(self._param('observation_timeout',5.))
        self.serial_runtime=SerialMissionRuntime(root,self.config,self._serial_scene,
            lambda: RosRobot._ensure_running(self,check_attachment=False),self._align_base)
        self._empty_travel_prepared = False

    def _serial_scene(self, after, deadline):
        with self.cv:
            while time.monotonic()<deadline:
                if self.run_token is None:
                    if self.stop_event.is_set() or not self.preparation_permitted():
                        raise MissionError('preparation interrupted')
                else:
                    RosRobot._ensure_running(self,check_attachment=False)
                if self.latest_scene is not None and self.latest_scene.version>after:
                    age=time.monotonic()-self.latest_scene.observed_at
                    if 0<=age<=float(self._param('observation_max_age',2.)):
                        return self.latest_scene
                self.cv.wait(min(.1,max(0.,deadline-time.monotonic())))
        raise TimeoutError('no fresh physical-instance observation from the camera adapter')

    def preflight(self, preparing=False):
        if preparing:
            self._require_preparing()
        else:
            self._ensure_running(check_attachment=False)
        self.serial_runtime.open(preparing=preparing,
                                 check_preparing=self._require_preparing if preparing else None,
                                 write_if_running=getattr(self, 'arm_write_if_running', None))
        after=-1 if self.latest_scene is None else self.latest_scene.version
        self._serial_scene(after,time.monotonic()+self.observation_timeout)

    def _require_preparing(self):
        if rospy.is_shutdown() or self.stop_event.is_set() or not self.preparation_permitted():
            raise MissionError('preparation was stopped')

    def prepare_observation(self, kind):
        self._require_stopped_base()
        deadline = min(self.deadline, time.monotonic()+float(self._param('action_timeout',60.)))
        if kind=='objects':
            self.serial_runtime.bootstrap(deadline)
        self.serial_runtime.observation(kind,deadline)

    def pickup_scenes(self):
        """Allow the same side-view tracker used by the stationary entry to settle."""
        end = min(self.deadline, time.monotonic()+self.observation_timeout)
        received = False
        while time.monotonic() < end:
            try:
                scene = self.serial_runtime.fresh_scene(end)
            except TimeoutError:
                if not received:
                    raise
                return
            self.selected_scene = scene
            received = True
            yield scene

    def set_deadline(self,value):
        super().set_deadline(value)
        self.config['_deadline']=value

    def board_stations(self):
        self._ensure_running()
        if not self.board_candidates:
            return ['Classification_area']
        navigator = self._get_navigator()
        grid = navigator.grid()
        radius = max(.25, navigator.radius)
        names = []
        for name, item in self.board_candidates.items():
            clear = goal_is_free(grid, *item['pose'][:2], radius=radius)
            self.serial_runtime.event('board_station_candidate', station=name, pose=item['pose'],
                view_evidence=item['view_evidence'], clearance_radius_m=radius, saved_map_clear=clear)
            if clear:
                names.append(name)
        return names

    def read_board_mapping(self, *, run_id, station):
        result=self.serial_runtime.read_board(run_id, station)
        sink=getattr(self,'board_evidence_sink',None)
        if sink is not None:sink(result.evidence)
        return result

    def _ensure_running(self, check_attachment=True):
        RosRobot._ensure_running(self,check_attachment=False)
        if check_attachment and self.holding and self.nav_active:
            self.serial_runtime.check_carrying(self.latest_scene)

    def _require_stopped_base(self):
        self._ensure_running()
        deadline = min(self.deadline, time.monotonic()+3.)
        if self.active_context is not None:
            deadline = min(deadline, self.active_context.deadline)
        if not RosNavigationTransport(self._get_navigator()).settle(deadline):
            raise MissionError('base stop and released navigation required before observation or manipulation')
        self._ensure_running()

    def _align_base(self, dx_mm, dy_mm, parent, index):
        from suat_mission.contracts import ActionContext
        self._require_stopped_base()
        if self.serial_runtime.held is not None or self.serial_runtime.session.pump_may_be_on:
            raise MissionError('pickup adjustment requires an empty travel pose and pump off')
        navigator=self._get_navigator()
        start=navigator.pose()
        mount=self.serial_runtime.geometry.number('arm_yaw_in_base_rad')
        turn=start[2]+mount;c,s=math.cos(turn),math.sin(turn)
        goal=[start[0]+(c*dx_mm-s*dy_mm)/1000.,start[1]+(s*dx_mm+c*dy_mm)/1000.,start[2]]
        child=ActionContext(parent.run_id,parent.goal_id+':base:'+str(index),'navigate',
            parent.object_id,parent.scene_version,parent.deadline)
        grid=navigator.grid()
        precise=navigator.keepout is not None and navigator.keepout.precise_pickup
        try:
            plan=navigator.plan(start,goal,grid,pickup_approach=precise)
        except NavigationPlanError as exc:
            if not precise and str(exc) not in NavigationPlanError.STATION_ERRORS:
                raise
            return {'passed':False,'pickup_plan_refused':True,'motion_goal_sent':False,
                    'error':str(exc),'rejected_plan':exc.plan}
        plan['timeout_s']=min(plan['timeout_s'],max(0.,parent.deadline-time.monotonic()))
        report=navigator.execute_leg({'name':'pickup_align','pose':goal},plan,context=child,
            phase='align',xy_tolerance=float(self._param('base_alignment_tolerance_m',.01)),yaw_tolerance=.05)
        if report['passed']:
            self._require_stopped_base()
        return report

    def _pick(self, target, label):
        self._require_stopped_base()
        result=self.serial_runtime.pick(target,label,self.active_context)
        self.holding=self.serial_runtime.held is not None
        return result

    def navigate(self, name, remaining, context):
        self.last_confirmed_station = None
        result = super().navigate(name, remaining, context)
        if result.outcome == Outcome.CONFIRMED:
            self.last_confirmed_station = (name, result.context.run_id, result.context.object_id)
        return result

    def _drop(self, zone, label, context):
        self._require_stopped_base()
        arrived = getattr(self, 'last_confirmed_station', None)
        if arrived != ('Collection_' + zone, context.run_id, context.object_id):
            return ActionResult(context, Outcome.UNKNOWN, 'current payload has no confirmed destination arrival')
        result=self.serial_runtime.drop(zone,label,context)
        self.holding=self.serial_runtime.held is not None
        return result

    def empty_confirmed(self):
        return self.serial_runtime.confirm_empty(self.latest_scene, self.deadline)

    def _before_navigation(self, name):
        runtime = self.serial_runtime
        if runtime.held is None:
            self._require_stopped_base()
            runtime.open()
            deadline = min(self.active_context.deadline,self.deadline)
            if not self._empty_travel_prepared:
                runtime.prepare_empty_observation(
                    self._param('arm_unfold_pose_mm', (191.74, -0.30, 124.09)), deadline,
                    fold_feed_limit=float(self._param('arm_fold_feed_mm_min', 1000.)))
                self._empty_travel_prepared = True
            travel = runtime.geometry.values['empty_travel_pose_mm']
            if math.dist(runtime.arm_pose,travel) > runtime.session.correction_tolerance_mm:
                runtime.move('empty_travel',travel,deadline)
        runtime.event('navigate',station=name)

    def delivery_cost(self, target, zone):
        x,y,_=self._get_navigator().pose();gx,gy,_=self.points['Collection_'+zone]
        return math.hypot(gx-x,gy-y)

    def stop_motion(self):
        try:
            RosRobot.stop_motion(self)
        finally:
            runtime=getattr(self,'serial_runtime',None)
            if runtime is not None and runtime.session is not None:
                runtime.session.stop(time.monotonic()+3.)

    def close(self):
        status=self.serial_runtime.close()
        self.recovery_required=bool(self.serial_runtime.held is not None
                                    or status.get('pump_may_be_on') or status.get('ownership_retained'))
        if not self.recovery_required:
            RosRobot.close(self)
        return status


class Node:
    def __init__(self):
        self.robot = None
        self.thread = None
        self.lock = threading.RLock()
        self.gate = RunGate()
        self.preparing = self.prepared = False
        self.config = {}
        self.recorder = None
        self.record_subs = []
        self.error = None
        self.scope = None
        self.preparation_id = None
        self.stop_requested_at = None
        self.state_pub = rospy.Publisher('/suat/mission/state', String, queue_size=1, latch=True)
        self.report_pub = rospy.Publisher('/suat/mission/report', String, queue_size=1, latch=True)
        self.start_sub = rospy.Subscriber('/suat/mission/command', String, self.command, queue_size=1)
        self.stop_sub = rospy.Subscriber('/suat/mission/stop', Bool, self.stop, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        from std_srvs.srv import Trigger
        self.recovery_service=rospy.Service('~release_for_recovery',Trigger,self.release_for_recovery)
        rospy.loginfo('Prepare once, then start the scoped autonomous run with one Enter.')
        self.publish_state()
        self.state_timer=rospy.Timer(rospy.Duration(1.),lambda event:self.publish_state())

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(dict(
            state=self.gate.state, run_id=self.gate.run_id, worker_id=self.gate.active,
            prepared=self.prepared, preparing=self.preparing, scope=self.scope, error=self.error,
            preparation_id=self.preparation_id,
            recovery_required=bool(getattr(self.robot,'recovery_required',False)),
            execution_backend=self.config.get('execution_backend'),
            core_delivery_id=self.config.get('core_delivery_id',rospy.get_param('~core_delivery_id','')),
            phase=(getattr(getattr(self.robot,'serial_runtime',None),'events',[]) or [{}])[-1].get('phase'),
            deadline=self.gate.deadline, budget_anchor_monotonic=self.gate.budget_anchor,
            run_directory=str(self.recorder.path) if self.recorder else None))))

    def record(self, kind, details=None):
        try:
            if self.recorder and not self.recorder.finished:
                self.recorder.event(kind, details)
        except Exception as exc:
            self.record_failed(exc)
            raise MissionError('automatic recording failed: ' + str(exc)) from exc

    def record_failed(self, exc):
        with self.lock:
            self.error = 'automatic recording failed: ' + str(exc)
            self.prepared = False
            self.gate.stop()
            robot = self.robot
        rospy.logerr('%s', self.error)
        if robot and not self.preparing:
            robot.stop_motion()
        self.publish_state()

    def _record_topics(self):
        """Use the same concrete message class as each in-process subscriber.

        rospy shares a subscriber implementation per resolved topic. AnyMsg here
        can make a String callback receive AnyMsg and lose navigation feedback.
        """
        limit = float(self.config.get('record_max_hz', 10.0))
        size_limit = self.config.get('record_max_bytes', 1048576)
        if (not math.isfinite(limit) or limit <= 0 or type(size_limit) is not int or size_limit <= 0):
            raise ValueError('invalid recording rate or message byte limit')
        last = {}
        record = self.recorder
        def receive(msg, topic):
            try:
                if record is not self.recorder or record.finished:
                    return
                now = time.monotonic()
                if now - last.get(topic, -float('inf')) < 1.0 / limit:
                    return
                last[topic] = now
                stream=io.BytesIO()
                msg.serialize(stream)
                wire=stream.getvalue()
                if len(wire) > size_limit:
                    record.event('record_message_omitted', {'topic': topic, 'bytes': len(wire),
                                                            'reason': 'configured byte limit'})
                    return
                header = getattr(msg, '_connection_header', {}) or {}
                record.topic(topic, {'type': getattr(msg,'_type',header.get('type')),
                                     'base64': base64.b64encode(wire).decode('ascii')})
            except Exception as exc:
                if record.finished or record is not self.recorder:
                    return
                self.record_failed(exc)
        types={name:String for name in (
            '/suat/mission/command','/suat/mission/state','/suat/mission/report',
            '/suat/perception/instances','/suat/perception/grasp_instances',
            '/suat/perception/grasp_status','/suat/manipulation/active',
            '/suat/navigation/state','/suat/placement/request','/suat/placement/result')}
        types.update({'/suat/mission/stop':Bool,'/move_base/result':MoveBaseActionResult,
                      '/cmd_vel':Twist,'/objects':Detection2DArray})
        types[rospy.resolve_name(self.config.get('instances_topic','/suat/perception/instances'))]=String
        for key in ('arm_arrived_topic','object_attached_topic'):
            if self.config.get(key):types[rospy.resolve_name(self.config[key])]=Bool
        for topic in record.topics:
            message_type=types.get(rospy.resolve_name(topic))
            if message_type is None:
                record.event('record_topic_omitted',{'topic':topic,'reason':'message type not registered'})
                rospy.logwarn('Recorder skipped %s: message type not registered',topic)
                continue
            self.record_subs.append(rospy.Subscriber(topic,message_type,receive,topic,queue_size=10))

    def _close_record_topics(self):
        for sub in self.record_subs:
            sub.unregister()
        self.record_subs = []

    def _make_recorder(self):
        root=Path(__file__).resolve().parents[5]
        config=self.config;selected=scope_from_config(config)
        source=Path(__file__).resolve().parents[1]
        paths=[Path(__file__).resolve(),source/'automation/mission.py',source/'automation/serial_mission.py',
               source/'manipulation/arm_session.py', source/'vision/mission_scene_node.py',
               source/'vision/experimental_vision.py',source/'navigation/navigation_ros.py']
        metadata={name:config.get(name) for name in ('operator','layout_id','calibration_id','execution_backend')}
        metadata.update(entrypoint=str(Path(__file__).resolve()),speed_profile=config['navigation_speed'],
            code_files={str(p):file_identity(p) for p in paths},
            config=dict(config),map_identity=map_identity(self.map_binding),
            resource_files={name:file_identity(config.get(name)) for name in
                ('source_config_file','pickup_geometry_file','recorded_route_file','board_geometry_file',
                 'board_reference_geometry_file','board_station_records_file')},
            robot_wall_time_raw=rospy.Time.now().to_sec(),
            recording_message_policy='typed_ros_messages_fix03',
            recording_policy='selected topic samples and source paths; no digest computation')
        topics=['/suat/mission/command','/suat/mission/stop',
                config.get('instances_topic','/suat/perception/instances'),'/suat/perception/grasp_status',
                '/suat/manipulation/active','/suat/navigation/state','/move_base/result','/cmd_vel']
        self.recorder=RunRecorder(config.get('run_root',str(root/'reports/runs')),
            self.gate.run_id,selected.name,config.get('record_topics',topics),metadata)
        self._record_topics()

    def prepare(self, scope, entered_at=None):
        robot = None
        try:
            config = dict(rospy.get_param('~', {}))
            config.setdefault('navigation_speed', 'slow')
            if scope is not None:
                config['mission_scope'] = scope
            if config.get('execution_backend')=='serial_rgbd':
                from suat_mission.automation.serial_mission import resolve_serial_config
                config=resolve_serial_config(config,Path(__file__).resolve().parents[5])
            selected = scope_from_config(config)
            self.map_binding = load_binding(Path(__file__).resolve().parents[5])
            diagnostics = inspect_config(config, binding=self.map_binding)
            if not diagnostics['configuration_complete']:
                raise ValueError(json.dumps(diagnostics['issues'], ensure_ascii=False))
            self.config, self.scope = config, selected.name
            self.gate.prepare(new_run=True, deadline_seconds=config.get('mission_timeout', 600.0),
                              startup_delay=config.get('startup_delay', 0.0),
                              stop_margin=config.get('stop_margin', 0.0),
                              clock_already_running=config.get('judge_clock_already_running', False),
                              entered_at=entered_at)
            self._make_recorder()
            config['run_directory']=str(self.recorder.path)
            robot_type=SerialRosRobot if config.get('execution_backend')=='serial_rgbd' else RosRobot
            robot = robot_type(permitted=lambda: self.gate.permitted(robot.run_token), config=config)
            robot.run_token = None
            robot.arm_write_if_running = lambda write: self.gate.write_if_running(robot.run_token, write)
            robot.board_evidence_sink = lambda decision: self.record('board_decision', decision)
            robot.navigation_evidence_sink = lambda result: self.record('navigation', result)
            robot.preparation_permitted = lambda: self.preparing and self.gate.state == 'idle' and not self.error
            with self.lock:
                self.robot = robot
            robot.preflight(preparing=True)
            if not robot.preparation_permitted():
                raise MissionError('preparation was stopped')
            robot.prepare_navigation()
            with self.lock:
                if self.gate.state != 'idle' or self.error:
                    raise MissionError('preparation was stopped')
                self.record('ready', {'scope': self.scope})
                self.prepared = True
        except Exception as exc:
            with self.lock:
                self.error = 'preparation failed: ' + str(exc)
                self.prepared = False
                self.gate.stop()
            rospy.logerr('%s', self.error)
            self._close_robot(robot)
            self._close_record_topics()
            if self.recorder and not self.recorder.finished:
                self._finish_record({'run_id': self.gate.run_id, 'status': 'preparation_failed',
                                     'error': self.error}, {})
        finally:
            with self.lock:
                self.preparing = False
                self.publish_state()

    def _close_robot(self, robot):
        if robot is None:
            return
        try:
            robot.close()
        except Exception as exc:
            robot.recovery_required = True
            self.error = (self.error + '; ' if self.error else '') + 'arm cleanup failed: ' + str(exc)
            rospy.logerr('%s', self.error)
        self.robot = robot if getattr(robot, 'recovery_required', False) else None

    def command(self, msg):
        try:
            request = json.loads(msg.data) if msg.data.strip().startswith('{') else {'command': msg.data.strip().lower()}
            value = request['command']
        except (TypeError, ValueError, KeyError):
            rospy.logwarn('Invalid mission command')
            return
        if value in ('prepare', 'start') and (getattr(msg, '_connection_header', {}) or {}).get('latching') == '1':
            rospy.logwarn('Latched start refused; send a fresh explicit start command')
            return
        if value == 'stop':
            self.stop(Bool(data=True))
            return
        if value == 'reset':
            with self.lock:
                try:
                    if self.thread and self.thread.is_alive():
                        raise ValueError('worker is still active')
                    if self.preparing:
                        raise ValueError('preparation is still active')
                    if getattr(self.robot,'recovery_required',False):
                        raise ValueError('possible payload retained; use explicit release_for_recovery after supporting it')
                    if self.prepared:
                        self._close_record_topics()
                        self.prepared = False
                        self._close_robot(self.robot)
                        self._finish_record({'run_id': self.gate.run_id, 'status': 'cancelled_before_start',
                                             'error': 'reset before start'}, {})
                        if self.robot is not None:
                            self.gate.stop()
                            raise ValueError('serial ownership is retained for recovery')
                    self.gate.reset()
                    self.error = None
                    self.publish_state()
                except ValueError as exc:
                    rospy.logwarn('Reset refused: %s', exc)
            return
        if value == 'prepare':
            with self.lock:
                if (self.preparing or self.prepared or self.robot is not None or self.gate.active is not None
                        or self.gate.state != 'idle' or (self.thread and self.thread.is_alive())):
                    rospy.logwarn('Preparation refused: reset after the previous run first')
                    return
                self.error = None
                self.recorder = None
                self.stop_requested_at = None
                self.preparing = True
                self.preparation_id = request.get('request_id') or uuid.uuid4().hex
                self.thread = threading.Thread(target=self.prepare,
                    args=(request.get('scope'), request.get('entered_at_monotonic')), daemon=True)
                try:
                    self.thread.start()
                except Exception as exc:
                    self.preparing = False
                    self.gate.stop()
                    self.error = 'could not start preparation worker: ' + str(exc)
                self.publish_state()
            return
        if value != 'start':
            rospy.logwarn('Ignoring unknown task command: %s', value)
            return
        with self.lock:
            if not self.prepared or self.preparing or self.gate.active is not None:
                rospy.logwarn('Start refused: a completed preparation is required')
                return
            if request.get('run_id', self.gate.run_id) != self.gate.run_id:
                rospy.logwarn('Start refused: prepared run ID mismatch')
                return
            token = None
            recorded = False
            try:
                token = self.gate.start()
                self.recorder.start(self.gate.started_at, self.gate.deadline,
                                    {'competition_limit_s': self.gate.deadline_seconds,
                                     'startup_delay_s': self.gate.startup_delay,
                                     'stop_margin_s': self.gate.stop_margin,
                                     'judge_clock_already_running': self.gate.clock_already_running,
                                     'budget_anchor_monotonic': self.gate.budget_anchor})
                recorded = True
                self.prepared = False
                self.robot.run_token = token
                self.thread = threading.Thread(target=self.run, args=(token,), daemon=True)
                self.thread.start()
            except Exception as exc:
                self.prepared = False
                self.gate.stop()
                if token is not None:
                    self.gate.finish(token)
                self.error = 'start failed: ' + str(exc)
                if self.robot is not None:
                    self.robot.run_token = None
                self._close_record_topics()
                self._close_robot(self.robot)
                status = 'start_refused' if token is None else (
                    'startup_failed_before_execution' if recorded else 'recording_failed_before_execution')
                report = {'run_id': self.gate.run_id, 'status': status, 'error': self.error}
                self._finish_record(report, {})
                self.report_pub.publish(String(data=json.dumps(report)))
                self.publish_state()
                rospy.logwarn('Start refused: %s', exc)
                return
            self.publish_state()

    def _finish_record(self, report, final_state):
        try:
            return self.recorder.finish(report, final_state)
        except Exception as exc:
            self.error = 'automatic result recording failed: ' + str(exc)
            rospy.logerr('%s', self.error)
            report['recording_error'] = self.error
            return None

    def run(self, token):
        robot = self.robot
        report = {'run_id': self.gate.run_id, 'status': 'aborted', 'error': 'worker did not finish'}
        final_state = {}
        try:
            robot.set_deadline(self.gate.deadline)
            robot.preflight()
            config = self.config
            cfg = MissionConfig(zones=config.get('board_slots', ['B', 'C', 'D']),
                                sorting_points=config.get('sorting_points', ['Sorting_W']),
                                mission_scope=self.scope,
                                board_votes=int(config.get('board_votes', 3)),
                                board_threshold=float(config.get('board_min_confidence', .6)),
                                board_axis=config.get('board_axis', 'x'), board_reverse=config.get('board_reverse', False),
                                target_threshold=float(config.get('target_min_confidence', .6)),
                                target_thresholds=config.get('target_thresholds', {}),
                                target_roi=config.get('target_roi') or None, each_count=int(config.get('each_count', 3)),
                                action_timeout=float(config.get('action_timeout', 60)),
                                navigation_timeout=float(config.get('navigation_timeout', 60)),
                                observation_max_age=float(config.get('observation_max_age', 2)),
                                deadline_seconds=self.gate.deadline - self.gate.started_at,
                                max_pick_retries=int(config.get('max_pick_retries',1)),
                                finish_margin_seconds=0., # RunGate already reserves stop_margin.
                                initial_cycle_budget_seconds=float(config.get('initial_cycle_budget_seconds',120.)),
                                allow_cake_degradation=bool(config.get('allow_cake_degradation',True)))
            report = asdict(Mission(robot, cfg, run_id=self.gate.run_id, started_at=self.gate.started_at,
                                    deadline=self.gate.deadline,
                                    event_sink=lambda text: self.record('mission', {'message': text})).run())
        except Exception as exc:
            self.gate.stop()
            rospy.logerr('Mission preflight failed: %s', exc)
            report['error'] = str(exc)
        finally:
            try:
                if robot is not None:
                    try:
                        try:
                            self.record('stop_command_requested')
                        finally:
                            robot.stop_motion()
                        final_state['stop_command_returned_at'] = time.monotonic()
                        final_state['attached_feedback'] = dict(zip(('value', 'sequence', 'observed_at'),
                                                                    robot.feedback['attached']))
                        runtime=getattr(robot,'serial_runtime',None)
                        if runtime is not None:
                            final_state['serial_manipulation']={'held_or_possible_id':runtime.held,
                                'last_measured_arm_pose_mm':runtime.arm_pose,
                                'pump_may_be_on':bool(runtime.session and runtime.session.pump_may_be_on)}
                    finally:
                        robot.run_token = None
                        self._close_robot(robot)
            except Exception as exc:
                report['stop_or_recording_error'] = str(exc)
                self.error = str(exc)
            finally:
                try:
                    self._close_record_topics()
                    final_state['stop_requested_at'] = self.stop_requested_at
                    self._finish_record(report, final_state)
                    report['run_directory'] = str(self.recorder.path)
                    if self.error:
                        report['recording_or_stop_error'] = self.error
                    self.report_pub.publish(String(data=json.dumps(report)))
                    rospy.logwarn('Mission %s; delivered=%s; error=%s',
                                  report['status'], report.get('delivered'), report.get('error'))
                finally:
                    with self.lock:
                        self.gate.finish(token)
                        self.publish_state()

    def stop(self, msg):
        if msg.data:
            with self.lock:
                self.gate.stop()
                self.prepared = False
                self.stop_requested_at = time.monotonic()
                robot = self.robot
                if self.preparing:
                    # Only the Prepare worker owns that lifecycle and can close
                    # a port while its initialization is unwinding.
                    if robot is not None:
                        robot.stop_event.set()
                    self.publish_state()
                    return
                worker_active = self.gate.active is not None
                if not worker_active:
                    self._close_robot(robot)
                    if self.recorder and not self.recorder.finished:
                        self._close_record_topics()
                        report = {'run_id': self.gate.run_id, 'status': 'cancelled_before_start',
                                  'error': 'stop before start'}
                        self._finish_record(report, {'stop_requested_at': self.stop_requested_at})
                        self.report_pub.publish(String(data=json.dumps(report)))
                    self.publish_state()
                    return
            if self.recorder and not self.recorder.finished:
                try:
                    if self.gate.started_at is not None:
                        self.recorder.intervention('operator_stop')
                    else:
                        self.record('stop_before_start')
                except Exception as exc:
                    rospy.logerr('Stop recording failed: %s', exc)
            if robot:
                try:
                    robot.stop_motion()
                except Exception as exc:
                    self.error = 'stop request failed: ' + str(exc)
                    rospy.logerr('%s', self.error)
            self.publish_state()

    def release_for_recovery(self, request):
        """Explicit operator recovery after an aborted possible-held state, never a mission step."""
        from std_srvs.srv import TriggerResponse
        with self.lock:
            if self.gate.active is not None or self.preparing or self.prepared:
                return TriggerResponse(False,'mission worker is still active')
            robot=self.robot
            if not getattr(robot,'recovery_required',False):
                return TriggerResponse(False,'no retained serial session needs release')
            try:
                runtime=getattr(robot,'serial_runtime',None)
                if runtime is not None:
                    if runtime.session is not None and runtime.session.pump_may_be_on:
                        runtime.session.pump(False,deadline=time.monotonic()+8.,recovery=True)
                    runtime.held=None
                self._close_robot(robot)
                if self.robot is not None:
                    return TriggerResponse(False,'serial ownership remains retained')
                self.publish_state()
                return TriggerResponse(True,'pump off; retained session closed; reset before preparation')
            except Exception as exc:
                return TriggerResponse(False,str(exc))

    def shutdown(self):
        self.stop(Bool(data=True))
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(5.0)


if __name__ == '__main__':
    rospy.init_node('suat_mission')
    Node()
    rospy.spin()
