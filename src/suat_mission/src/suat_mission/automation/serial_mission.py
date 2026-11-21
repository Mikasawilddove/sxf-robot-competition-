"""Measured serial manipulation connected to the competition scene stream."""
import json
import math
from pathlib import Path
import time

from suat_mission.contracts import ActionResult, Outcome
from suat_mission.manipulation.arm_path import FEED_FAST
from suat_mission.manipulation.pickup_geometry import PickupGeometry, GeometryUnavailable
from suat_mission.navigation.map_identity import validate_map_identity
from suat_mission.navigation.navigation_map import load_binding
from suat_mission.navigation.board_stations import board_station_candidates
from suat_mission.navigation.stations import mission_stations


def resource(root, value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else Path(root)/path


def resolve_serial_config(config, root):
    """Resolve recorded resources; do not manufacture missing B/C/D stations."""
    result = dict(config)
    geometry_path = resource(root, result.get('pickup_geometry_file', 'data/calibration/pickup_geometry_active.yaml'))
    geometry = PickupGeometry.load(geometry_path)
    result['pickup_geometry_file'] = str(geometry_path)
    result['calibration_id'] = geometry.calibration_id
    filename = resource(root, result.get('recorded_route_file', 'data/navigation/recorded_stations_20261115.json'))
    route = json.loads(filename.read_text(encoding='utf-8'))
    binding=load_binding(root)
    validate_map_identity(route,binding)
    result['recorded_route_file']=str(filename)
    result['waypoint_map_identity']=route['map_identity']
    stations = mission_stations(root, result, binding)
    points = {name: entry['pose'] for name, entry in stations.items()}
    if result.get('pickup_stations_file'):
        result['pickup_stations_file'] = str(resource(root,result['pickup_stations_file']))
    result['waypoints'] = points
    if result.get('board_station_records_file'):
        station_file = resource(root, result['board_station_records_file'])
        records = json.loads(station_file.read_text(encoding='utf-8'))
        result['board_station_records_file'] = str(station_file)
        result['board_station_candidates'] = board_station_candidates(points['Classification_area'], binding, records)
    board = resource(root, result.get('board_geometry_file',
        'data/calibration/board_view_candidates/classification_arrival_20261115_03_v1.json'))
    current_board=resource(root,'data/calibration/field_20261117/board_view_classification_arrival_20261117_v1.json')
    if result.get('use_latest_board_geometry',True) and current_board.exists():
        reference_data = json.loads(board.read_text(encoding='utf-8'))
        validate_map_identity(reference_data,binding)
        result['board_reference_geometry_file'] = str(board)
        board=current_board
    result['board_geometry_file'] = str(board)
    board_data = json.loads(board.read_text(encoding='utf-8'))
    validate_map_identity(board_data,binding)
    result.update(board_slots=board_data['zones'], board_corners=board_data['corners'],
                  board_slots_confirmed=board_data.get('direction_confirmed') is True)
    return result


class SerialMissionRuntime:
    def __init__(self, root, config, next_scene, ensure_running):
        self.root, self.config = Path(root), config
        self.confirm_frames = config.get('manipulation_confirm_frames', 2)
        if type(self.confirm_frames) is not int or self.confirm_frames < 1:
            raise ValueError('manipulation_confirm_frames must be a positive integer')
        self.geometry = PickupGeometry.load(config['pickup_geometry_file'])
        self.next_scene, self.ensure = next_scene, ensure_running
        self.run = Path(config['run_directory'])/'manipulation'
        self.run.mkdir(parents=True,exist_ok=True)
        self.session = None
        self.version = 0
        self.held = None
        self.held_label = ''
        self.selected_target = None
        self.arm_pose = None
        self.last_hold_seen = 0.
        self.slots = {}
        self.events = []
        self.active_pub = None
        self.observed_after = 0.
        self.scene_id = None
        self.last_scene = None
        self.bootstrapped=False
        self.zone=None
        self.grasp_camera_offset = (0., 0., 0.)

    def event(self, phase, **values):
        self.events.append(dict(phase=phase, at=time.monotonic(), **values))
        (self.run/'events.json').write_text(json.dumps(self.events,ensure_ascii=False,indent=2),encoding='utf-8')

    def active(self, phase, target=None):
        import rospy
        from std_msgs.msg import String
        if self.active_pub is None:
            self.active_pub=rospy.Publisher('/suat/manipulation/active',String,queue_size=1,latch=True)
        target=target or self.selected_target
        payload={
            'phase':phase,'object_id':getattr(target,'object_id',self.held or ''),
            'label':getattr(target,'label',self.held_label),
            'calibration_id':self.geometry.calibration_id,'stamp':rospy.Time.now().to_sec(),
            'zone':self.zone,
            'arm_pose_mm':self.arm_pose,'cup_offset_mm':self.geometry.cup_offset}
        if phase == 'observe_objects':
            payload['center_ordinary_top'] = True
        if self.held and phase in ('lift', 'hold_view', 'transport', 'carrying', 'observe_placement'):
            cup = [v-d for v,d in zip(self.arm_pose,self.grasp_camera_offset)]
            cup[2] -= self.geometry.cup_offset
            payload['payload_cup_arm_mm'] = cup
        self.active_pub.publish(String(data=json.dumps(payload)))

    def open(self, *, preparing=False, check_preparing=None, write_if_running=None):
        if self.session is not None:
            if not preparing and not self.session.initialized:
                self.session.enable_motion()
                self.arm_pose=self.session.read_pose(time.monotonic()+3.)['xyz']
                self.event('serial_initialized', measured_pose_mm=self.arm_pose)
            return
        from suat_mission.manipulation.arm_session import ArmSession
        self.session=ArmSession(self.run,feed=FEED_FAST)
        self.session.permitted=lambda: self._permitted()
        self.session.write_if_running=write_if_running
        self.session.open(preparing=preparing,check_preparing=check_preparing)
        if check_preparing is not None:
            check_preparing()
        self.arm_pose=self.session.read_pose(time.monotonic()+3.)['xyz']
        self.event('serial_open', measured_pose_mm=self.arm_pose)

    def _permitted(self):
        try:
            self.ensure()
            return True
        except Exception:
            return False

    def move(self, phase, xyz, deadline, loaded=False, *, target_prevalidated=False):
        self.ensure()
        pickup = phase in ('approach', 'contact', 'lift', 'static_return_above', 'static_return_contact')
        if not target_prevalidated:
            label = getattr(self.selected_target, 'label', None) if pickup else None
            self.geometry.check_pose(xyz, pickup_label=label)
        self.event(phase,status='moving',target_mm=xyz)
        measurement=self.session.go(phase,xyz,loaded=loaded,deadline=deadline,
                                    target_prevalidated=target_prevalidated)
        self.arm_pose=measurement['measurements'][-1]['xyz']
        self.observed_after=time.monotonic()
        self.event(phase, target_mm=xyz, measured_pose_mm=self.arm_pose, measurement=measurement)
        self.active(phase)

    def prepare_empty_observation(self, unfolded_pose, deadline, *, left_view=None):
        self.open()
        self.ensure()
        if self.held is not None or self.session.pump_may_be_on:
            raise RuntimeError('workpose recovery requires an empty arm with the pump off')
        home = self.geometry.check_pose(self.geometry.values['empty_travel_pose_mm'])
        side = self.geometry.check_pose(
            left_view if left_view is not None else self.geometry.values['observation_pose_mm'])
        tolerance = self.session.correction_tolerance_mm
        start = self.arm_pose
        if (math.dist(start, side) <= tolerance
                or (left_view is None and math.dist(start, home) <= tolerance)):
            return

        # The front fold is below the normal grasp workspace. All motion
        # targets still pass the existing workspace and controller IK checks.
        front = self.geometry.bounds[0] if self.geometry.bounds is not None else 0.
        if 0. < start[0] < front and math.hypot(*start[:2]) < front and start[2] < unfolded_pose[2]:
            self.move('unfold_empty_arm', unfolded_pose, deadline)
            if left_view is None:
                self.move('center_workpose', home, deadline)
        else:
            self.geometry.check_pose(start)
            if start[2] < unfolded_pose[2] - tolerance:
                clearance = [start[0], start[1], unfolded_pose[2]]
                self.move('raise_empty_arm', clearance, deadline)
        if left_view is not None:
            self.move('pickup_camera_left', side, deadline)

    def observe_objects(self):
        if self.held is not None:
            raise RuntimeError('cannot use an empty observation pose while carrying')
        self.selected_target = None
        self.observed_after = time.monotonic()
        self.active('observe_objects')

    def fresh_scene(self,deadline):
        while time.monotonic()<deadline:
            self.ensure();scene=self.next_scene(self.version,deadline);self.version=scene.version
            if self.scene_id is not None and self.scene_id!=scene.scene_id:
                raise RuntimeError('instance producer restarted during the mission')
            self.scene_id=scene.scene_id
            if scene.observed_at<self.observed_after:continue
            self.last_scene=scene
            return scene
        raise TimeoutError('fresh synchronized scene unavailable')

    def bootstrap(self,deadline):
        if self.bootstrapped:return
        self.open()
        if self.geometry.values.get('refine_translation_at_start'):
            for index,pose in enumerate(self.geometry.values['calibration_view_poses_mm']):
                self.move('calibration_view_'+str(index+1),pose,deadline)
                seen=False;until=min(deadline,time.monotonic()+8.)
                while time.monotonic()<until:
                    scene=self.fresh_scene(until)
                    calibration=scene.manipulation.get('calibration',{})
                    if calibration.get('samples',0)>=index+1:
                        seen=True;break
                if not seen:raise GeometryUnavailable('orange calibration reference not observed at pose '+str(index+1))
            if self.last_scene.manipulation.get('calibration',{}).get('status')!='translation_refined':
                raise GeometryUnavailable('measured camera/arm reference pairs did not agree')
            self.event('camera_translation_refined',observation=self.last_scene.manipulation['calibration'])
        else:
            self.event('camera_calibration_loaded', calibration_id=self.geometry.calibration_id,
                       source_file=self.geometry.values.get('source_file'))
        self.bootstrapped=True

    @staticmethod
    def observed_cup(scene):
        tool=scene.manipulation.get('tool',{})
        cup=tool.get('cup_point_arm_mm')
        if tool.get('state')=='observed' and cup and len(cup)==3 and all(math.isfinite(v) for v in cup):
            return cup
        return None

    def matches_observed_cup(self,scene,point):
        cup=self.observed_cup(scene)
        tolerance=float(self.geometry.values.get('holding_position_tolerance_mm',25.))
        return cup is not None and math.dist(point,cup)<=tolerance

    def confirm_empty(self, scene, deadline):
        if self.held is not None or self.session is None or self.session.pump_may_be_on:
            return False
        until = min(deadline, time.monotonic() + float(self.config.get('observation_timeout', 5.)))
        while time.monotonic() < until:
            self.ensure()
            if scene is not None and (self.scene_id is None or scene.scene_id == self.scene_id):
                age = time.monotonic() - scene.observed_at
                hand = scene.manipulation.get('gripper', {})
                if (scene.observed_at >= self.observed_after
                        and 0 <= age <= float(self.config.get('observation_max_age', 2.))
                        and hand.get('state') == 'empty' and hand.get('evidence')):
                    return True
            try:
                scene = self.fresh_scene(until)
            except TimeoutError:
                break
        return False

    def unreachable_pick(self, context, reason):
        empty = self.confirm_empty(self.last_scene, context.deadline)
        return ActionResult(context, Outcome.UNREACHABLE if empty else Outcome.UNKNOWN, str(reason))

    @staticmethod
    def observed_target(scene, object_id, label):
        return next((d for d in scene.detections if d.object_id == object_id and d.label == label), None)

    @staticmethod
    def observed_empty_for(scene, object_id):
        hand = scene.manipulation.get('gripper', {})
        return hand.get('state') == 'empty' and bool(hand.get('evidence')) and hand.get('object_id') == object_id

    def payload_at_cup(self, scene, point):
        return (self.matches_observed_cup(scene,point)
                and not self.observed_empty_for(scene,self.held))

    @staticmethod
    def raised_payload(scene, object_id):
        raised = scene.manipulation.get('raised_payload') or {}
        point = raised.get('point_arm_mm')
        if (raised.get('object_id') == object_id and isinstance(point, (list, tuple))
                and len(point) == 3 and all(math.isfinite(v) for v in point)):
            return raised
        return None

    def _refresh_hold(self, scene):
        """Return hold evidence from this frame, preserving state through occlusion."""
        if (self.held is None or scene is None or scene.scene_id != self.scene_id
                or scene.observed_at < self.observed_after
                or scene.observed_at < self.last_hold_seen):
            return False
        if self.observed_empty_for(scene, self.held):
            raise RuntimeError('gripper visibly empty while carrying; preserve suction and stop travel')
        target = self.observed_target(scene, self.held, self.held_label)
        supported = self.raised_payload(scene, self.held) is not None
        if not supported and target is not None:
            try:
                supported = self.payload_at_cup(scene, self.geometry.payload_point(target))
            except GeometryUnavailable:
                pass
        if supported:
            self.last_hold_seen = max(self.last_hold_seen, scene.observed_at)
        return supported

    def _wait_for_holding(self, deadline):
        until = min(deadline, time.monotonic()+float(self.config.get('feedback_timeout',15.)))
        while not self._refresh_hold(self.fresh_scene(until)):
            pass

    def target_on_support(self, scene, object_id, point, original_top):
        tolerance = float(self.geometry.values.get('holding_position_tolerance_mm',25.))
        if math.dist(point,original_top)>tolerance:
            return False
        if self.observed_empty_for(scene,object_id):
            return True
        if not self.config.get('stationary_ordinary_test'):
            return False
        cup = self.observed_cup(scene)
        hand = scene.manipulation.get('gripper', {})
        # The held object cannot simultaneously be at the original support
        # and separated from an observed cup with visible free space below it.
        return (cup is not None and cup[2]-point[2]>tolerance
                and hand.get('object_id')==object_id and hand.get('occupied_rays')==0
                and hand.get('free_rays',0)>=int(self.geometry.values.get('gripper_min_rays',30)))

    def _confirm_pick(self,target,top_before,lift,deadline):
        held_votes=empty_votes=0
        needed=self.confirm_frames
        while time.monotonic()<deadline:
            scene=self.fresh_scene(deadline)
            raised = self.raised_payload(scene, target.object_id)
            if raised and not self.observed_empty_for(scene, target.object_id):
                point = raised['point_arm_mm']
                if point[2] >= top_before[2] + lift*.5:
                    held_votes += 1
                    empty_votes = 0
                    if held_votes >= needed:
                        self.event('holding_depth_evidence', object_id=target.object_id,
                                   contact_top_mm=point, support_pixels=raised['support_pixels'])
                        return True, point
                    continue
            current=self.observed_target(scene,target.object_id,target.label)
            if current is None:
                held_votes=empty_votes=0
                continue
            try:point=self.geometry.payload_point(current)
            except GeometryUnavailable:
                held_votes=empty_votes=0;continue
            held=(point[2]>=top_before[2]+lift*.5 and self.matches_observed_cup(scene,point))
            hand_empty=self.observed_empty_for(scene,target.object_id)
            empty=self.target_on_support(scene,target.object_id,point,top_before)
            held_votes=held_votes+1 if held and not hand_empty else 0
            empty_votes=empty_votes+1 if empty and not held else 0
            if held_votes>=needed:return True,point
            if empty_votes>=needed:return False,point
        raise TimeoutError('neither held nor positively empty observed')

    def _wait_target(self, object_id, deadline, predicate, *, point_of=None, stable_mm=None):
        votes=0;required=self.confirm_frames
        point_of = point_of or self.geometry.payload_point
        anchor=None
        while time.monotonic()<deadline:
            scene=self.fresh_scene(deadline)
            target=self.observed_target(scene,object_id,self.held_label)
            if target is None:
                votes=0;anchor=None
                continue
            try:
                point=point_of(target)
            except GeometryUnavailable:
                votes=0;anchor=None;continue
            if not predicate(target,point):
                votes=0;anchor=None
                continue
            if stable_mm is not None and (anchor is None or math.dist(point,anchor)>stable_mm):
                votes=0;anchor=point
            votes+=1
            if votes>=required:
                return target,point
        raise TimeoutError('fresh visual manipulation evidence did not converge')

    def pick(self, target, label, context, *, scene, return_to_support=False):
        self.open()
        self.ensure()
        if self.held is not None or self.session.pump_may_be_on:
            return ActionResult(context, Outcome.UNKNOWN, 'hand is not established empty before a new pick')
        if (scene.version != context.scene_version or target not in scene.detections
                or target.object_id != context.object_id or target.label != label
                or (self.scene_id is not None and scene.scene_id != self.scene_id)):
            return ActionResult(context, Outcome.UNKNOWN, 'pickup target does not match the stopped camera frame')
        self.scene_id, self.last_scene, self.version = scene.scene_id, scene, scene.version
        if target.graspable is not True or target.occluded_by:
            return ActionResult(context, Outcome.UNREACHABLE, 'no unobstructed top in the stopped camera frame')

        approach = float(self.geometry.values.get('approach_offset_mm', 40.))
        lift = float(self.geometry.values.get('lift_offset_mm', 50.))
        source = self.geometry.grasp_source(label)
        forward = float(self.config.get('stationary_forward_correction_mm', 0.)) if source == 'dlt' else 0.
        if not math.isfinite(approach) or approach <= 0. or not math.isfinite(lift) or lift <= 0.:
            return ActionResult(context, Outcome.UNREACHABLE, 'invalid approach or lift height')
        correction = (0., 0., 0.)
        cup = self.observed_cup(scene)
        if cup is not None and source == 'dlt':
            reference = (*cup[:2], cup[2]+self.geometry.cup_offset)
            offset = [measured-observed for measured,observed in zip(self.arm_pose,reference)]
            if math.hypot(*offset) <= float(self.geometry.values.get('max_visual_correction_mm', 30.)):
                correction = offset

        # All candidate points come from the same stopped RGB-D frame.
        for option in self.geometry.pickup_options(target):
            try:
                projection = self.geometry.grasp_projection(option)
                if math.hypot(*self.geometry.base_adjustment(option, projection=projection)) > 1.:
                    continue
                nominal = self.geometry.contact_target(option, projection=projection)
                adjusted = [v+d for v,d in zip(nominal,correction)]
                adjusted[0] += forward
                contact = self.geometry.check_pose(adjusted, pickup_label=label)
                hover = self.geometry.check_pose([*contact[:2],contact[2]+approach], pickup_label=label)
                lift_target = self.geometry.check_pose([*contact[:2],contact[2]+lift], pickup_label=label)
            except GeometryUnavailable as exc:
                self.event('pickup_candidate_rejected', object_id=option.object_id, reason=str(exc))
                continue
            for phase, pose in (('approach', hover), ('contact', contact), ('lift', lift_target)):
                if not self.session.validate_target(pose, context.deadline):
                    self.event('pickup_candidate_rejected', object_id=option.object_id,
                               reason='controller rejected target', stage=phase, target_mm=pose)
                    break
            else:
                break
        else:
            return ActionResult(context, Outcome.UNREACHABLE, 'no grasp point within the arm workspace in the stopped frame')

        top_before = self.geometry.point(target)
        target = self.selected_target = option
        self.grasp_camera_offset = correction
        self.event('first_frame_grasp_plan', object_id=target.object_id, scene_version=scene.version,
                   source_stamp=scene.source_stamp, selected_source=source,
                   grasp_pixel=target.surface.get('grasp_pixel'),
                   top_arm_mm=self.geometry.point(target), support_arm_z_mm=target.surface.get('support_z_mm'),
                   nominal_contact_mm=nominal, reference_offset_mm=correction,
                   official_model=projection[1].get('official_model'),
                   stationary_forward_correction_mm=forward,
                   hover_mm=hover, contact_mm=contact, lift_mm=lift_target)
        self.active('pick', target)
        self.move('approach', hover, context.deadline, target_prevalidated=True)
        self.session.pump(True, deadline=context.deadline, wait_for_current=False)
        self.move('contact', contact, context.deadline, target_prevalidated=True)
        lift_from = list(self.arm_pose)
        self.held=target.object_id  # possible holding until image evidence settles it
        self.held_label=label
        # Keep the lift vertical at the measured contact XY.
        measured_lift = [*lift_from[:2], lift_from[2]+lift]
        self.move('lift', measured_lift, context.deadline, loaded=True,
                  target_prevalidated=measured_lift == lift_target)
        hold_pose = self.geometry.values.get('hold_observation_pose_mm')
        if hold_pose:
            self.move('hold_view',hold_pose,context.deadline,loaded=True)
        feedback_timeout = float(self.config.get('feedback_timeout',15.))
        try:
            held,point=self._confirm_pick(target,top_before,lift,min(context.deadline,
                time.monotonic()+feedback_timeout))
        except TimeoutError:
            return ActionResult(context,Outcome.UNKNOWN,'pump may hold an object; lift not visually confirmed')
        if held is False:
            self.session.pump(False,deadline=context.deadline)
            self.held=None;self.held_label=''
            self.observe_objects()
            return ActionResult(context,Outcome.NOT_ACQUIRED,'selected target stayed on support; gripper volume visibly empty')
        if held:
            self.last_hold_seen = self.last_scene.observed_at
        if return_to_support:
            self.event('holding_confirmed',object_id=target.object_id,observed_top_mm=point)
            if hold_pose:
                self.move('static_return_above',lift_target,context.deadline,loaded=True)
            self.move('static_return_contact',lift_from,context.deadline,loaded=True)
            self.session.pump(False,deadline=context.deadline)
            self.move('verify_placement',lift_target,context.deadline)
            self._wait_target(target.object_id,context.deadline,lambda d,p:
                self.target_on_support(self.last_scene,target.object_id,p,top_before),stable_mm=5.)
            self.event('returned_to_support',object_id=target.object_id)
            self.held=None;self.held_label=''
            self.observe_objects()
            return ActionResult(context,Outcome.CONFIRMED,'measured lift, visual hold and return to original support confirmed')
        self.move('transport',self.geometry.values['transport_pose_mm'],context.deadline,loaded=True)
        self.active('carrying',target)
        self.event('holding_confirmed',object_id=target.object_id,observed_top_mm=point)
        return ActionResult(context,Outcome.CONFIRMED,'fresh top motion matches measured cup lift: '+target.object_id)

    def check_carrying(self, scene):
        if self.held is None:
            return
        camera_at = max(self.last_hold_seen, scene.observed_at if scene is not None
                        and scene.scene_id == self.scene_id else 0.)
        if time.monotonic()-camera_at > float(self.config.get('holding_observation_timeout',2.)):
            raise RuntimeError('RGB-D stream expired while carrying; preserve suction and stop travel')
        self._refresh_hold(scene)

    def drop(self, zone, label, context):
        if self.held!=context.object_id or self.held_label!=label:
            return ActionResult(context,Outcome.UNKNOWN,'placement does not match the held object')
        slot=self.slots.get(zone,0)
        region=None;polygon=None;above=None
        landing_kind='collection'
        self.zone=zone
        if self.geometry.values.get('placement_mode')=='visual_red_zone':
            self.observed_after=time.monotonic();self.active('observe_placement')
            until=min(context.deadline,time.monotonic()+8.);placement=None
            while time.monotonic()<until:
                try:
                    scene = self.fresh_scene(until)
                except TimeoutError:
                    break
                holding = self._refresh_hold(scene)
                candidate = scene.manipulation.get('placement')
                if holding and candidate and candidate.get('zone') == zone and candidate.get('candidates_arm_mm'):
                    placement = candidate
                    break
            if self.last_hold_seen < self.observed_after:
                return ActionResult(context, Outcome.UNKNOWN, 'no fresh holding evidence at the release station')
            if placement is None:
                if zone=='A' or not self.config.get('allow_transport_zone_fallback',False):
                    return ActionResult(context,Outcome.UNKNOWN,'no observed reachable free floor in the destination red zone')
                try:release,region,support=self.geometry.transport_release(zone,label,slot)
                except GeometryUnavailable as exc:return ActionResult(context,Outcome.UNKNOWN,str(exc))
                landing_kind='transport'
                self.event('transport_fallback_plan',zone=zone,release_mm=release,recorded_region=region)
            else:
                polygon=placement.get('polygon_arm_mm')
                if (not isinstance(polygon, (list, tuple)) or len(polygon)<3
                        or any(not isinstance(p, (list, tuple)) or len(p)!=2
                               or not all(type(v) in (int, float) and math.isfinite(v) for v in p)
                               for p in polygon)):
                    return ActionResult(context,Outcome.UNKNOWN,'no valid observed destination polygon')
                support=float(placement['support_z_mm'])
                height=float(self.geometry.values.get('object_height_mm',{}).get(label,100.))
                z=support+height+float(self.geometry.values.get('release_bottom_clearance_mm',100.))+self.geometry.cup_offset
                release=None
                for x,y,*_ in placement['candidates_arm_mm']:
                    candidate=[x,y,z]
                    approach=[x,y,max(z+40.,self.arm_pose[2])]
                    if (self.session.validate_target(candidate,context.deadline) and
                            self.session.validate_target(approach,context.deadline)):
                        release,above=candidate,approach
                        break
                if release is None:
                    if zone=='A' or not self.config.get('allow_transport_zone_fallback',False):
                        return ActionResult(context,Outcome.UNKNOWN,'observed red zone is outside the arm reach')
                    try:release,region,support=self.geometry.transport_release(zone,label,slot)
                    except GeometryUnavailable as exc:return ActionResult(context,Outcome.UNKNOWN,str(exc))
                    landing_kind='transport'
                    polygon=None
                    self.event('transport_fallback_plan',zone=zone,release_mm=release,recorded_region=region)
                else:
                    region=[min(p[0] for p in polygon),max(p[0] for p in polygon),min(p[1] for p in polygon),max(p[1] for p in polygon)]
                    self.event('visual_release_plan',zone=zone,release_mm=release,observed_region=placement)
        else:
            self.observed_after = time.monotonic()
            self._wait_for_holding(context.deadline)
            release=self.geometry.release(zone,label,slot)
            support=float(self.geometry.values.get('drop_support_z_mm',{}).get(zone,self.geometry.support_z))
        region=region or self.geometry.values.get('drop_regions_arm_mm',{}).get(zone)
        if (not isinstance(region, (list, tuple)) or len(region)!=4
                or not all(type(v) in (int, float) and math.isfinite(v) for v in region)
                or region[0]>=region[1] or region[2]>=region[3] or not math.isfinite(support)):
            return ActionResult(context,Outcome.UNKNOWN,'no measured placement region for '+zone)
        if above is None:
            above=[release[0],release[1],max(release[2]+40.,self.arm_pose[2])]
            if not all(self.session.validate_target(point,context.deadline) for point in (release,above)):
                return ActionResult(context,Outcome.UNKNOWN,'destination is outside the arm reach')
        self.active('place')
        self.move('place_approach',above,context.deadline,loaded=True,target_prevalidated=True)
        self.move('release_height',release,context.deadline,loaded=True,target_prevalidated=True)
        self.session.pump(False,deadline=context.deadline)
        self.move('withdraw',above,context.deadline,target_prevalidated=True)
        self.active('verify_placement')
        height=float(self.geometry.values.get('object_height_mm',{}).get(label,100.))
        tolerance=float(self.geometry.values.get('height_tolerance_mm',12.))
        landing_margin=math.sqrt(2)*float(self.geometry.values.get('object_width_mm',{}).get(label,100.))/2.
        def inside(p):
            if polygon is None:
                margin=landing_margin if landing_kind=='transport' else 0.
                return region[0]+margin<=p[0]<=region[1]-margin and region[2]+margin<=p[1]<=region[3]-margin
            import cv2
            import numpy as np
            return cv2.pointPolygonTest(np.asarray(polygon,np.float32).reshape(-1,1,2),
                (float(p[0]),float(p[1])),True)>=landing_margin
        try:
            _,point=self._wait_target(context.object_id,min(context.deadline,time.monotonic()+
                float(self.config.get('feedback_timeout',15.))),lambda d,p:
                inside(p)
                and abs(p[2]-(support+height))<=tolerance
                and self.observed_empty_for(self.last_scene,context.object_id),
                point_of=lambda d:self.geometry.point(d,require_fit=False),stable_mm=tolerance)
        except TimeoutError:
            return ActionResult(context,Outcome.UNKNOWN,'release commanded; correct final region not observed')
        self.held=None;self.held_label='';self.selected_target=None
        self.zone=None
        self.slots[zone]=slot+1
        self.active('empty')
        self.event('placement_confirmed',object_id=context.object_id,zone=zone,slot=slot,point_mm=point,
                   landing_kind=landing_kind)
        return ActionResult(context,Outcome.CONFIRMED,'fresh stable released top inside '+zone+' '+landing_kind+' slot '+str(slot))

    def read_board(self, run_id, station):
        import cv2
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from suat_mission.vision.board_recognition import BoardDecision
        from suat_mission.vision.board_station_view import station_views, visible_board_view
        from suat_mission.vision.mission_overlay import MissionOverlay, draw_board
        from suat_mission.vision.perception import OBJECTS
        from suat_mission.vision.template_matcher import TemplateMatcher

        geometry_file, _, views = station_views(self.config, station)
        start = rospy.Time.now().to_nsec()
        deadline = min(time.monotonic() + float(self.config.get('board_timeout', 12.)),
                       float(self.config.get('_deadline', float('inf'))))
        topic = self.config.get('board_image_topic', '/camera/color/image_raw')
        bridge = CvBridge()
        overlay = MissionOverlay('board', bridge, draw_board)
        directory = self.run / 'board' / station
        directory.mkdir(parents=True, exist_ok=True)
        zones = ('B', 'C', 'D')
        templates = {label: [cv2.imread(str(self.root/'src/suat_mission/assets/templates'/(label+'.png')))]
                     for label in OBJECTS}
        matcher = TemplateMatcher(templates, scales=[.4,.5,.6,.7,.8,.9,1.,1.1,1.2],
                                  threshold=float(self.config.get('board_min_confidence', .6)), max_peaks=8)
        evidence = dict(run_id=run_id, station=station, geometry_file=str(geometry_file),
                        direction_source='fixed_reference', slot_order=list(zones))
        last_view = None
        mapping = {}
        while time.monotonic() < deadline and not mapping:
            self.ensure()
            try:
                msg = rospy.wait_for_message(topic, Image, timeout=max(.01, deadline-time.monotonic()))
            except rospy.ROSException:
                break
            stamp = msg.header.stamp.to_nsec()
            if stamp <= start:
                continue
            image = bridge.imgmsg_to_cv2(msg, 'bgr8')
            if tuple(image.shape[1::-1]) != views[0].image_size:
                continue
            visible = visible_board_view(image, views[0])
            for view in ([visible] if visible is not None else views):
                detections = matcher.detect_board(view.normalize(image))['detections']
                slots = {}
                for detection in sorted(detections, key=lambda item: item.score, reverse=True):
                    slot = int(3*detection.x/view.normalized_size[0])
                    if 0 <= slot < len(zones):
                        slots.setdefault(slot, detection)
                last_view = (msg, image, view, tuple(slots.values()), stamp, visible is not None)
                if {item.label for item in slots.values()} == set(OBJECTS):
                    mapping = {slots[i].label: zone for i, zone in enumerate(zones)}
                    break

        accepted = bool(mapping)
        reason = 'accepted' if accepted else 'board_not_read'
        if last_view is not None:
            msg, image, view, detections, stamp, observed = last_view
            image_file = directory / 'board_frame.png'
            cv2.imwrite(str(image_file), image)
            evidence.update(image=str(image_file.relative_to(self.run)), stamp_ns=stamp,
                            selected_geometry_id=view.geometry_id, geometry=view.evidence(),
                            geometry_source='visible_outline' if observed else 'recorded_reference')
            overlay.render(msg, image, view, (), detections, zones, reason, mapping,
                           'Fixed slots: B | C | D', save=directory / 'mission_overlay.png')
        self.event('board_result', accepted=accepted, reason=reason, mapping=mapping, **evidence)
        return BoardDecision(accepted, reason, mapping, evidence)

    def close(self):
        if self.active_pub is not None:
            self.active_pub.unregister();self.active_pub=None
        return self.session.close() if self.session is not None else {}
