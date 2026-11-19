"""Measured serial manipulation connected to the competition scene stream."""
import json
import math
from pathlib import Path
import time

from suat_mission.contracts import ActionResult, Outcome
from suat_mission.manipulation.pickup_geometry import PickupGeometry, GeometryUnavailable
from suat_mission.navigation.map_identity import validate_map_identity
from suat_mission.navigation.navigation_map import load_binding
from suat_mission.navigation.board_stations import board_station_candidates


class BaseAlignmentRequired(GeometryUnavailable):
    def __init__(self,error):
        super().__init__('measured cup correction is outside controller reach')
        self.error=error


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
    points = {p['name']:p['pose'] for p in route['waypoints']}
    for name,pose in result.get('waypoints',{}).items():
        if name in points and points[name]!=pose:
            raise ValueError('mission config conflicts with recorded station: '+name)
        points[name]=pose
    bcd = result.get('additional_waypoints_file')
    if bcd and resource(root,bcd).exists():
        data=json.loads(resource(root,bcd).read_text(encoding='utf-8'))
        validate_map_identity(data,binding)
        additions=data.get('waypoints',data)
        if isinstance(additions,list): additions={p['name']:p['pose'] for p in additions}
        for name,pose in additions.items():
            if name in points and points[name]!=pose:
                raise ValueError('additional waypoints conflict with recorded station: '+name)
            points[name]=pose
    if result.get('pickup_stations_file'):
        pickup_file = resource(root, result['pickup_stations_file'])
        pickup = json.loads(pickup_file.read_text(encoding='utf-8'))
        validate_map_identity(pickup, binding)
        approaches = pickup.get('waypoints', {})
        if (pickup.get('frame_id') != 'map' or not isinstance(approaches, dict)
                or set(approaches) != {'Sorting_W', 'Sorting_S', 'Sorting_E', 'Sorting_N'}):
            raise ValueError('four map-frame pickup approaches required in ' + str(pickup_file))
        for name, pose in approaches.items():
            if (not isinstance(pose, list) or len(pose) != 3
                    or not all(type(v) in (int, float) and math.isfinite(v) for v in pose)):
                raise ValueError('invalid pickup approach: ' + name)
        points.update(approaches)
        result['pickup_stations_file'] = str(pickup_file)
    result['waypoints'] = points
    if result.get('mission_scope') == 'cake-only':
        result['max_base_alignments'] = 0
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
    def __init__(self, root, config, next_scene, ensure_running, align_base):
        self.root, self.config = Path(root), config
        self.confirm_frames = config.get('manipulation_confirm_frames', 2)
        if type(self.confirm_frames) is not int or self.confirm_frames < 1:
            raise ValueError('manipulation_confirm_frames must be a positive integer')
        self.geometry = PickupGeometry.load(config['pickup_geometry_file'])
        self.next_scene, self.ensure, self.align_base = next_scene, ensure_running, align_base
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

    def event(self, phase, **values):
        self.events.append(dict(phase=phase, at=time.monotonic(), **values))
        (self.run/'events.json').write_text(json.dumps(self.events,ensure_ascii=False,indent=2),encoding='utf-8')

    def active(self, phase, target=None):
        import rospy
        from std_msgs.msg import String
        if self.active_pub is None:
            self.active_pub=rospy.Publisher('/suat/manipulation/active',String,queue_size=1,latch=True)
        self.active_pub.publish(String(data=json.dumps({
            'phase':phase,'object_id':getattr(target or self.selected_target,'object_id',self.held or ''),
            'label':getattr(target or self.selected_target,'label',self.held_label),
            'calibration_id':self.geometry.calibration_id,'stamp':rospy.Time.now().to_sec(),
            'zone':self.zone,
            'arm_pose_mm':self.arm_pose,'cup_offset_mm':self.geometry.cup_offset})))

    def open(self):
        if self.session is not None:
            return
        from suat_mission.manipulation.arm_session import ArmSession
        self.session=ArmSession(self.run,feed=float(self.config.get('arm_feed_mm_min',1000.)))
        self.session.fast_feed=float(self.config.get('arm_fast_feed_mm_min',80000.))
        self.session.contact_feed=float(self.config.get('arm_contact_feed_mm_min',40000.))
        self.session.correction_feed=float(self.config.get('arm_correction_feed_mm_min',1000.))
        if self.config.get('mission_scope') == 'cake-only':
            self.session.loaded_feed_limit = 10000.
        self.session.permitted=lambda: self._permitted()
        self.session.open()
        self.arm_pose=self.session.read_pose(time.monotonic()+3.)['xyz']
        self.event('serial_open', measured_pose_mm=self.arm_pose)

    def _permitted(self):
        try:
            self.ensure()
            return True
        except Exception:
            return False

    def move(self, phase, xyz, deadline, loaded=False):
        self.ensure()
        self.geometry.check_pose(xyz)
        self.event(phase,status='moving',target_mm=xyz)
        measurement=self.session.go(phase,xyz,loaded=loaded,deadline=deadline)
        self.arm_pose=self.session.read_pose(deadline)['xyz']
        self.observed_after=time.monotonic()
        self.event(phase, target_mm=xyz, measured_pose_mm=self.arm_pose, measurement=measurement)
        self.active(phase)

    def observation(self, kind, deadline):
        if self.held is not None:
            raise RuntimeError('cannot use an empty observation pose while carrying')
        key='board_observation_pose_mm' if kind=='board' else 'observation_pose_mm'
        pose=self.geometry.values.get(key,self.geometry.values.get('observation_pose_mm'))
        if pose is None:
            raise GeometryUnavailable('missing observation pose: '+key)
        self.move('observe_'+kind,list(pose),deadline)
        self.active('observe_'+kind)

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
        self.bootstrapped=True

    def reachable_grasp(self,contact,approach,lift,deadline):
        points=[contact,[contact[0],contact[1],contact[2]+approach],
                [contact[0],contact[1],contact[2]+lift]]
        for point in points:
            self.geometry.check_pose(point)
            if not self.session.validate_target(point,deadline):return False
        # Check the actual straight connections, not merely isolated endpoints.
        for a,b in zip(points,points[1:]):
            for t in (.25,.5,.75):
                if not self.session.validate_target([a[i]+(b[i]-a[i])*t for i in range(3)],deadline):return False
        return True

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

    def align_tool(self,target,deadline):
        # Keep the point fitted before the arm obscures the centre of the face.
        # A fresh frame must still show the same object and support plane.
        top=self.geometry.point(target)
        edge_clearance=target.surface['edge_margin_mm']-self.geometry.cup_radius
        tolerance=min(edge_clearance,float(self.geometry.values.get('alignment_tolerance_mm',5.)))
        height_tolerance=float(self.geometry.values.get('height_tolerance_mm',12.))
        step=float(self.geometry.values.get('max_visual_correction_mm',30.))
        required=self.confirm_frames
        aligned=[]
        corrections=0
        until=min(deadline,time.monotonic()+8.)
        try:
            while time.monotonic()<until:
                scene=self.fresh_scene(until)
                current=next((d for d in scene.detections if d.object_id==target.object_id),None)
                cup=self.observed_cup(scene)
                if current is None or cup is None:
                    aligned.clear()
                    continue
                surface=current.surface
                normal=surface.get('plane_normal_arm')
                offset=surface.get('plane_offset_arm_mm')
                if (not normal or len(normal)!=3 or not all(math.isfinite(v) for v in normal)
                        or normal[2]<=0. or offset is None or not math.isfinite(offset)):
                    aligned.clear()
                    continue
                height_at_lock=(offset-normal[0]*top[0]-normal[1]*top[1])/normal[2]
                if abs(height_at_lock-top[2])>height_tolerance:
                    raise GeometryUnavailable('selected top changed height after approach')
                error=[top[0]-cup[0],top[1]-cup[1]]
                distance=math.hypot(*error)
                gap=cup[2]-(offset-normal[0]*cup[0]-normal[1]*cup[1])/normal[2]
                if gap<0.:
                    raise GeometryUnavailable('observed cup is already below the selected top plane')
                self.event('visual_alignment',target_id=target.object_id,locked_top_mm=top,
                    observed_cup_mm=cup,xy_error_mm=error,remaining_vertical_gap_mm=gap)
                if distance<=tolerance:
                    if aligned and abs(aligned[-1]-gap)>tolerance:
                        aligned.clear()
                    aligned.append(gap)
                    if len(aligned)>=required:
                        return [self.arm_pose[0],self.arm_pose[1],self.arm_pose[2]-sum(aligned)/len(aligned)]
                    continue
                if corrections>=2:
                    raise GeometryUnavailable('visual cup/target error exceeds bounded local correction')
                scale=min(1.,step/distance)
                goal=[self.arm_pose[0]+error[0]*scale,self.arm_pose[1]+error[1]*scale,self.arm_pose[2]]
                self.geometry.check_pose(goal)
                if not self.session.validate_target(goal,deadline):
                    raise BaseAlignmentRequired(error)
                self.move('align_tool',goal,deadline)
                corrections+=1
                aligned.clear()
                until=min(deadline,time.monotonic()+8.)
        except TimeoutError as exc:
            raise GeometryUnavailable('fresh cup and target depth unavailable before descent') from exc
        raise GeometryUnavailable('cannot observe a stable cup and target depth before descent')

    def fresh_target(self, object_id, deadline):
        while time.monotonic()<deadline:
            self.ensure()
            scene=self.fresh_scene(deadline)
            target=next((d for d in scene.detections if d.object_id==object_id),None)
            if target is not None:
                return target
        raise TimeoutError('selected physical object not observed in a fresh scene')

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

    def _confirm_pick(self,target,top_before,lift,deadline):
        held_votes=empty_votes=0
        needed=self.confirm_frames
        tolerance=float(self.geometry.values.get('holding_position_tolerance_mm',25.))
        while time.monotonic()<deadline:
            scene=self.fresh_scene(deadline)
            current=self.observed_target(scene,target.object_id,target.label)
            if current is None:
                held_votes=empty_votes=0
                continue
            try:point=self.geometry.payload_point(current)
            except GeometryUnavailable:
                held_votes=empty_votes=0;continue
            held=(point[2]>=top_before[2]+lift*.5 and self.matches_observed_cup(scene,point))
            # Gripper empty is positive depth visibility, not missing detection.
            hand_empty=self.observed_empty_for(scene,target.object_id)
            empty=hand_empty and math.dist(point,top_before)<=tolerance
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

    def pick(self, target, label, context, _alignment_round=0):
        self.open();self.ensure()
        self.version=max(self.version,context.scene_version)
        self.selected_target=target
        if self.held is not None or self.session.pump_may_be_on:
            return ActionResult(context,Outcome.UNKNOWN,'hand is not established empty before a new pick')
        try:
            target=self.fresh_target(target.object_id,context.deadline)
            approach=float(self.geometry.values.get('approach_offset_mm',40.))
            lift=float(self.geometry.values.get('lift_offset_mm',50.))
            remaining_adjustments=max(0,int(self.config.get('max_base_alignments',2))-_alignment_round)
            for index in range(remaining_adjustments+1):
                if target.label != label:
                    return ActionResult(context,Outcome.UNKNOWN,'selected physical object changed class')
                if target.graspable is not True or target.occluded_by:
                    raise GeometryUnavailable('fresh target is occluded or lacks a graspable surface')
                dx,dy=self.geometry.base_adjustment(target)
                candidate=self.geometry.contact(target)
                if math.hypot(dx,dy)<=1. and self.reachable_grasp(candidate,approach,lift,context.deadline):
                    break
                if index>=remaining_adjustments:
                    raise GeometryUnavailable('no reachable grasp from the stopped pickup station')
                if math.hypot(dx,dy)<=1.:
                    choices=sorted(((x,y) for x in (-100.,-75.,-50.,-25.,0.,25.,50.,75.,100.)
                        for y in (-75.,-50.,-25.,0.,25.,50.,75.) if 0<math.hypot(x,y)<=100.),key=lambda p:math.hypot(*p))
                    choice=next(((x,y) for x,y in choices if self.reachable_grasp(
                        [candidate[0]-x,candidate[1]-y,candidate[2]],approach,lift,context.deadline)),None)
                    if choice is None:raise GeometryUnavailable('no controller-reachable approach/contact/lift after a local base adjustment')
                    dx,dy=choice
                if math.hypot(dx,dy)>float(self.config.get('max_base_adjustment_mm',100.)):
                    raise GeometryUnavailable('target remains outside the reliable work area')
                report=self.align_base(dx,dy,context,_alignment_round)
                _alignment_round+=1
                self.event('base_align',report=report)
                if not report['passed']:
                    return ActionResult(context,Outcome.FAILED,'base alignment failed: '+json.dumps(report))
                self.observed_after=time.monotonic()
                target=self.fresh_target(target.object_id,context.deadline)
            contact=self.geometry.contact(target)
            top_before=self.geometry.point(target)
            _,projection=self.geometry.grasp_projection(target)
            self.event('grasp_projection',object_id=target.object_id,contact_command_mm=contact,
                       **projection)
        except GeometryUnavailable as exc:
            return self.unreachable_pick(context,exc)
        approach=float(self.geometry.values.get('approach_offset_mm',40.))
        lift=float(self.geometry.values.get('lift_offset_mm',50.))
        self.active('pick',target)
        self.move('approach',[contact[0],contact[1],contact[2]+approach],context.deadline)
        if self.geometry.values.get('require_visual_alignment',False):
            try:
                contact=self.align_tool(target,context.deadline)
                self.geometry.check_pose(contact)
                if not self.reachable_grasp(contact,self.arm_pose[2]-contact[2],lift,context.deadline):
                    raise GeometryUnavailable('visually corrected approach/contact/lift is outside controller reach')
            except BaseAlignmentRequired as exc:
                self.move('empty_travel',self.geometry.values['empty_travel_pose_mm'],context.deadline)
                if _alignment_round>=int(self.config.get('max_base_alignments',2)) or math.hypot(*exc.error)>100.:
                    return self.unreachable_pick(context,exc)
                report=self.align_base(exc.error[0],exc.error[1],context,_alignment_round)
                self.event('visual_base_align',report=report,measured_error_mm=exc.error)
                if not report['passed']:return ActionResult(context,Outcome.FAILED,'visual base adjustment failed: '+json.dumps(report))
                self.observation('objects',context.deadline)
                return self.pick(target,label,context,_alignment_round+1)
            except GeometryUnavailable as exc:
                self.observation('objects',context.deadline)
                return self.unreachable_pick(context,exc)
        self.move('contact',contact,context.deadline)
        lift_from = contact
        if self.session.loaded_feed_limit is not None:
            # Anchor the vertical command to the measured contact pose. The cup
            # may have settled a few mm from the requested contact coordinates.
            lift_from = self.arm_pose
            if not math.isfinite(lift) or lift <= self.session.correction_tolerance_mm:
                raise GeometryUnavailable('lift must exceed the arm arrival tolerance')
            for fraction in (.25, .5, .75, 1.):
                sample = [lift_from[0], lift_from[1], lift_from[2] + fraction * lift]
                self.geometry.check_pose(sample)
                if not self.session.validate_target(sample, context.deadline):
                    raise GeometryUnavailable('vertical lift from measured contact is outside controller reach')
        lift_target = [lift_from[0], lift_from[1], lift_from[2]+lift]
        self.session.pump(True,deadline=context.deadline)
        self.held=target.object_id  # possible holding until image evidence settles it
        self.held_label=label
        self.move('lift',lift_target,context.deadline,loaded=True)
        if self.geometry.values.get('hold_observation_pose_mm'):
            self.move('hold_view',self.geometry.values['hold_observation_pose_mm'],context.deadline,loaded=True)
        try:
            held,point=self._confirm_pick(target,top_before,lift,min(context.deadline,
                time.monotonic()+float(self.config.get('feedback_timeout',15.))))
        except TimeoutError:
            return ActionResult(context,Outcome.UNKNOWN,'pump may hold an object; lift not visually confirmed')
        if not held:
            self.session.pump(False,deadline=context.deadline)
            self.held=None;self.held_label=''
            self.observation('objects',context.deadline)
            return ActionResult(context,Outcome.NOT_ACQUIRED,'selected target stayed on support; gripper volume visibly empty')
        self.last_hold_seen=time.monotonic()
        self.move('transport',self.geometry.values['transport_pose_mm'],context.deadline,loaded=True)
        self.active('carrying',target)
        self._wait_target(target.object_id,min(context.deadline,time.monotonic()+
            float(self.config.get('feedback_timeout',15.))),lambda d,p:
            self.payload_at_cup(self.last_scene,p))
        self.last_hold_seen=time.monotonic()
        self.event('holding_confirmed',object_id=target.object_id,observed_top_mm=point)
        return ActionResult(context,Outcome.CONFIRMED,'fresh top motion matches measured cup lift: '+target.object_id)

    def check_carrying(self, scene):
        if self.held is None:
            return
        target=self.observed_target(scene,self.held,self.held_label) if scene and scene.scene_id==self.scene_id else None
        if target is not None:
            try:
                point=self.geometry.payload_point(target)
                if self.payload_at_cup(scene,point):
                    self.last_hold_seen=scene.observed_at
            except GeometryUnavailable:
                pass
        if time.monotonic()-self.last_hold_seen>float(self.config.get('holding_observation_timeout',2.)):
            raise RuntimeError('holding observation lost or stale; preserve suction and stop travel')

    def drop(self, zone, label, context):
        if self.held!=context.object_id or self.held_label!=label:
            return ActionResult(context,Outcome.UNKNOWN,'placement does not match the held object')
        self.observed_after=time.monotonic()
        self._wait_target(context.object_id,min(context.deadline,time.monotonic()+
            float(self.config.get('feedback_timeout',15.))),lambda d,p:
            self.payload_at_cup(self.last_scene,p))
        slot=self.slots.get(zone,0)
        region=None;polygon=None
        landing_kind='collection'
        self.zone=zone
        if self.geometry.values.get('placement_mode')=='visual_red_zone':
            self.observed_after=time.monotonic();self.active('observe_placement')
            until=min(context.deadline,time.monotonic()+8.);placement=None
            while time.monotonic()<until:
                scene=self.fresh_scene(until);candidate=scene.manipulation.get('placement')
                if candidate and candidate.get('zone')==zone and candidate.get('candidates_arm_mm'):
                    placement=candidate;break
            if placement is None:
                if zone=='A' or not self.config.get('allow_transport_zone_fallback',False):
                    return ActionResult(context,Outcome.UNKNOWN,'no observed reachable free floor in the destination red zone')
                try:release,region,support=self.geometry.transport_release(zone,label,slot)
                except GeometryUnavailable as exc:return ActionResult(context,Outcome.UNKNOWN,str(exc))
                landing_kind='transport'
                self.event('transport_fallback_plan',zone=zone,release_mm=release,recorded_region=region)
            else:
                support=float(placement['support_z_mm'])
                height=float(self.geometry.values.get('object_height_mm',{}).get(label,100.))
                z=support+height+float(self.geometry.values.get('release_bottom_clearance_mm',100.))+self.geometry.cup_offset
                release=next(([p[0],p[1],z] for p in placement['candidates_arm_mm'] if
                    self.session.validate_target([p[0],p[1],z],context.deadline) and
                    self.session.validate_target([p[0],p[1],max(z+40.,self.arm_pose[2])],context.deadline)),None)
                if release is None:
                    if zone=='A' or not self.config.get('allow_transport_zone_fallback',False):
                        return ActionResult(context,Outcome.UNKNOWN,'observed red zone is outside the arm reach')
                    try:release,region,support=self.geometry.transport_release(zone,label,slot)
                    except GeometryUnavailable as exc:return ActionResult(context,Outcome.UNKNOWN,str(exc))
                    landing_kind='transport'
                    self.event('transport_fallback_plan',zone=zone,release_mm=release,recorded_region=region)
                else:
                    polygon=placement['polygon_arm_mm']
                    region=[min(p[0] for p in polygon),max(p[0] for p in polygon),min(p[1] for p in polygon),max(p[1] for p in polygon)]
                    self.event('visual_release_plan',zone=zone,release_mm=release,observed_region=placement)
        else:
            release=self.geometry.release(zone,label,slot)
            support=float(self.geometry.values.get('drop_support_z_mm',{}).get(zone,self.geometry.support_z))
        above=[release[0],release[1],max(release[2]+40.,self.arm_pose[2])]
        self.active('place')
        self.move('place_approach',above,context.deadline,loaded=True)
        self.move('release_height',release,context.deadline,loaded=True)
        self.session.pump(False,deadline=context.deadline)
        self.move('withdraw',above,context.deadline)
        self.active('verify_placement')
        height=float(self.geometry.values.get('object_height_mm',{}).get(label,100.))
        region=region or self.geometry.values.get('drop_regions_arm_mm',{}).get(zone)
        if not region or len(region)!=4:
            return ActionResult(context,Outcome.UNKNOWN,'released, but no measured placement region for '+zone)
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
                and math.dist(p,[self.arm_pose[0],self.arm_pose[1],self.arm_pose[2]-self.geometry.cup_offset])>self.geometry.cup_radius,
                point_of=lambda d:self.geometry.point(d,require_fit=False),stable_mm=tolerance)
        except TimeoutError:
            return ActionResult(context,Outcome.UNKNOWN,'release commanded; correct final region not observed')
        self.held=None;self.held_label='';self.selected_target=None
        self.zone=None
        self.slots[zone]=slot+1
        self.move('empty_travel',self.geometry.values['empty_travel_pose_mm'],context.deadline)
        self.active('empty')
        self.event('placement_confirmed',object_id=context.object_id,zone=zone,slot=slot,point_mm=point,
                   landing_kind=landing_kind)
        return ActionResult(context,Outcome.CONFIRMED,'fresh stable released top inside '+zone+' '+landing_kind+' slot '+str(slot))

    def read_board(self, run_id, station):
        import cv2
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from suat_mission.vision.board_recognition import BoardGeometry, BoardReader, DirectionEvidence
        from suat_mission.vision.template_matcher import TemplateMatcher
        geometry_file = self.config['board_geometry_file']
        if station == 'Classification_area__recorded_reference':
            geometry_file = self.config.get('board_reference_geometry_file', geometry_file)
        data=json.loads(Path(geometry_file).read_text(encoding='utf-8'))
        geometry=BoardGeometry(tuple(data['image_size']),data['corners'],tuple(data['normalized_size']),data['geometry_id'])
        direction=DirectionEvidence(run_id,geometry.geometry_id,'fixed_reference',data['direction_reference'],
                                    tuple(data['zones']),data.get('direction_confirmed') is True)
        reader=BoardReader(run_id,geometry,direction,started_ns=rospy.Time.now().to_nsec(),
            min_votes=int(self.config.get('board_votes',3)),threshold=float(self.config.get('board_min_confidence',.6)))
        templates={n:[cv2.imread(str(self.root/'src/suat_mission/assets/templates'/(n+'.png')))]
                   for n in ('clock','teddybear','banana')}
        matcher=TemplateMatcher(templates,scales=[.4,.5,.6,.7,.8,.9,1.,1.1,1.2],threshold=.6,max_peaks=8)
        images_dir = self.run/'board'/station
        images_dir.mkdir(parents=True, exist_ok=True)
        bridge=CvBridge();last=reader.started_ns;rows=[]
        deadline=min(time.monotonic()+float(self.config.get('board_timeout',20.)),
                     float(self.config.get('_deadline',float('inf'))))
        while len(rows)<int(self.config.get('board_frames',5)) and time.monotonic()<deadline:
            self.ensure()
            msg=rospy.wait_for_message(self.config.get('board_image_topic','/camera/color/image_raw'),Image,timeout=3)
            stamp=msg.header.stamp.to_nsec()
            if stamp<=last:continue
            last=stamp;image=bridge.imgmsg_to_cv2(msg,'bgr8')
            matches=matcher.detect_board(geometry.normalize(image));detections=matches.pop('detections')
            image_file=images_dir/('frame_%02d.png'%len(rows))
            name=str(image_file.relative_to(self.run))
            if not cv2.imwrite(str(image_file),image):raise RuntimeError('cannot save board image')
            decision=reader.observe(detections,coordinate_space='normalized_board',run_id=run_id,
                stamp_ns=stamp,now_ns=rospy.Time.now().to_nsec(),frame_id=msg.header.frame_id,image_id=name)
            rows.append({'stamp_ns':stamp,'image':name,'accepted':decision.accepted})
        result=reader.result(now_ns=rospy.Time.now().to_nsec())
        self.event('board_result',station=station,geometry_file=geometry_file,
                   accepted=result.accepted,reason=result.reason,mapping=dict(result.mapping),frames=rows)
        return result

    def close(self):
        if self.active_pub is not None:
            self.active_pub.unregister();self.active_pub=None
        return self.session.close() if self.session is not None else {}
