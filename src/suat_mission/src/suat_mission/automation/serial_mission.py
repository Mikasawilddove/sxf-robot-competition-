"""Measured serial manipulation connected to the competition scene stream."""
import json
import math
from pathlib import Path
import time

from suat_mission.contracts import ActionResult, Outcome
from suat_mission.manipulation.arm_path import FEED_APPROACH, FEED_FAST, FEED_LOADED, vertical_lift_points
from suat_mission.manipulation.pickup_geometry import PickupGeometry, GeometryUnavailable
from suat_mission.navigation.map_identity import validate_map_identity
from suat_mission.navigation.navigation_map import load_binding
from suat_mission.navigation.board_stations import board_station_candidates
from suat_mission.navigation.stations import mission_stations


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
    stations = mission_stations(root, result, binding)
    points = {name: entry['pose'] for name, entry in stations.items()}
    if result.get('pickup_stations_file'):
        result['pickup_stations_file'] = str(resource(root,result['pickup_stations_file']))
    result['waypoints'] = points
    if (result.get('mission_scope') == 'cake-only'
            and not (result.get('navigation_keepout_file')
                     and geometry.values.get('reachable_top_selection') is True)):
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
        target=target or self.selected_target
        payload={
            'phase':phase,'object_id':getattr(target,'object_id',self.held or ''),
            'label':getattr(target,'label',self.held_label),
            'calibration_id':self.geometry.calibration_id,'stamp':rospy.Time.now().to_sec(),
            'zone':self.zone,
            'arm_pose_mm':self.arm_pose,'cup_offset_mm':self.geometry.cup_offset}
        if target is not None and phase in ('approach','align_tool'):
            surface=target.surface
            if surface.get('top_polygon_px') and surface.get('image_size'):
                payload['locked_top']={'polygon_px':surface['top_polygon_px'],
                                       'image_size':surface['image_size'],
                                       'top_z_mm':self.geometry.point(target)[2]}
        self.active_pub.publish(String(data=json.dumps(payload)))

    def open(self, *, preparing=False, check_preparing=None, write_if_running=None):
        if self.session is not None:
            if not preparing and not self.session.initialized:
                self.session.enable_motion()
                self.arm_pose=self.session.read_pose(time.monotonic()+3.)['xyz']
                self.event('serial_initialized', measured_pose_mm=self.arm_pose)
            return
        from suat_mission.manipulation.arm_session import ArmSession
        self.session=ArmSession(self.run,feed=float(self.config.get('arm_feed_mm_min',1000.)))
        self.session.fast_feed=float(self.config.get('arm_fast_feed_mm_min',FEED_FAST))
        self.session.contact_feed=float(self.config.get('arm_contact_feed_mm_min',FEED_APPROACH))
        self.session.correction_feed=float(self.config.get('arm_correction_feed_mm_min',1000.))
        self.session.loaded_feed_limit = FEED_LOADED
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

    def move(self, phase, xyz, deadline, loaded=False, *, feed_limit=None):
        self.ensure()
        self.geometry.check_pose(xyz)
        self.event(phase,status='moving',target_mm=xyz)
        motion_options = {} if feed_limit is None else {'feed_limit': feed_limit}
        measurement=self.session.go(phase,xyz,loaded=loaded,deadline=deadline,**motion_options)
        self.arm_pose=measurement['measurements'][-1]['xyz']
        self.observed_after=time.monotonic()
        self.event(phase, target_mm=xyz, measured_pose_mm=self.arm_pose, measurement=measurement)
        self.active(phase)

    def prepare_empty_observation(self, unfolded_pose, deadline, *, fold_feed_limit):
        self.open()
        self.ensure()
        if self.held is not None or self.session.pump_may_be_on:
            raise RuntimeError('workpose recovery requires an empty arm with the pump off')
        home = self.geometry.check_pose(self.geometry.values['empty_travel_pose_mm'])
        side = self.geometry.check_pose(self.geometry.values['observation_pose_mm'])
        tolerance = self.session.correction_tolerance_mm
        start = self.arm_pose
        if min(math.dist(start, home), math.dist(start, side)) <= tolerance:
            return

        # The front fold is below the normal grasp workspace. All motion
        # targets still pass the existing workspace and controller IK checks.
        front = self.geometry.bounds[0] if self.geometry.bounds is not None else 0.
        if 0. < start[0] < front and math.hypot(*start[:2]) < front and start[2] < unfolded_pose[2]:
            self.move('unfold_empty_arm', unfolded_pose, deadline, feed_limit=fold_feed_limit)
            self.move('center_workpose', home, deadline)
        else:
            self.geometry.check_pose(start)
            if start[2] < unfolded_pose[2] - tolerance:
                clearance = [start[0], start[1], unfolded_pose[2]]
                self.move('raise_empty_arm', clearance, deadline)

    def observation(self, kind, deadline, *, feed_limit=None):
        if self.held is not None:
            raise RuntimeError('cannot use an empty observation pose while carrying')
        key='board_observation_pose_mm' if kind=='board' else 'observation_pose_mm'
        pose=self.geometry.values.get(key,self.geometry.values.get('observation_pose_mm'))
        if pose is None:
            raise GeometryUnavailable('missing observation pose: '+key)
        self.move('observe_'+kind,list(pose),deadline,feed_limit=feed_limit)

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

    def reachable_grasp(self,contact,approach,lift,deadline):
        points=[contact,[contact[0],contact[1],contact[2]+approach],
                [contact[0],contact[1],contact[2]+lift]]
        for point in points:
            try:
                self.geometry.check_pose(point)
            except GeometryUnavailable:
                return False
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
        observations={'rgbd_frames':0,'cup_frames':0,'top_frames':0}
        tool_evidence={}
        top_evidence={}
        until=min(deadline,time.monotonic()+8.)
        try:
            while time.monotonic()<until:
                scene=self.fresh_scene(until)
                observations['rgbd_frames']+=1
                current=next((d for d in scene.detections if d.object_id==target.object_id),None)
                tool_evidence=scene.manipulation.get('tool',{})
                top_evidence=scene.manipulation.get('target_top') or {}
                surface=current.surface if current is not None else {}
                if top_evidence.get('state')=='observed' and top_evidence.get('object_id')==target.object_id:
                    surface=top_evidence['surface']
                cup=self.observed_cup(scene)
                normal=surface.get('plane_normal_arm')
                offset=surface.get('plane_offset_arm_mm')
                top_valid=(normal and len(normal)==3 and all(math.isfinite(v) for v in normal)
                           and normal[2]>0. and offset is not None and math.isfinite(offset))
                observations['cup_frames']+=int(cup is not None)
                observations['top_frames']+=int(bool(top_valid))
                if not top_valid or cup is None:
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
                try:
                    self.geometry.check_pose(goal)
                except GeometryUnavailable as exc:
                    raise BaseAlignmentRequired(error) from exc
                if not self.session.validate_target(goal,deadline):
                    raise BaseAlignmentRequired(error)
                self.move('align_tool',goal,deadline)
                corrections+=1
                aligned.clear()
                until=min(deadline,time.monotonic()+8.)
        except TimeoutError:
            pass
        self.event('pregrasp_depth_timeout',object_id=target.object_id,**observations,
                   tool=tool_evidence,target_top=top_evidence)
        raise GeometryUnavailable('pre-descent RGB-D evidence did not converge '
                                  '(frames={rgbd_frames}, cup={cup_frames}, top={top_frames})'.format(**observations))

    def pregrasp_view_margin(self, contact, approach):
        if not self.geometry.values.get('require_visual_alignment') or self.last_scene is None:
            return None
        view=self.last_scene.manipulation
        camera=view.get('camera') or {}
        marker_margin=view.get('tool',{}).get('view_margin_mm')
        translation=view.get('calibration',{}).get('legacy_camera_B_mm')
        matrix=self.geometry.values.get('legacy_camera_A')
        if not camera or marker_margin is None or not translation or not matrix:
            return None
        pose=[contact[0],contact[1],contact[2]+approach]
        x,y,z=[sum(a*b for a,b in zip(row,pose))+offset for row,offset in zip(matrix,translation)]
        if z<=0.:
            return -math.inf
        fx,fy,cx,cy=camera['intrinsics']
        w,h=camera['image_size']
        u,v=fx*x/z+cx,fy*y/z+cy
        return min(u,v,w-1-u,h-1-v)-marker_margin*max(fx,fy)/z

    def select_pickup_pose(self,target,approach,lift,deadline):
        choices=[]
        for option in self.geometry.pickup_options(target):
            try:
                dx,dy=self.geometry.base_adjustment(option)
                contact=self.geometry.contact_target(option)
            except GeometryUnavailable:
                continue
            clearance=float(option.surface.get('edge_margin_mm',0.))
            choices.append((math.hypot(dx,dy),-clearance,option,contact,dx,dy))
        if not choices:
            raise GeometryUnavailable('no supported suction point on the observed top')
        # Minimal or zero base motion first. Try the best-supported point and
        # a shorter reach early, before spending serial queries on near ties.
        choices.sort(key=lambda entry:entry[:2])
        stationary=[entry for entry in choices if entry[0]<=1.]
        if len(stationary)>1:
            nearer=min(range(1,len(stationary)),key=lambda i:math.hypot(*stationary[i][3][:2]))
            stationary.insert(1,stationary.pop(nearer))
        margins=[self.pregrasp_view_margin(entry[3],approach) for entry in stationary]
        if margins and margins[0] is not None:
            order=sorted(range(len(stationary)),key=lambda i:(margins[i]<0,i if margins[i]>=0 else -margins[i]))
            stationary=[stationary[i] for i in order]
        for _,_,option,contact,dx,dy in stationary[:5]:
            if self.reachable_grasp(contact,approach,lift,deadline):
                margin=self.pregrasp_view_margin(contact,approach)
                if margin is not None:
                    self.event('pregrasp_view_selected',object_id=option.object_id,
                               predicted_tool_margin_px=margin if math.isfinite(margin) else None)
                return option, ()
        return None, choices

    def pickup_corrections(self, choices, approach, lift, deadline):
        limit=float(self.config.get('max_base_adjustment_mm',100.))
        offsets=[]
        for distance,_,_,contact,dx,dy in choices:
            if distance<=1.:
                continue
            if self.config.get('navigation_keepout_file'):
                margin=1000.*float(self.config.get('base_alignment_tolerance_m',.01))
                dx=dx+math.copysign(margin,dx) if abs(dx)>1. else 0.
                dy=dy+math.copysign(margin,dy) if abs(dy)>1. else 0.
            if 1.<math.hypot(dx,dy)<=limit:
                offsets.append((dx,dy,contact))
        # Some top points lie within the rectangle but fail the full arm path.
        if choices[0][0]<=1.:
            contact=choices[0][3]
            offsets.extend((x,y,contact) for x in range(-100,101,25) for y in range(-75,76,25)
                           if 0<math.hypot(x,y)<=min(limit,100.))
        tried=set()
        for dx,dy,contact in sorted(offsets,key=lambda item:math.hypot(*item[:2])):
            offset=(round(dx,1),round(dy,1))
            if offset in tried:
                continue
            if time.monotonic()>=deadline:
                return
            if self.reachable_grasp([contact[0]-dx,contact[1]-dy,contact[2]],approach,lift,deadline):
                tried.add(offset)
                yield dx,dy

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

    def pick(self, target, label, context, _alignment_round=0, *, return_to_support=False):
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
                selected,choices=self.select_pickup_pose(
                    target,approach,lift,context.deadline)
                if selected is not None:
                    target=selected
                    break
                if index>=remaining_adjustments:
                    raise GeometryUnavailable('no reachable grasp from the stopped pickup station')
                refusal='no controller-reachable local base adjustment for the observed top'
                stowed=False
                for dx,dy in self.pickup_corrections(choices,approach,lift,context.deadline):
                    if not stowed:
                        self.move('empty_travel',self.geometry.values['empty_travel_pose_mm'],context.deadline)
                        stowed=True
                    report=self.align_base(dx,dy,context,_alignment_round)
                    self.event('base_align',report=report,requested_offset_mm=[dx,dy])
                    if report['passed']:
                        _alignment_round+=1
                        break
                    if not report.get('pickup_plan_refused') or report.get('motion_goal_sent'):
                        return ActionResult(context,Outcome.FAILED,'base alignment failed: '+json.dumps(report))
                    refusal=report['error']
                else:
                    if stowed:
                        self.observation('objects',context.deadline)
                    raise GeometryUnavailable(refusal)
                self.observation('objects',context.deadline)
                target=self.fresh_target(target.object_id,context.deadline)
            contact=self.geometry.contact(target)
            # Holding evidence tracks the physical target, independent of the
            # particular suction point selected for contact.
            original_target=self.observed_target(self.last_scene,target.object_id,label)
            top_before=self.geometry.point(original_target or target)
            _,projection=self.geometry.grasp_projection(target)
            self.event('grasp_projection',object_id=target.object_id,contact_command_mm=contact,
                       **projection)
        except GeometryUnavailable as exc:
            return self.unreachable_pick(context,exc)
        approach=float(self.geometry.values.get('approach_offset_mm',40.))
        lift=float(self.geometry.values.get('lift_offset_mm',50.))
        measured_lift=(self.session.loaded_feed_limit is not None
                       or self.geometry.values.get('reachable_top_selection') is True)
        if measured_lift and (not math.isfinite(lift) or lift<=self.session.correction_tolerance_mm):
            return self.unreachable_pick(context,'lift must exceed the arm arrival tolerance')
        self.selected_target=target
        self.active('pick',target)
        hover_z=contact[2]+approach
        side=self.geometry.values.get('observation_pose_mm')
        tolerance=self.session.correction_tolerance_mm
        if side and math.dist(self.arm_pose,side)<=tolerance and abs(self.arm_pose[1])>tolerance:
            # Return from the side observation at the same radius and height,
            # then approach the block horizontally with a clear suction lip.
            self.move('pregrasp_return_front',[math.hypot(*self.arm_pose[:2]),0.,self.arm_pose[2]],context.deadline)
            if abs(self.arm_pose[2]-hover_z)>tolerance:
                self.move('pregrasp_clearance',[*self.arm_pose[:2],hover_z],context.deadline)
        self.move('approach',[contact[0],contact[1],hover_z],context.deadline)
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
                if report.get('pickup_plan_refused'):
                    self.observation('objects',context.deadline)
                    return self.unreachable_pick(context,report['error'])
                if not report['passed']:return ActionResult(context,Outcome.FAILED,'visual base adjustment failed: '+json.dumps(report))
                self.observation('objects',context.deadline)
                return self.pick(target,label,context,_alignment_round+1,return_to_support=return_to_support)
            except GeometryUnavailable as exc:
                self.event('visual_alignment_rejected', object_id=target.object_id, error=str(exc))
                self.observation('objects',context.deadline)
                return self.unreachable_pick(context,exc)
        self.move('contact',contact,context.deadline)
        lift_from = contact
        if measured_lift:
            # Anchor the vertical command to the measured contact pose. The cup
            # may have settled a few mm from the requested contact coordinates.
            lift_from = self.arm_pose
            for fraction in (.25, .5, .75, 1.):
                sample = [lift_from[0], lift_from[1], lift_from[2] + fraction * lift]
                try:
                    self.geometry.check_pose(sample)
                    valid=self.session.validate_target(sample, context.deadline)
                except GeometryUnavailable:
                    valid=False
                if not valid:
                    return ActionResult(context,Outcome.FAILED,
                                        'pump is off; lift from measured contact is outside controller reach')
        lift_target, = vertical_lift_points(lift_from, lift_from[2]+lift)
        self.session.pump(True,deadline=context.deadline)
        self.held=target.object_id  # possible holding until image evidence settles it
        self.held_label=label
        self.move('lift',lift_target,context.deadline,loaded=True)
        hold_pose = self.geometry.values.get('hold_observation_pose_mm')
        if hold_pose:
            self.move('hold_view',hold_pose,context.deadline,loaded=True)
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
        if return_to_support:
            self.event('holding_confirmed',object_id=target.object_id,observed_top_mm=point)
            if hold_pose:
                self.move('static_return_above',lift_target,context.deadline,loaded=True)
            self.move('static_return_contact',lift_from,context.deadline,loaded=True)
            self.session.pump(False,deadline=context.deadline)
            self.move('verify_placement',lift_target,context.deadline)
            self._wait_target(target.object_id,context.deadline,lambda d,p:
                math.dist(p,top_before)<=float(self.geometry.values.get('holding_position_tolerance_mm',25.))
                and self.observed_empty_for(self.last_scene,target.object_id),stable_mm=5.)
            self.event('returned_to_support',object_id=target.object_id)
            self.held=None;self.held_label=''
            self.observation('objects',context.deadline)
            return ActionResult(context,Outcome.CONFIRMED,'measured lift, visual hold and return to original support confirmed')
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
                polygon=placement.get('polygon_arm_mm')
                if (not isinstance(polygon, (list, tuple)) or len(polygon)<3
                        or any(not isinstance(p, (list, tuple)) or len(p)!=2
                               or not all(type(v) in (int, float) and math.isfinite(v) for v in p)
                               for p in polygon)):
                    return ActionResult(context,Outcome.UNKNOWN,'no valid observed destination polygon')
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
                    polygon=None
                    self.event('transport_fallback_plan',zone=zone,release_mm=release,recorded_region=region)
                else:
                    region=[min(p[0] for p in polygon),max(p[0] for p in polygon),min(p[1] for p in polygon),max(p[1] for p in polygon)]
                    self.event('visual_release_plan',zone=zone,release_mm=release,observed_region=placement)
        else:
            release=self.geometry.release(zone,label,slot)
            support=float(self.geometry.values.get('drop_support_z_mm',{}).get(zone,self.geometry.support_z))
        region=region or self.geometry.values.get('drop_regions_arm_mm',{}).get(zone)
        if (not isinstance(region, (list, tuple)) or len(region)!=4
                or not all(type(v) in (int, float) and math.isfinite(v) for v in region)
                or region[0]>=region[1] or region[2]>=region[3] or not math.isfinite(support)):
            return ActionResult(context,Outcome.UNKNOWN,'no measured placement region for '+zone)
        above=[release[0],release[1],max(release[2]+40.,self.arm_pose[2])]
        self.active('place')
        self.move('place_approach',above,context.deadline,loaded=True)
        self.move('release_height',release,context.deadline,loaded=True)
        self.session.pump(False,deadline=context.deadline)
        self.move('withdraw',above,context.deadline)
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
        from suat_mission.vision.board_recognition import BoardDecision, BoardReader, DirectionEvidence
        from suat_mission.vision.board_station_view import observe_zone_markers, station_views
        from suat_mission.vision.mission_overlay import MissionOverlay, draw_board
        from suat_mission.vision.template_matcher import TemplateMatcher

        geometry_file, data, views = station_views(self.config, station)
        start = rospy.Time.now().to_nsec()
        deadline = min(time.monotonic() + float(self.config.get('board_timeout', 20.)),
                       float(self.config.get('_deadline', float('inf'))))
        topic = self.config.get('board_image_topic', '/camera/color/image_raw')
        bridge = CvBridge()
        overlay = MissionOverlay('board', bridge, draw_board)
        directory = self.run / 'board' / station
        directory.mkdir(parents=True, exist_ok=True)
        evidence = dict(run_id=run_id, station=station, geometry_file=str(geometry_file),
                        view_ids=[v.geometry_id for v in views], frames=[])
        overlay_frame = None
        direction_caption = 'Waiting for direction evidence'

        def finish(accepted, reason, mapping, details=None):
            if details:
                evidence.update(details)
            if overlay_frame is not None:
                frame, rgb, view, codes, detections, slots = overlay_frame
                status = ('confirmed' if accepted else 'rejected') + ' | ' + reason
                overlay.render(frame, rgb, view, codes, detections, slots, status, mapping,
                               direction_caption, save=directory / 'mission_overlay.png')
            self.event('board_result', accepted=accepted, reason=reason, mapping=mapping, **evidence)
            return BoardDecision(accepted, reason, mapping, evidence)

        last = start
        def next_image():
            nonlocal last
            while time.monotonic() < deadline:
                self.ensure()
                try:
                    msg = rospy.wait_for_message(topic, Image, timeout=min(3., max(.01, deadline-time.monotonic())))
                except rospy.ROSException:
                    if rospy.is_shutdown():
                        raise
                    continue
                stamp = msg.header.stamp.to_nsec()
                if stamp <= last or not 0 <= rospy.Time.now().to_nsec()-stamp <= 2000000000:
                    continue
                if not msg.header.frame_id:
                    continue
                last = stamp
                return msg, bridge.imgmsg_to_cv2(msg, 'bgr8')
            return None

        probe = next_image()
        if probe is None:
            return finish(False, 'no_fresh_board_image', {})
        marker_msg, image = probe
        overlay_frame = (marker_msg, image, None, (), (), ())
        if tuple(image.shape[1::-1]) != views[0].image_size:
            return finish(False, 'station_camera_size_mismatch', {})
        marker_file = directory / 'zone_markers.png'
        if not cv2.imwrite(str(marker_file), image):
            raise RuntimeError('cannot save station marker image')
        try:
            markers = observe_zone_markers(self.root, image)
        except cv2.error as exc:
            markers = []
            evidence['zone_marker_error'] = str(exc)
        observed = tuple(m['zone'] for m in markers)
        expected = tuple(data['zones'])
        source = 'fixed_reference'
        reference = data['direction_reference']
        confirmed = data.get('direction_confirmed') is True
        zones = expected
        overlay_frame = (marker_msg, image, views[0], markers, (), expected)
        marker_status = ', '.join('%s/%s' % (m['zone'] or '?', m['source']) for m in markers) or 'none'
        direction_caption = 'Markers: ' + marker_status
        overlay.render(*overlay_frame, 'checking direction', {}, direction_caption,
                       save=directory / 'zone_markers_overlay.png')
        evidence['zone_markers'] = dict(image=str(marker_file.relative_to(self.run)), observations=markers,
                                       frame_id=marker_msg.header.frame_id, stamp_ns=last)
        if len(observed) == 3:
            if confirmed and any(z is not None and z != expected[i] for i, z in enumerate(observed)):
                return finish(False, 'station_markers_conflict_with_slot_order', {})
            if all(observed):
                if set(observed) != {'B','C','D'}:
                    return finish(False, 'duplicate_station_marker_zones', {})
                source, reference, confirmed, zones = 'station_zone_markers', str(marker_file), True, observed
        evidence['direction_source'] = source
        direction_caption = 'Direction: ' + source + ' | markers: ' + marker_status
        if not confirmed:
            return finish(False, 'unconfirmed_station_direction', {})

        readers = []
        for view in views:
            direction = DirectionEvidence(run_id, view.geometry_id, source, reference, zones, confirmed,
                                          stamp_ns=last if source == 'station_zone_markers' else None,
                                          observation_frame=marker_msg.header.frame_id)
            readers.append(BoardReader(run_id, view, direction, started_ns=start,
                min_votes=int(self.config.get('board_votes', 3)),
                threshold=float(self.config.get('board_min_confidence', .6))))
        templates = {label: [cv2.imread(str(self.root/'src/suat_mission/assets/templates'/(label+'.png')))]
                     for label in ('clock', 'teddybear', 'banana')}
        matcher = TemplateMatcher(templates, scales=[.4,.5,.6,.7,.8,.9,1.,1.1,1.2], threshold=.6, max_peaks=8)
        for index in range(int(self.config.get('board_frames', 8))):
            sample = next_image()
            if sample is None:
                break
            msg, image = sample
            if msg.header.frame_id != marker_msg.header.frame_id or tuple(image.shape[1::-1]) != views[0].image_size:
                return finish(False, 'station_camera_changed', {})
            image_file = directory / ('frame_%02d.png' % index)
            if not cv2.imwrite(str(image_file), image):
                raise RuntimeError('cannot save board image')
            name = str(image_file.relative_to(self.run))
            row = dict(image=name, stamp_ns=last, views={})
            frame_candidates = []
            for reader in readers:
                matches = matcher.detect_board(reader.geometry.normalize(image))
                decision = reader.observe(matches['detections'], coordinate_space='normalized_board', run_id=run_id,
                    stamp_ns=last, now_ns=rospy.Time.now().to_nsec(), frame_id=msg.header.frame_id, image_id=name)
                row['views'][reader.geometry.geometry_id] = decision.reason
                frame_candidates.append((reader, matches['detections'], decision))
            evidence['frames'].append(row)
            now = rospy.Time.now().to_nsec()
            results = [reader.result(now_ns=now) for reader in readers]
            best = max(range(len(readers)), key=lambda i: (results[i].accepted, frame_candidates[i][2].accepted))
            reader, detections, frame_decision = frame_candidates[best]
            overlay_frame = (msg, image, reader.geometry, (), detections, zones)
            status = '%s | %s | %d/%d votes' % (
                reader.geometry.geometry_id, frame_decision.reason,
                len(results[best].evidence['support_frames']), reader.min_votes)
            overlay.render(*overlay_frame, status, frame_decision.mapping, direction_caption)
            mappings = {tuple(sorted(vote['mapping'].items())) for result in results
                        for vote in result.evidence['votes']}
            if len(mappings) > 1:
                return finish(False, 'conflicting_station_view_votes', {})
            for (reader, detections, _), result in zip(frame_candidates, results):
                if result.accepted and time.monotonic() < deadline:
                    self.ensure()
                    overlay_frame = (msg, image, reader.geometry, (), detections, zones)
                    return finish(True, result.reason, dict(result.mapping),
                                  dict(selected_geometry_id=reader.geometry.geometry_id,
                                       geometry=result.evidence['geometry'], votes=result.evidence['votes'],
                                       support_frames=result.evidence['support_frames']))
        return finish(False, 'insufficient_station_view_votes', {})

    def close(self):
        if self.active_pub is not None:
            self.active_pub.unregister();self.active_pub=None
        return self.session.close() if self.session is not None else {}
