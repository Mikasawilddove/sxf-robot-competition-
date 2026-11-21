#!/usr/bin/env python3
"""Isolated navigation output: initially no /cmd_vel publisher, explicit leased arm."""
import json
import math
from pathlib import Path
import threading
import time
import uuid
from collections import deque
import yaml

import rosgraph
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from suat_mission.navigation.navigation_gate import (
    NavigationGate, blocked_footprint_cell, goal_is_free, motion_bounds_fault, navigation_clients_fault,
)
from suat_mission.navigation.path_tracker import BRAKING_LATENCY_S, PathTracker, braking_arc, braking_speed
from suat_mission.navigation.navigation_map import load_binding, verify_map
from suat_mission.navigation.pickup_keepout import KEEP_OUT_FILE, PLANNING_MAP_TOPIC, PickupKeepout
from suat_mission.navigation.navigation_profiles import speed_profile, path_travel_fault
from suat_mission.navigation.corner_egress import (
    DISTANCE, SweepBlocked, anchor_check, check_rear_clearance,
    map_departure, scan_departure, sweep_observation_gap, contact_path,
)


MAX_EGRESS_SCAN_ODOM_SKEW_S = .10
SHORT_ECHO_M = .15
ECHO_RECHECK_TIMEOUT_S = 1.5
ECHO_RECHECK_SETTLE_S = .15
ECHO_RECHECK_CLEAR_SCANS = 3
TERMINAL_RECHECK_TIMEOUT_S = 1.5


class Node:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode=rospy.get_param('~mode','short')
        if self.mode not in ('short','field'):raise ValueError('unknown navigation mode')
        default_owner='/suat_field_navigation' if self.mode=='field' else '/suat_point_navigation_test'
        self.owner=rospy.get_param('~navigation_owner',default_owner)
        if self.owner not in ('/suat_field_navigation','/suat_point_navigation_test','/suat_mission'):
            raise ValueError('navigation owner must be one explicit supported client')
        self.has_armed=False
        self.input_samples={'raw':None,'smooth':None}
        self.speed_name=rospy.get_param('~speed_profile','slow')
        if self.mode=='short' and self.speed_name!='slow':raise ValueError('short mode requires slow speed')
        speed=speed_profile(self.speed_name)
        self.egress_accel=speed['accel_linear']
        self.speed=speed
        self.gate = NavigationGate(speed['linear'],speed['angular'],180. if self.mode=='field' else 60.)
        self.travel=0.
        self.previous_odom_pose=None
        self.pub = None
        self.stopping = threading.Event()
        self.scan = self.odom = self.goals = None
        self.scan_msg = self.map_grid = self.egress = None
        self.local_grid=None
        self.local_grid_at=0.
        self.local_frame=None
        self.odom_at_scan = None
        self.odom_history = deque(maxlen=200)
        self.still_since = None
        self.start_odom_pose = None
        self.graph = None
        self.graph_at = self.tf_at = 0.0
        self.tf_fault = 'TF_not_received'
        self.map_fault = 'map_not_verified'
        self.binding = load_binding(Path(rospy.get_param('~workspace_root')))
        self.workspace_root = Path(rospy.get_param('~workspace_root'))
        self.keepout = None
        self.keepout_file = rospy.get_param('~keepout_file',KEEP_OUT_FILE)
        self.planning_map_pub = None
        self.map_xy = self.map_pose = None
        self.route = None
        if rospy.get_param('~mission_keepout',False):
            self.keepout = PickupKeepout.load(self.workspace_root,self.binding,self.keepout_file)
            self.keepout_radius = float(rospy.get_param('/move_base/global_costmap/robot_radius',.20)) + float(
                rospy.get_param('/move_base/global_costmap/footprint_padding',.02))
            if not math.isfinite(self.keepout_radius) or self.keepout_radius<=0:
                raise ValueError('invalid footprint radius for central pickup keepout')
            self.planning_map_pub = rospy.Publisher(PLANNING_MAP_TOPIC,OccupancyGrid,queue_size=1,latch=True)
        self.profile = rospy.get_param('~profile', 'UNKNOWN')
        self.path_control = self.profile == 'field_path'
        if self.path_control and (self.speed_name != 'fast' or self.mode != 'field'):
            raise ValueError('the field path controller uses the fixed fast profile')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.state_pub = rospy.Publisher('/suat/navigation/state', String, queue_size=1, latch=True)
        self.subs = [
            rospy.Subscriber('/suat/navigation/heartbeat', String, self.heartbeat, queue_size=1),
            rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1),
            rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1),
            rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1),
            rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.on_local_map, queue_size=1),
            rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, self.on_local_update, queue_size=1),
            rospy.Subscriber('/move_base/status', GoalStatusArray, self.on_goals, queue_size=1)]
        if not self.path_control:
            self.subs.extend([
                rospy.Subscriber('/suat/navigation/raw_cmd_vel', Twist, self.command, 'raw', queue_size=1),
                rospy.Subscriber('/suat/navigation/smoothed_cmd_vel', Twist, self.command, 'smooth', queue_size=1)])
        self.services = [rospy.Service('~arm', Trigger, self.arm),
                         rospy.Service('~follow_path', Trigger, self.follow_path),
                         rospy.Service('~stop', Trigger, self.stop),
                         rospy.Service('~release', Trigger, self.release),
                         rospy.Service('~begin_a_egress', Trigger, self.begin_a_egress)]
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.monitor = threading.Thread(target=self.monitor_health, daemon=True)
        self.worker.start()
        self.monitor.start()
        rospy.on_shutdown(self.shutdown)

    def command(self, msg, source):
        # A loaded nodelet registers publishers under the manager process name.
        expected = '/move_base' if source == 'raw' else '/suat_navigation_nodelet_manager'
        header = getattr(msg, '_connection_header', {}) or {}
        with self.lock:
            values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
            if (header.get('callerid')==expected and header.get('latching')!='1'
                    and all(math.isfinite(value) for value in values)):
                self.input_samples[source]=values
            else:
                # None means never received. An invalid input must remain a
                # drain failure until a valid zero arrives from its owner.
                self.input_samples[source]=(float('nan'),)*6
            if self.gate.state == 'ARMED' and header.get('callerid') != expected:
                self.gate.stop('unexpected_' + source + '_publisher')
                return
            if getattr(self,'egress',None) and self.egress['status'] in ('WAIT_HEARTBEAT','ACTIVE'):
                return  # The gate supplies the straight departure command itself.
            self.gate.command(source, (msg.linear.x, msg.linear.y, msg.linear.z,
                                      msg.angular.x, msg.angular.y, msg.angular.z), time.monotonic(),
                              header.get('latching') == '1')

    def heartbeat(self, msg):
        try:
            value = json.loads(msg.data)
            with self.lock:
                accepted=self.gate.heartbeat(value.get('token'), value.get('sequence'), time.monotonic(),
                                    (getattr(msg, '_connection_header', {}) or {}).get('latching') == '1')
                if accepted and getattr(self,'egress',None) and self.egress['status']=='WAIT_HEARTBEAT':
                    self.egress['status']='ACTIVE'
        except (ValueError, AttributeError, TypeError):
            pass  # Invalid heartbeats never renew a lease.

    def on_scan(self, msg):
        with self.lock:
            self.scan = (time.monotonic(), msg.header.stamp.to_sec(), msg.header.frame_id)
            self.scan_msg = msg
            self.odom_at_scan = self.odom

    def on_odom(self, msg):
        now = time.monotonic()
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            self.odom = (now, msg.header.stamp.to_sec(), msg.header.frame_id, v, w, p.x, p.y, yaw)
            self.odom_history.append(self.odom)
            if self.gate.state == 'ARMED' and self.start_odom_pose is not None:
                if self.mode=='short':
                    fault = motion_bounds_fault(self.start_odom_pose, self.odom[5:8])
                else:
                    fault,self.travel=path_travel_fault(self.previous_odom_pose,self.odom[5:8],self.travel)
                    self.previous_odom_pose=self.odom[5:8]
                if fault:
                    self.gate.stop(fault)
            if math.isfinite(v) and math.isfinite(w) and abs(v) < .015 and abs(w) < .03:
                if self.still_since is None:
                    self.still_since = now
            else:
                self.still_since = None

    def on_goals(self, msg):
        with self.lock:
            self.goals = (time.monotonic(), any(s.status in (0, 1, 6, 7) for s in msg.status_list))

    def on_map(self, msg):
        try:
            grid=verify_map(msg, self.binding)
            if self.keepout is not None:
                grid=self.keepout.mask(grid)
                planning=OccupancyGrid(header=msg.header,info=msg.info,data=grid['data'])
                self.planning_map_pub.publish(planning)
            fault = None
        except Exception as exc:
            fault = 'map_binding_failed:' + str(exc)
        with self.lock:
            self.map_fault = fault
            if not fault:self.map_grid=grid
            if fault:
                self.gate.stop(fault)

    def monitor_health(self):
        master = rosgraph.Master(rospy.get_name())
        while not self.stopping.is_set() and not rospy.is_shutdown():
            try:
                pubs, subs, _ = master.getSystemState()
                transform = self.buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
                age = rospy.Time.now().to_sec() - transform.header.stamp.to_sec()
                fault = None if -1.1 <= age <= .5 else 'stale_map_TF'
                with self.lock:
                    self.graph = (dict(pubs), dict(subs))
                    self.graph_at = self.tf_at = time.monotonic()
                    self.tf_fault = fault
                    p=transform.transform.translation
                    q=transform.transform.rotation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    self.map_xy=(p.x,p.y)
                    self.map_pose=(p.x,p.y,yaw)
            except Exception:
                pass  # The output thread stops on monitor expiry even if this thread blocks.
            self.stopping.wait(.1)

    def health(self, require_still=False, require_idle=False):
        now, ros_now = time.monotonic(), rospy.Time.now().to_sec()
        if self.graph is None or now - self.graph_at > .7:
            return 'graph_check_expired'
        pubs, subs = self.graph
        others = [n for n in pubs.get('/cmd_vel', []) if n != rospy.get_name()]
        if others:
            return 'other_cmd_vel_publishers:' + ','.join(others)
        if '/spark_base_server' not in subs.get('/cmd_vel', []):
            return 'base_driver_not_subscribed'
        if not self.path_control:
            if pubs.get('/suat/navigation/raw_cmd_vel', []) != ['/move_base']:
                return 'raw_navigation_owner_mismatch'
            if pubs.get('/suat/navigation/smoothed_cmd_vel', []) != ['/suat_navigation_nodelet_manager']:
                return 'smoother_owner_mismatch'
        if pubs.get('/map', []) != ['/map_server'] or self.map_fault:
            return self.map_fault or 'map_owner_mismatch'
        if self.keepout is not None and (pubs.get(PLANNING_MAP_TOPIC,[]) != [rospy.get_name()]
                                         or '/move_base' not in subs.get(PLANNING_MAP_TOPIC,[])):
            return 'central_pickup_keepout_map_not_connected'
        clients_fault = navigation_clients_fault(pubs,self.owner)
        if clients_fault:
            return clients_fault
        for name, item, frame in (('scan', self.scan, 'laser_frame'), ('odom', self.odom, 'odom')):
            if item is None or now-item[0] > .4 or not -.1 <= ros_now-item[1] <= .5 or item[2].lstrip('/') != frame:
                return 'stale_or_invalid_' + name
        if not all(math.isfinite(v) for v in self.odom[3:]):
            return 'invalid_odometry_velocity'
        if abs(self.odom[3]) > self.gate.MAX_V+.04 or abs(self.odom[4]) > self.gate.MAX_W+.15:
            return 'odometry_exceeds_configured_speed'
        if self.tf_fault or now-self.tf_at > .5:
            return self.tf_fault or 'TF_check_expired'
        if self.goals is None or now-self.goals[0] > 1.0:
            return 'move_base_status_expired'
        if (require_idle or self.path_control) and self.goals[1]:
            return 'existing_active_goal'
        if require_still and (self.still_since is None or now-self.still_since < .5):
            return 'base_not_stationary'
        return None

    def inputs_clear(self):
        # A fresh stack has no previous command to drain. After its first lease,
        # retain independently observed raw/smoothed zeros through the handoff.
        if self.path_control:
            return True
        if not self.has_armed:
            return all(values is None or all(abs(value)<=1e-8 for value in values)
                       for values in self.input_samples.values())
        return all(values is not None and all(abs(value)<=1e-8 for value in values)
                   for values in self.input_samples.values())

    def owner_request(self, request):
        return (getattr(request,'_connection_header',{}) or {}).get('callerid')==self.owner

    def arm(self, request):
        with self.lock:
            was_disarmed = self.gate.state == 'DISARMED'
            try:
                if self.path_control or not self.owner_request(request):
                    raise ValueError('external velocity input is disabled for this navigation owner/profile')
                if not self.inputs_clear():raise ValueError('previous navigation inputs are not zero')
                token = self.gate.arm(time.monotonic(), self.health(True, True))
                self.start_odom_pose = self.odom[5:8]
                self.previous_odom_pose=self.start_odom_pose
                self.travel=0.
                self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, latch=False)
                self.has_armed=True
                return TriggerResponse(True, json.dumps({'token':token,'linear_cap_mps':self.gate.lease_v,
                                                        'angular_cap_radps':self.gate.lease_w}))
            except Exception as exc:
                if was_disarmed and self.gate.state == 'ARMED':
                    self.gate.stop('output_publisher_creation_failed')
                return TriggerResponse(False, str(exc))

    def stop(self, _):
        with self.lock:
            self.gate.stop('explicit_stop')
            if self.pub is not None:
                self.pub.publish(Twist())
        return TriggerResponse(True, 'Output stopped and latched; only the owning client cancels its goal.')

    def egress_scan_pose(self):
        if self.scan is None or not self.odom_history:
            raise ValueError('起步扫描或里程计尚未收到')
        stamp=self.scan[1]
        history=[o for o in self.odom_history if o[2].lstrip('/')=='odom' and math.isfinite(o[1])]
        for left,right in zip(history,history[1:]):
            if left[1]<=stamp<=right[1] and 0<right[1]-left[1]<=.20:
                t=(stamp-left[1])/(right[1]-left[1])
                yaw_delta=math.atan2(math.sin(right[7]-left[7]),math.cos(right[7]-left[7]))
                self.odom_at_scan=(right[0],stamp,'odom',
                    left[3]+t*(right[3]-left[3]),left[4]+t*(right[4]-left[4]),
                    left[5]+t*(right[5]-left[5]),left[6]+t*(right[6]-left[6]),left[7]+t*yaw_delta)
                return self.odom_at_scan[5:8]
        nearest=min(history,key=lambda o:abs(o[1]-stamp)) if history else None
        if nearest is None or abs(nearest[1]-stamp)>MAX_EGRESS_SCAN_ODOM_SKEW_S:
            raise ValueError('起步扫描与里程计时间未对齐: skew='+str(None if nearest is None else stamp-nearest[1]))
        self.odom_at_scan=nearest
        return nearest[5:8]

    def on_local_map(self,msg):
        q=msg.info.origin.orientation
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if msg.header.frame_id.lstrip('/') not in ('map','odom') or abs(angle)>1e-5:
            return
        with self.lock:
            self.local_frame=msg.header.frame_id.lstrip('/')
            self.local_grid={'resolution':msg.info.resolution,'width':msg.info.width,'height':msg.info.height,
                'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
                'data':[100 if v==100 else -1 if v<0 else 0 for v in msg.data]}
            self.local_grid_at=time.monotonic()

    def on_local_update(self,msg):
        with self.lock:
            g=self.local_grid
            if g is None or msg.header.frame_id.lstrip('/')!=self.local_frame:return
            if msg.x+msg.width>g['width'] or msg.y+msg.height>g['height']:return
            for j in range(msg.height):
                for i in range(msg.width):
                    v=msg.data[j*msg.width+i]
                    g['data'][(msg.y+j)*g['width']+msg.x+i]=100 if v==100 else -1 if v<0 else 0
            self.local_grid_at=time.monotonic()

    def begin_a_egress(self, request):
        """Recover initial wall contact along a live recorded-goal path prefix."""
        with self.lock:
            started_here=False
            try:
                if self.path_control or self.mode!='field' or not self.owner_request(request):
                    raise ValueError('only the field owner may start A departure')
                if not self.inputs_clear():raise ValueError('previous navigation inputs are not zero')
                fault=self.health(True,True)
                if fault:raise ValueError(fault)
                if self.map_grid is None or self.scan_msg is None:raise ValueError('map or scan unavailable')
                transform=self.buffer.lookup_transform('map','base_footprint',rospy.Time(0)).transform
                q=transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                pose=[transform.translation.x,transform.translation.y,yaw]
                prefix=rospy.get_param('~path_prefix_target',None)
                radius=float(rospy.get_param('/move_base/global_costmap/robot_radius',.20))+float(
                    rospy.get_param('/move_base/global_costmap/footprint_padding',.02))
                if prefix is not None:
                    if len(prefix)!=3 or not all(math.isfinite(v) for v in prefix):
                        raise ValueError('path prefix requires an explicit finite target')
                    if math.dist(pose[:2],prefix[:2])>.65:
                        raise ValueError('path prefix exceeds 65cm')
                    planned=contact_path(self.map_grid,pose,[prefix[:2]],radius)
                    if not planned['escaped_initial_overlap']:
                        raise ValueError('prefix does not reach a clear handoff footprint')
                    if self.local_grid is None or time.monotonic()-self.local_grid_at>1.5:
                        raise ValueError('live local costmap missing for contact recovery')
                    planned.update(start=pose,end=prefix,execution_mode='AUTOMATIC_GOAL_PATH_PREFIX')
                else:
                    anchor=yaml.safe_load((self.workspace_root/'data/waypoints/start_pose_A.yaml').read_text())['pose']
                    anchor_check(pose,anchor)
                    planned=map_departure(self.map_grid,pose,radius)
                laser=self.buffer.lookup_transform('base_footprint','laser_frame',rospy.Time(0)).transform
                q=laser.rotation
                laser_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                laser_pose=(laser.translation.x,laser.translation.y,laser_yaw)
                checked=scan_departure(self.scan_msg,radius,DISTANCE,laser_pose,report_block=prefix is not None)
                reference_pose=self.egress_scan_pose()
                token=self.gate.arm(time.monotonic(),None)
                started_here=True
                self.start_odom_pose=self.odom[5:8]
                self.previous_odom_pose=self.start_odom_pose
                self.travel=0.
                egress_id=uuid.uuid4().hex
                self.egress={'id':egress_id,'status':'WAIT_HEARTBEAT','started':time.monotonic(),
                             'origin':self.start_odom_pose,'distance':0.,'radius':radius,
                             'laser_pose':laser_pose,'rear_points':checked['rear_points'],
                             'rear_reference_pose':reference_pose,'contact_walls':checked['contact_walls'],'error':None,
                             'echo_hold':None,'validated_echo_keys':set(),'forward_pause_count':0,'rear_pause_count':0,
                             'last_forward_block':None,'last_rear_block':None,
                             'last_echo_coverage':None,
                             'nonzero_velocity_published':False,
                             'command_v':0.,'command_at':time.monotonic()}
                if prefix is not None:
                    ox,oy,oa=self.start_odom_pose
                    a=oa-pose[2]; c,s=math.cos(a),math.sin(a)
                    dx,dy=prefix[0]-pose[0],prefix[1]-pose[1]
                    self.egress['target_odom']=[ox+c*dx-s*dy,oy+s*dx+c*dy]
                    self.egress['target_map']=list(prefix)
                    self.egress['command_w']=0.
                    self.egress['map_origin']=pose
                    self.egress['local_frame']=self.local_frame
                    g=self.local_grid
                    ref=set() # Only the original saved-map wall component is exempt.
                    # The obstacle layer clears the robot's footprint, including
                    # original wall cells. Preserve those walls from the saved
                    # map so their reappearance is not a new false obstruction.
                    cached={};contact_path(self.map_grid,pose,[prefix[:2]],radius,_cache=cached)
                    mg=self.map_grid;mr=mg['resolution'];mx,my=mg['origin']
                    for i,j in cached['walls']:
                        px,py=mx+(i+.5)*mr,my+(j+.5)*mr
                        if self.local_frame=='odom':
                            px,py=ox+c*(px-pose[0])-s*(py-pose[1]),oy+s*(px-pose[0])+c*(py-pose[1])
                        ref.add((math.floor((px-g['origin'][0])/g['resolution']),
                                 math.floor((py-g['origin'][1])/g['resolution'])))
                    self.egress['wall_reference']=(*g['origin'],g['resolution'],ref)
                    origin=pose if self.local_frame=='map' else self.start_odom_pose
                    target=prefix if self.local_frame=='map' else self.egress['target_odom']
                    contact_path(g,origin,[target],radius,self.egress['wall_reference'],static_contact=True)
                self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1,latch=False)
                # Internal departure does not create upstream planner commands;
                # preserve whether an earlier external lease needs draining.
                return TriggerResponse(True,json.dumps({'token':token,'egress_id':egress_id,'plan':planned}))
            except Exception as exc:
                if started_here and self.gate.state=='ARMED':self.gate.stop('a_egress_start_failed')
                if started_here and self.pub is not None:
                    self.pub.publish(Twist());self.pub.unregister();self.pub=None
                return TriggerResponse(False,str(exc))

    def follow_path(self, request):
        with self.lock:
            try:
                if not self.path_control or not self.owner_request(request):
                    raise ValueError('only the field path owner can start a route')
                fault=self.health(True,True)
                if fault:
                    raise ValueError(fault)
                if self.gate.state!='DISARMED':
                    raise ValueError('previous route has not released control')
                spec=rospy.get_param('~route_request')
                points,goal=spec['points'],spec['goal']
                xy,angle=spec['xy_tolerance'],spec['yaw_tolerance']
                if (not 1<=len(points)<=2000 or len(goal)!=3
                        or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points)
                        or not all(math.isfinite(v) for v in goal)
                        or not .005<=xy<=.08 or not .04<=angle<=.15
                        or math.dist(points[-1],goal[:2])>1e-5
                        or math.dist(points[0],self.map_pose[:2])>.035
                        or sum(math.dist(a,b) for a,b in zip(points,points[1:]))>8.):
                    raise ValueError('invalid or stale verified navigation path')
                now=time.monotonic()
                if self.local_grid is None or now-self.local_grid_at>1.5:
                    raise ValueError('live local costmap unavailable')
                radius=float(rospy.get_param('/move_base/global_costmap/robot_radius',.20))+float(
                    rospy.get_param('/move_base/global_costmap/footprint_padding',.02))
                if not math.isfinite(radius) or radius<=0:
                    raise ValueError('invalid competition footprint')
                origin=self.map_pose
                odom=self.odom[5:8]
                contact_end=None
                walls={}
                reference=None
                if not goal_is_free(self.map_grid,*origin[:2],radius=radius):
                    contact_end=spec['contact_end']
                    if len(contact_end)!=2 or math.dist(origin[:2],contact_end)>.65:
                        raise ValueError('no bounded contact prefix')
                    proof=contact_path(self.map_grid,origin,[contact_end],radius,_cache=walls)
                    if not proof['escaped_initial_overlap']:
                        raise ValueError('path prefix does not clear original wall contact')
                    mg=self.map_grid
                    g=self.local_grid
                    ox,oy=g['origin']; r=g['resolution']
                    cells=set()
                    for i,j in walls['walls']:
                        point=(mg['origin'][0]+(i+.5)*mg['resolution'],
                               mg['origin'][1]+(j+.5)*mg['resolution'])
                        x,y=self.local_point(point,origin,odom)
                        cells.add((math.floor((x-ox)/r),math.floor((y-oy)/r)))
                    reference=(ox,oy,r,cells)
                    contact_path(g,self.local_point(origin,origin,odom),
                                 [self.local_point(contact_end,origin,odom)],radius,
                                 reference,static_contact=True)
                tracker=PathTracker(points,goal[2],xy,angle,self.speed,contact_end)
                token=self.gate.arm(now)
                try:
                    self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1,latch=False)
                except Exception:
                    self.gate.stop('output_publisher_creation_failed')
                    raise
                self.start_odom_pose=self.previous_odom_pose=odom
                self.travel=0.
                self.route={'id':uuid.uuid4().hex,'status':'ACTIVE','error':None,'at':now,
                            'tracker':tracker,'radius':radius,'origin':origin,'odom_origin':odom,
                            'walls':walls,'reference':reference,'local_frame':self.local_frame,
                            'contact':contact_end is not None,'moving':False,'peak_mps':0.,
                            'reversals':0,'last_turn':0}
                return TriggerResponse(True,json.dumps({'token':token,'route_id':self.route['id']}))
            except Exception as exc:
                return TriggerResponse(False,str(exc))

    def local_point(self, point, map_pose, odom_pose):
        if self.local_frame=='map':
            return tuple(point[:2])
        dx,dy=point[0]-map_pose[0],point[1]-map_pose[1]
        rotation=odom_pose[2]-map_pose[2]
        c,s=math.cos(rotation),math.sin(rotation)
        return (odom_pose[0]+c*dx-s*dy,odom_pose[1]+s*dx+c*dy)

    def terminal_clearance_wait(self, route, block, now):
        tracker=route['tracker']
        hold=route.get('terminal_recheck')
        if block is not None:
            route['blocked_footprint']=block
            if (not tracker.position_reached or block['source']!='live_costmap'
                    or block['value']!=100 or (hold is not None and hold['resumed'])):
                raise ValueError('path footprint blocked in '+block['source'])
            if hold is None:
                hold=route['terminal_recheck']={'started':now,'resumed':False}
            hold.update(clear_updates=0,grid_at=self.local_grid_at)
        if hold is None or hold['resumed']:
            return False
        if now-hold['started']>=TERMINAL_RECHECK_TIMEOUT_S:
            raise ValueError('terminal footprint did not clear while stopped')
        if (block is None and self.local_grid_at>hold['grid_at']
                and self.still_since is not None and self.local_grid_at>=self.still_since
                and self.local_grid_at>hold['started']):
            hold['grid_at']=self.local_grid_at
            hold['clear_updates']+=1
            if hold['clear_updates']>=2:
                hold['resumed']=True
        tracker.v=tracker.w=0.
        return True

    def preview_terminal_speed(self, pose, odom, v, w, radius):
        """Slow the commanded arc while there is still room to brake."""
        previous=pose[:2]
        for point in braking_arc(pose,v,w,self.egress_accel,latency=2*BRAKING_LATENCY_S):
            local=self.local_point(point,pose,odom)
            if (not goal_is_free(self.local_grid,*local,radius=radius)
                    or not goal_is_free(self.map_grid,*point,radius=radius)
                    or (self.keepout is not None and not self.keepout.segment_clear(previous,point,radius))):
                return min(v,braking_speed(math.dist(pose[:2],previous),self.egress_accel))
            previous=point
        return v

    def route_output(self, now):
        route=self.route
        if not route or route['status']!='ACTIVE':
            return 0.,0.
        try:
            fault=self.health()
            if self.gate.state!='ARMED' or fault:
                raise ValueError(fault or self.gate.reason)
            if self.gate.sequence<0:
                return self.gate.departure_output(now,0.)
            if (self.local_grid is None or self.local_frame!=route['local_frame']
                    or now-self.local_grid_at>1.5):
                raise ValueError('live local costmap expired')
            pose=self.map_pose
            odom=self.odom[5:8]
            radius=route['radius']
            local=self.local_point(pose,pose,odom)
            route['contact'] = route['contact'] and (
                not goal_is_free(self.map_grid,*pose[:2],radius=radius)
                or not goal_is_free(self.local_grid,*local,radius=radius))
            tracker=route['tracker']
            measured_v,measured_w=self.odom[3:5]
            dt=min(.10,now-route['at'])
            route['at']=now
            v,w=tracker.command(pose,measured_w,dt,route['contact'],measured_v=measured_v)
            if v>0. and tracker.near_goal and not route['contact']:
                v=tracker.v=self.preview_terminal_speed(pose,odom,v,w,radius)
            # Check the commanded and measured braking arcs against both maps.
            # The original contact reference expires as soon as the body clears.
            block=None
            for arc_source,speed,turn in (('commanded',v,w),('measured',measured_v,measured_w)):
                arc=braking_arc(pose,speed,turn,self.egress_accel)
                local_arc=[self.local_point(point,pose,odom) for point in arc]
                if route['contact']:
                    contact_path(self.map_grid,route['origin'],[pose,*arc],radius,_cache=route['walls'])
                    contact_path(self.local_grid,self.local_point(route['origin'],route['origin'],route['odom_origin']),
                                 [local,*local_arc],radius,route['reference'],static_contact=True,
                                 _cache=route.setdefault('local_walls',{}))
                else:
                    maps=(('saved_map',self.map_grid,[pose,*arc]),('live_costmap',self.local_grid,[local,*local_arc]))
                    for source,grid,path in maps:
                        for point in path:
                            cell=blocked_footprint_cell(grid,*point[:2],radius)
                            if cell is not None:
                                cell.update(source=source,arc=arc_source,point_xy=list(point[:2]),
                                            pose_map=list(pose),linear_mps=speed,angular_radps=turn)
                                if (block is None or source=='saved_map'
                                        or (block['source']!='saved_map' and cell['value']!=100)):
                                    block=cell
                                break
                if self.keepout is not None and any(not self.keepout.segment_clear(a,b,radius)
                        for a,b in zip([pose,*arc],arc)):
                    raise ValueError('planned motion enters central pickup keepout')
            if self.terminal_clearance_wait(route,block,now):
                return self.gate.departure_output(now,0.)
            if tracker.arrived(pose,measured_v,measured_w):
                route['status']='COMPLETED'
                self.gate.stop('path_arrived')
                return 0.,0.
            v,w=self.gate.departure_output(now,v,angular=w)
            if self.gate.state=='ARMED':
                self.gate.reason='field_path'
            route['moving']|=abs(v)>0. or abs(w)>0.
            route['peak_mps']=max(route['peak_mps'],abs(measured_v))
            if abs(measured_w)>.05:
                sign=1 if measured_w>0. else -1
                route['reversals']+=int(route['last_turn']==-sign)
                route['last_turn']=sign
            return v,w
        except Exception as exc:
            route.update(status='FAILED',error=str(exc))
            self.gate.stop('field_path_blocked')
            return 0.,0.

    def pause_for_near_echo(self, state, blocks, now, scan_pose):
        scan_stamp = self.scan[1]
        for block in blocks:
            state['last_' + block.side + '_block'] = dict(
                block.point, beam_count=block.beam_count, scan_stamp_s=scan_stamp,
                forward_distance_m=state['distance'], odom_pose=list(self.odom[5:8]))
        if any(block.beam_count > 1 for block in blocks):
            raise ValueError('起步扫掠同帧检测到多个阻挡点')
        for block in blocks:
            if block.point['range_m'] > SHORT_ECHO_M:
                raise block

        hold = state['echo_hold']
        if hold is None:
            hold = {'since': now, 'last_scan_stamp': -float('inf'), 'clear_scans': 0,
                    'points_odom': set(), 'sides': set(), 'beam_keys': set()}
            state['echo_hold'] = hold
        for block in blocks:
            hold['beam_keys'].add(round(block.point['bearing_rad'],2))
            if block.side not in hold['sides']:
                hold['sides'].add(block.side)
                state[block.side + '_pause_count'] += 1
            x, y = block.point['base_xy_m']
            ox, oy, yaw = scan_pose
            c, s = math.cos(yaw), math.sin(yaw)
            hold['points_odom'].add((ox + c*x - s*y, oy + s*x + c*y))
        state['last_echo_coverage'] = {'clear_scans': hold['clear_scans'], 'max_gap_m': None}
        state.update(command_v=0., command_at=now)

    def echo_hold_cleared(self, state, now, scan_pose, distance):
        hold = state['echo_hold']
        if hold is None:
            return True

        state.update(command_v=0., command_at=now)
        scan_stamp = self.scan[1]
        if scan_stamp <= hold['last_scan_stamp']:
            return False

        hold['last_scan_stamp'] = scan_stamp
        stationary_scan = (
            self.scan[0] - hold['since'] >= ECHO_RECHECK_SETTLE_S
            and self.still_since is not None and self.still_since <= self.scan[0]
            and abs(self.odom_at_scan[3]) <= .005 and abs(self.odom_at_scan[4]) <= .02
            and abs(self.odom[3]) <= .005 and abs(self.odom[4]) <= .02
        )
        gap = None
        if stationary_scan:
            ox, oy, yaw = scan_pose
            c, s = math.cos(yaw), math.sin(yaw)
            laser_x, laser_y, laser_yaw = state['laser_pose']
            bearings = []
            for px, py in hold['points_odom']:
                dx, dy = px - ox, py - oy
                lx, ly = c*dx + s*dy - laser_x, -s*dx + c*dy - laser_y
                bearings.append(math.atan2(ly, lx) - laser_yaw)
            gap = sweep_observation_gap(self.scan_msg, bearings, state['radius'],
                                        distance, state['laser_pose'],
                                        initial_walls=state['contact_walls'],
                                        reference_pose=state['rear_reference_pose'], current_pose=scan_pose)
        if gap is not None:
            hold['clear_scans'] += 1
            if hold['clear_scans'] >= ECHO_RECHECK_CLEAR_SCANS:
                state['validated_echo_keys'].update(hold['beam_keys'])
                state['echo_hold'] = None
        else:
            hold['clear_scans'] = 0
        state['last_echo_coverage'] = {'clear_scans': hold['clear_scans'], 'max_gap_m': gap}
        # Even the scan that completes confirmation receives a zero command.
        return False

    def update_a_egress(self, now):
        state=self.egress
        if not state or state['status'] not in ('WAIT_HEARTBEAT','ACTIVE'):
            return
        if self.gate.state!='ARMED':
            state.update(status='FAILED',error=self.gate.reason);return
        if now-state['started']>25.:
            state.update(status='FAILED',error='A起步超时');self.gate.stop('a_egress_timeout');return
        if state['status']=='WAIT_HEARTBEAT':return
        try:
            x0,y0,yaw0=state['origin'];x,y,yaw=self.odom[5:8]
            if state.get('target_odom') is not None:
                tx,ty=state['target_odom']; dx,dy=tx-x,ty-y
                remaining=math.hypot(dx,dy)
                state['distance']=math.hypot(x-x0,y-y0)
                if remaining<=.020:
                    state['status']='COMPLETED';state['command_w']=0.
                    self.gate.stop('goal_path_prefix_completed');return
                if self.local_grid is None or now-self.local_grid_at>1.5 or self.local_frame!=state['local_frame']:
                    raise ValueError('live local obstacle map expired during contact recovery')
                if self.local_frame=='odom':
                    origin=state['origin'];current=[x,y];target=[tx,ty]
                else:
                    tr=self.buffer.lookup_transform('map','base_footprint',rospy.Time(0)).transform
                    origin=state['map_origin'];current=[tr.translation.x,tr.translation.y];target=state['target_map'][:2]
                contact_path(self.local_grid,origin,[current,target],state['radius'],state['wall_reference'],static_contact=True)
                tr=self.buffer.lookup_transform('map','base_footprint',rospy.Time(0)).transform
                contact_path(self.map_grid,state['map_origin'],
                    [[tr.translation.x,tr.translation.y],state['target_map'][:2]],state['radius'])
                error=math.atan2(math.sin(math.atan2(dy,dx)-yaw),math.cos(math.atan2(dy,dx)-yaw))
                turn_limit = self.gate.MAX_W
                state['command_w']=max(-turn_limit,min(turn_limit,1.2*error))
                desired=0. if abs(error)>.15 else min(self.gate.MAX_V,.8*remaining)
                dt=max(0.,min(.1,now-state['command_at']))
                state['command_v']=min(desired,state['command_v']+self.egress_accel*dt)
                state['command_at']=now
                return
            dx,dy=x-x0,y-y0
            forward=dx*math.cos(yaw0)+dy*math.sin(yaw0)
            sideways=-dx*math.sin(yaw0)+dy*math.cos(yaw0)
            turn=abs(math.atan2(math.sin(yaw-yaw0),math.cos(yaw-yaw0)))
            state['distance']=forward
            if forward<-.01 or abs(sideways)>.04 or turn>.10:
                raise ValueError('A起步未保持向前直行')
            if forward>=DISTANCE:
                state['status']='COMPLETED';self.gate.stop('a_egress_completed');return
            hold = state['echo_hold']
            if hold is not None and now - hold['since'] >= ECHO_RECHECK_TIMEOUT_S:
                raise ValueError('起步扫掠近点复核超时')
            remaining = DISTANCE - max(0., forward)
            scan_pose = self.egress_scan_pose()
            checked = scan_departure(self.scan_msg, state['radius'], remaining,
                                     state['laser_pose'], report_block=True,
                                     initial_walls=state['contact_walls'],
                                     reference_pose=state['rear_reference_pose'], current_pose=scan_pose)
            blocks = []
            blocks.extend(checked['forward_blocks'])
            try:
                check_rear_clearance(state['rear_points'], checked['rear_points'],
                                     state['rear_reference_pose'], scan_pose,
                                     state['radius'], rear_beams=checked['rear_beams'])
            except SweepBlocked as block:
                blocks.extend(getattr(block,'blocks',[block]))
            # A confirmed isolated echo is reusable only while fresh adjacent rays
            # still bound its blind width. Solid/new obstacles are never masked.
            blocks=[block for block in blocks if not (
                block.beam_count==1 and block.point['range_m']<=SHORT_ECHO_M
                and round(block.point['bearing_rad'],2) in state['validated_echo_keys']
                and sweep_observation_gap(self.scan_msg,[block.point['bearing_rad']],
                    state['radius'],remaining,state['laser_pose'],
                    initial_walls=state['contact_walls'],
                    reference_pose=state['rear_reference_pose'],current_pose=scan_pose) is not None)]
            if blocks:
                self.pause_for_near_echo(state, blocks, now, scan_pose)
            if not self.echo_hold_cleared(state, now, scan_pose, remaining):
                return
            dt=max(0.,min(.1,now-state['command_at']))
            state['command_v']=min(self.gate.MAX_V,.8*remaining,state['command_v']+self.egress_accel*dt)
            state['command_at']=now
        except Exception as exc:
            state.update(status='FAILED',error=str(exc));self.gate.stop('a_egress_blocked')

    def release(self, request):
        with self.lock:
            try:
                if not self.owner_request(request):raise ValueError('only configured navigation owner may release')
                fault = self.health(True, True)
                if fault:
                    raise ValueError(fault)
                if not self.inputs_clear():raise ValueError('previous navigation inputs are not zero')
                self.gate.release(True, True)
                if self.pub is not None:
                    self.pub.publish(Twist())
                    self.pub.unregister()
                    self.pub = None
                return TriggerResponse(True, 'Released; a new explicit arm is required.')
            except Exception as exc:
                return TriggerResponse(False, str(exc))

    def loop(self):
        count = 0
        while not self.stopping.wait(.05) and not rospy.is_shutdown():
            with self.lock:
                if self.path_control:
                    v,w=self.route_output(time.monotonic())
                else:
                    self.update_a_egress(time.monotonic())
                    if self.egress and self.egress['status'] in ('WAIT_HEARTBEAT','ACTIVE'):
                        v,w=self.gate.departure_output(time.monotonic(),self.egress['command_v'],self.health(),angular=self.egress.get('command_w',0.))
                    else:
                        v,w=self.gate.output(time.monotonic(), self.health())
                if not self.path_control and self.keepout is not None and self.gate.state=='ARMED' and self.map_xy is not None:
                    # The swept center stays within this radius for the next
                    # output/TF cycle and the configured braking distance.
                    speed=max(abs(v),abs(self.odom[3]))
                    reach=speed*(.15+time.monotonic()-self.tf_at)+speed*speed/(2*self.egress_accel)
                    if (not all(math.isfinite(p) for p in self.map_xy)
                            or self.keepout.distance(self.map_xy)<=self.keepout_radius+reach):
                        self.gate.stop('central_pickup_keepout_swept_footprint')
                        v=w=0.
                if self.pub is not None:
                    msg = Twist()
                    msg.linear.x, msg.angular.z = v, w
                    self.pub.publish(msg)
                    if v > 0. and self.egress and self.egress['status'] == 'ACTIVE':
                        self.egress['nonzero_velocity_published'] = True
                if count % 4 == 0:
                    self.state_pub.publish(String(data=json.dumps({
                        'state': self.gate.state, 'reason': self.gate.reason,
                        'profile': self.profile,
                        'mode':self.mode,'speed_profile':self.speed_name,'travel_m':self.travel,
                        'owner':self.owner,'inputs_clear':self.inputs_clear(),
                        'workspace_root':str(Path(__file__).resolve().parents[5]),
                        'mission_keepout':self.keepout is not None,
                        'keepout_file':self.keepout_file,
                        'active_goal':None if self.goals is None else self.goals[1],
                        'health': self.health(), 'route': None if self.route is None else {
                            key:self.route.get(key) for key in ('id','status','error','moving','peak_mps','reversals',
                                'blocked_footprint','terminal_recheck')},
                        'egress':None if self.egress is None else {
                            'id':self.egress['id'], 'status':self.egress['status'], 'distance_m':self.egress['distance'],
                             'error':self.egress['error'],
                             'path_prefix_target_map':self.egress.get('target_map'),
                             'forward_waiting':'forward' in (self.egress['echo_hold'] or {}).get('sides', ()),
                             'rear_waiting':'rear' in (self.egress['echo_hold'] or {}).get('sides', ()),
                             'forward_pause_count':self.egress['forward_pause_count'],
                             'rear_pause_count':self.egress['rear_pause_count'],
                             'last_forward_block':self.egress['last_forward_block'],
                             'last_rear_block':self.egress['last_rear_block'],
                             'last_echo_coverage':self.egress['last_echo_coverage'],
                             'nonzero_velocity_published':self.egress['nonzero_velocity_published']},
                         'linear_cap_mps': self.gate.MAX_V,
                        'angular_cap_radps': self.gate.MAX_W,
                        'lease_linear_cap_mps':self.gate.lease_v,
                        'lease_angular_cap_radps':self.gate.lease_w})))
            count += 1

    def shutdown(self):
        self.stopping.set()
        with self.lock:
            self.gate.stop('shutdown')
            if self.pub is not None:
                self.pub.publish(Twist())
                time.sleep(.1)
                self.pub.unregister()


if __name__ == '__main__':
    rospy.init_node('suat_navigation_gate')
    Node()
    rospy.spin()
