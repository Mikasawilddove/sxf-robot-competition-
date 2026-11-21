#!/usr/bin/env python3
"""Map-bound long-distance points and routes. Planning is the default."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from nav_msgs.msg import OccupancyGrid,Odometry
from nav_msgs.srv import GetPlan
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from suat_mission.navigation.navigation_map import load_binding,verify_map
from suat_mission.navigation.navigation_gate import goal_check_radius, goal_is_free
from suat_mission.navigation.navigation_budget import NavigationBudget
from suat_mission.navigation.navigation_profiles import (
    CONTROLLER_HZ, configured_global_planner, LOCAL_PLANNER_PLUGIN, PLANNER_NAMESPACE,
    speed_profile,leg_timeout,path_travel_fault,planner_speed_limits,
    local_planner_settings, planner_profile_matches, settings_match,
)
from suat_mission.navigation.navigation_test_geometry import arrival_error,arrival_confirmed
from suat_mission.navigation.field_routes import validate_route,StaticPlanner
from suat_mission.navigation.navigation_executor import NavigationExecutor
from suat_mission.contracts import ActionContext
from suat_mission.navigation.corner_egress import contact_path,contact_handoffs
from suat_mission.navigation.pickup_keepout import KEEP_OUT_FILE, PLANNING_MAP_TOPIC, PickupKeepout, is_pickup_station


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class NavigationPlanError(ValueError):
    """Keep the rejected planner output available to the normal run recorder."""
    STATION_ERRORS = frozenset((
        'goal body footprint is blocked on selected map', 'no complete live plan',
        'one leg exceeds 8 m', 'pickup endpoint lacks stopping clearance from the brick array',
        'live planner returned a partial path', 'live path crosses central pickup keepout',
        'live path body overlaps occupied or unknown saved map cells',
    ))

    def __init__(self, message, plan):
        super().__init__(message)
        self.plan = plan


class Navigator:
    A_DEPARTURE_LIMIT_S = NavigationBudget().departure

    def __init__(self,root,speed,owner=None,health_check=None,planner_mode=None,keepout_file=KEEP_OUT_FILE,
                 *,comparison=False):
        from geometry_msgs.msg import Twist

        self.owner=owner or rospy.get_name()
        self.health_check=health_check
        self.planner_mode=planner_mode
        self.global_planner=None if planner_mode is None else configured_global_planner(
            root, 'rollout' if comparison else planner_mode)
        self.binding=load_binding(root);self.speed=speed
        self.keepout_file=keepout_file
        self.keepout=PickupKeepout.load(root,self.binding,keepout_file) if self.owner=='/suat_mission' else None
        footprint=rospy.get_param('/move_base/global_costmap/footprint','[]')
        if footprint not in (None,'','[]',[]):raise ValueError('field validator requires a circular footprint')
        self.radius=float(rospy.get_param('/move_base/global_costmap/robot_radius',.20))+float(rospy.get_param('/move_base/global_costmap/footprint_padding',.02))
        if not math.isfinite(self.radius) or self.radius<=0:raise ValueError('invalid configured body radius')
        self.buffer=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buffer)
        self.lock=threading.RLock();self.odom=None;self.state=None;self.state_at=0.
        self.output_stats=None
        self.subs=[rospy.Subscriber('/odom',Odometry,self.on_odom,queue_size=1),
                   rospy.Subscriber('/cmd_vel',Twist,self.on_output,queue_size=20),
                   rospy.Subscriber('/suat/navigation/state',String,self.on_state,queue_size=1)]
        self.nav=None

    def on_odom(self,msg):
        with self.lock:self.odom=(time.monotonic(),msg)

    def on_output(self,msg):
        now=time.monotonic()
        with self.lock:
            if self.output_stats is not None:
                count,last,gap=self.output_stats
                self.output_stats=(count+1,now,max(gap,now-last))

    def last_pose(self):
        try:
            return self.pose()
        except (ValueError,rospy.ROSException,tf2_ros.TransformException):
            return None

    def on_state(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.state,self.state_at=value,time.monotonic()
        except ValueError:pass

    def pose(self):
        tr=self.buffer.lookup_transform('map','base_footprint',rospy.Time(0),rospy.Duration(2))
        if not -1.1<=(rospy.Time.now()-tr.header.stamp).to_sec()<=.5:
            raise ValueError('stale map pose')
        p,q=tr.transform.translation,tr.transform.rotation
        return [p.x,p.y,yaw(q)]

    def measured(self):
        with self.lock:
            if self.odom is None or time.monotonic()-self.odom[0]>.4:raise ValueError('odometry expired')
            at,msg=self.odom
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            return [p.x,p.y,yaw(q)],msg.twist.twist.linear.x,msg.twist.twist.angular.z,time.monotonic()-at

    def grid(self):
        topic=PLANNING_MAP_TOPIC if self.keepout is not None else '/map'
        return verify_map(rospy.wait_for_message(topic,OccupancyGrid,timeout=4),self.binding)

    @staticmethod
    def stamped(pose):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=rospy.Time.now()
        msg.pose.position.x,msg.pose.position.y=pose[:2]
        msg.pose.orientation.z=math.sin(pose[2]/2);msg.pose.orientation.w=math.cos(pose[2]/2)
        return msg

    def plan(self,start,goal,grid,*,pickup_approach=False,pickup_station=False):
        evidence={'start_pose':list(start),'goal_pose':list(goal),'frame_id':'map',
                  'body_radius_m':self.radius,'max_leg_length_m':8.,'planning_attempts':[],
                  'planner_policy':'shortest_valid_contact_handoff_fix03'}
        def reject(message):
            evidence['error']=message
            raise NavigationPlanError(message,evidence)
        if (len(start)!=3 or len(goal)!=3 or
                not all(math.isfinite(v) for v in list(start)+list(goal))):
            reject('invalid navigation pose')
        if self.keepout is not None and min(self.keepout.distance(start),self.keepout.distance(goal))<=self.radius:
            reject('start or goal body overlaps central pickup keepout')
        start_clear=goal_is_free(grid,*start[:2],radius=self.radius)
        goal_radius=goal_check_radius(self.radius)
        if pickup_approach or pickup_station:
            if self.keepout is None or not self.keepout.precise_pickup:
                reject('pickup endpoint requires the matching keepout profile')
            goal_radius=goal_check_radius(self.radius,precise_pickup=True)
            if self.keepout.distance(goal)<=goal_radius:
                reject('pickup endpoint lacks stopping clearance from the brick array')
        if pickup_approach:
            if not start_clear or math.dist(start[:2],goal[:2])>.101:
                reject('pickup approach requires a clear start and a correction within 10 cm')
            if (not StaticPlanner(grid,radius=self.radius).segment_free(start,goal)
                    or not self.keepout.segment_clear(start,goal,self.radius)):
                reject('direct pickup approach intersects the floor obstacles or brick clearance')
        if not goal_is_free(grid,*goal[:2],radius=goal_radius):
            reject('goal body footprint is blocked on selected map')
        checker=StaticPlanner(grid,radius=self.radius)
        def live_points(a,b):
            attempt={'planner_start':list(a),'planner_goal':list(b)}
            evidence['planning_attempts'].append(attempt)
            rospy.wait_for_service('/move_base/make_plan',timeout=3)
            path=rospy.ServiceProxy('/move_base/make_plan',GetPlan)(self.stamped(a),self.stamped(b),0.).plan
            attempt.update(planner_frame_id=path.header.frame_id,
                           pose_frame_ids=sorted({p.header.frame_id for p in path.poses}),
                           planner_point_count=len(path.poses))
            points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
            attempt['path_xy']=[[v if math.isfinite(v) else None for v in p] for p in points]
            if path.header.frame_id not in ('','map') or any(
                    p.header.frame_id not in ('','map') for p in path.poses):
                raise ValueError('live planner returned a path outside map frame')
            if not all(math.isfinite(v) for p in points for v in p):
                raise ValueError('live planner returned nonfinite path coordinates')
            attempt['planner_path_length_m']=sum(math.dist(p,q) for p,q in zip(points,points[1:]))
            if len(points)<2:
                raise ValueError('no complete live plan')
            if math.dist(points[0],a[:2])>.10 or math.dist(points[-1],b[:2])>.05:
                raise ValueError('live planner returned a partial path')
            return [tuple(a[:2])]+points+[tuple(b[:2])],attempt
        def checked(points,handoff,attempt):
            points=[tuple(start[:2])]+points+[tuple(goal[:2])]
            segments=[math.dist(a,b) for a,b in zip(points,points[1:])]
            length=sum(segments)
            attempt.update(path_xy=points,path_length_m=length,
                           largest_segment_m=max(segments,default=0.),
                           contact_handoff=handoff)
            if length>8:
                raise ValueError('one leg exceeds 8 m')
            if pickup_approach and length>min(.17,math.dist(start[:2],goal[:2])+.05):
                raise ValueError('pickup alignment requires a short local path')
            if self.keepout is not None and not all(self.keepout.segment_clear(a,b,self.radius)
                                                    for a,b in zip(points,points[1:])):
                raise ValueError('live path crosses central pickup keepout')
            body_clear=all(checker.segment_free(a,b) for a,b in zip(points,points[1:]))
            recovery=None
            if not body_clear:
                if start_clear:
                    raise ValueError('live path body overlaps occupied or unknown saved map cells')
                recovery=contact_path(grid,start,points,self.radius)
                if not recovery['initial_wall_overlap'] or not recovery['escaped_initial_overlap']:
                    raise ValueError('live goal path does not escape the initial wall overlap')
            attempt['accepted']=True
            return {'path_length_m':length,'path_xy':points,'timeout_s':leg_timeout(length,self.speed),
                    'saved_map_body_clear':body_clear,'initial_contact_recovery':recovery,
                    'contact_handoff':handoff}
        if not start_clear:
            options=contact_handoffs(grid,start,goal,self.radius)
            for _,candidate,_ in options[:6]:
                attempt=None
                try:
                    tail,attempt=live_points(candidate,goal)
                    if not all(checker.segment_free(a,b) for a,b in zip(tail,tail[1:])):
                        raise ValueError('handoff path body overlaps occupied or unknown saved map cells')
                    plan=checked(tail,candidate,attempt)
                    break
                except (ValueError,rospy.ROSException,rospy.ServiceException) as exc:
                    if attempt is None and evidence['planning_attempts']:
                        attempt=evidence['planning_attempts'][-1]
                    if attempt is not None:attempt['error']=str(exc)
            else:
                errors=[p.get('error') for p in evidence['planning_attempts']]
                reject('one leg exceeds 8 m' if 'one leg exceeds 8 m' in errors else
                       'no clear automatic departure connection to the recorded goal')
        else:
            try:
                if math.dist(start[:2],goal[:2])<=.05:
                    attempt={'planner_start':list(start),'planner_goal':list(goal)}
                    evidence['planning_attempts'].append(attempt)
                    points=[start[:2],goal[:2]]
                else:
                    points,attempt=live_points(start,goal)
                plan=checked(points,None,attempt)
            except (ValueError,rospy.ROSException,rospy.ServiceException) as exc:
                if evidence['planning_attempts']:
                    evidence['planning_attempts'][-1]['error']=str(exc)
                reject(str(exc))
        plan.update(start_pose=list(start),goal_pose=list(goal),
                    planner_policy=evidence['planner_policy'],
                    planning_attempts=evidence['planning_attempts'],pickup_approach=pickup_approach,
                    pickup_station=pickup_station)
        return plan

    def connect_control(self):
        profile=speed_profile(self.speed)
        for name in ('arm','stop','release'):rospy.wait_for_service('/suat_navigation_gate/'+name,timeout=3)
        until=time.monotonic()+3.
        while self.state is None and time.monotonic()<until and not rospy.is_shutdown():
            time.sleep(.05)
        with self.lock:
            if self.state is None or time.monotonic()-self.state_at>.7:raise ValueError('fresh output state missing')
            if self.state.get('owner')!=self.owner:
                raise ValueError('navigation output belongs to another owner')
            if self.keepout is not None and self.state.get('mission_keepout') is not True:
                raise ValueError('mission navigation requires the central pickup keepout')
            if self.keepout is not None and self.state.get('keepout_file',KEEP_OUT_FILE)!=self.keepout_file:
                raise ValueError('navigation gate is using a different pickup keepout')
            if self.owner=='/suat_mission' and not {'lease_linear_cap_mps','lease_angular_cap_radps'}<=self.state.keys():
                raise ValueError('mission navigation requires output lease limits')
            if (self.state.get('health') is not None or self.state.get('active_goal') is not False
                    or self.state.get('inputs_clear') is not True
                    or self.state.get('state') not in ('DISARMED','LATCHED')):
                raise ValueError('navigation gate is not healthy, idle and drained')
            if self.state.get('mode')!='field' or self.state.get('speed_profile')!=self.speed:
                raise ValueError('start field navigation with the same speed profile first')
            if self.state.get('linear_cap_mps')!=profile['linear'] or self.state.get('angular_cap_radps')!=profile['angular']:
                raise ValueError('output speed profile mismatch')
            if self.planner_mode=='path':
                if (self.state.get('profile')!='field_path'
                        or rospy.get_param('/move_base/base_global_planner',None)!=self.global_planner):
                    raise ValueError('fixed path controller or global planner mismatch')
                self.connected=True
                return
        # Detect partial speed edits before any lease can be opened.
        limits=planner_speed_limits(self.speed)
        live=rospy.get_param(PLANNER_NAMESPACE,{})
        if self.planner_mode is not None:
            actual_global=rospy.get_param('/move_base/base_global_planner',None)
            if actual_global!=self.global_planner:
                raise ValueError('global planner mismatch: expected %s, got %s' % (self.global_planner,actual_global))
            if (rospy.get_param('/move_base/base_local_planner',None)!=LOCAL_PLANNER_PLUGIN
                    or rospy.get_param('/move_base/controller_frequency',None)!=CONTROLLER_HZ
                    or not planner_profile_matches(live,self.planner_mode)):
                raise ValueError('active ROS navigation does not match the selected local planner profile')
        if not settings_match(live,limits):
            raise ValueError('planner limits differ from the selected speed profile')
        params={'/suat_navigation_smoother/speed_lim_v':profile['linear'],
                '/suat_navigation_smoother/speed_lim_w':profile['angular']}
        if any(abs(float(rospy.get_param(k,float('nan')))-v)>1e-8 or not math.isfinite(float(rospy.get_param(k,float('nan')))) for k,v in params.items()):
            raise ValueError('planner/smoother limits differ from selected speed')
        if self.nav is None:
            self.nav=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        if not self.nav.wait_for_server(rospy.Duration(3)):raise ValueError('navigation server unavailable')
        self.connected=True

    def depart_a(self, context=None):
        """Lease the existing output gate for the recorded-goal path prefix."""
        io=RosNavigationTransport(self)
        report={'passed':False,'motion_command_sent':False,'output_released':False}
        started=prepared=False
        at=io.now()
        end=context.deadline if context is not None else at+self.A_DEPARTURE_LIMIT_S+self.executor_stop_budget()
        try:
            if io.now()+self.executor_stop_budget()>=end:
                raise ValueError('insufficient original leg budget for contact recovery')
            io.ready()
            if not io.settle(min(end,io.now()+self.executor_stop_budget())) or not io.release():
                raise ValueError('departure output handoff is not stopped and released')
            prepared=True;io.prepare_heartbeat()
            rospy.wait_for_service('/suat_navigation_gate/begin_a_egress',timeout=3)
            if io.now()+self.executor_stop_budget()>=end:
                raise ValueError('departure deadline reached before the gate request')
            result=io.service('begin_a_egress')
            if not result.success:raise ValueError(result.message)
            response=json.loads(result.message)
            token=response['token'];egress_id=response['egress_id']
            report['plan']=response.get('plan')
            started=True;report['motion_command_sent']=True
            io.start_heartbeat(token)
            deadline = io.now() + self.A_DEPARTURE_LIMIT_S
            if context is not None:
                deadline = min(deadline, context.deadline-self.executor_stop_budget())
            while io.now()<deadline and not io.shutdown():
                if self.health_check:self.health_check()
                with self.lock:
                    state=dict(self.state or {});age=io.now()-self.state_at
                if age>.7:raise ValueError('departure gate state expired')
                egress=state.get('egress') or {}
                if egress.get('id')!=egress_id:
                    io.sleep(.05);continue
                report['distance_m']=egress.get('distance_m')
                for key in ('forward_pause_count', 'rear_pause_count', 'last_forward_block',
                            'last_rear_block', 'last_echo_coverage', 'nonzero_velocity_published'):
                    if key in egress:
                        report[key] = egress[key]
                if egress.get('status')=='COMPLETED':
                    break
                if egress.get('status')=='FAILED':
                    raise ValueError(egress.get('error') or 'departure gate stopped')
                io.sleep(.05)
            else:raise ValueError('departure deadline reached')
            report['passed']=True
        except (Exception,KeyboardInterrupt) as exc:
            report['error']=str(exc) or type(exc).__name__
        finally:
            if started:
                try:
                    report['stop_requested']=bool(io.stop())
                except Exception as exc:
                    report['stop_error']=str(exc)
            if prepared:
                try:io.stop_heartbeat()
                except Exception as exc:report['heartbeat_error']=str(exc)
            if started:
                try:
                    report['output_released']=bool(io.settle(min(end,io.now()+self.executor_stop_budget())) and io.release())
                except Exception as exc:
                    report['cleanup_error']=str(exc)
                report['passed']=bool(report['passed'] and report.get('stop_requested')
                                      and report['output_released'] and not report.get('heartbeat_error'))
        report.update(elapsed_s=io.now()-at,actual_pose=self.last_pose())
        return report

    @staticmethod
    def executor_stop_budget():
        return NavigationExecutor.STOP_BUDGET

    def can_try_next_station(self, report, context):
        if self.planner_mode=='path':
            return False
        if (report.get('passed') or not report.get('output_released')
                or time.monotonic()+self.executor_stop_budget()>=context.deadline
                or any(report.get(key) for key in ('stop_error','release_error','heartbeat_error'))):
            return False
        if report.get('candidate_unreachable'):
            return True
        error=report.get('error')
        if error in ('no navigation progress for 20 seconds','navigation deadline reached'):
            return True
        if report.get('action_terminal_state')==4 and report.get('action_terminal_error'):
            return error==report['action_terminal_error']
        return not error and report.get('action_terminal_state') in (3,4)

    def execute_leg(self,item,plan,cancel_distance=None,context=None,phase='coarse',
                    xy_tolerance=.08,yaw_tolerance=.12,align_path=False,station_approach=False,
                    budget=None):
        if station_approach and (context is None or align_path or cancel_distance is not None):
            raise ValueError('station approach needs a mission deadline and an ordinary navigation leg')
        if self.planner_mode=='path':
            if cancel_distance is not None:
                raise ValueError('distance-cancel trials use the diagnostic entry')
            if station_approach:
                xy_tolerance = .01 if is_pickup_station(self.keepout,item['name']) else .04
                yaw_tolerance = .07
            return self.follow_path(item,plan,context,xy_tolerance,yaw_tolerance)
        if align_path and cancel_distance is not None:
            raise ValueError('entry alignment and distance cancellation require separate trials')
        started=time.monotonic()
        budget=budget or NavigationBudget()
        prefix_report=None
        grid=self.grid();start=self.pose()
        pickup_approach=plan.get('pickup_approach',False)
        if pickup_approach and (phase!='align' or context is None or align_path
                                or station_approach or cancel_distance is not None):
            raise ValueError('pickup approach requires one bounded alignment leg')
        start_clear=goal_is_free(grid,*start[:2],radius=self.radius)
        target=plan.get('contact_handoff')
        if not start_clear:
            if target is None:
                raise ValueError('No clear handoff point on the live recorded-goal path within 60cm')
            context=context or ActionContext(self.owner,'field-'+str(time.monotonic()),
                'navigate',item['name'],0,time.monotonic()+float(plan['timeout_s'])+NavigationExecutor.STOP_BUDGET)
            rospy.set_param('/suat_navigation_gate/path_prefix_target',target)
            try:
                departure_context=budget.stage(context,'departure',time.monotonic()) if station_approach else context
                prefix_report=self.depart_a(departure_context)
            finally:
                if rospy.has_param('/suat_navigation_gate/path_prefix_target'):
                    rospy.delete_param('/suat_navigation_gate/path_prefix_target')
            if not prefix_report['passed']:
                return dict(prefix_report,name=item['name'],goal=list(item['pose']),
                    motion_goal_sent=prefix_report.get('motion_command_sent',False),
                    arrival_confirmed=False,error='recorded-goal path prefix failed: '+prefix_report.get('error',''),
                    phase='departure',first_failure_stage='departure',elapsed_s=time.monotonic()-started,
                    actual_pose=prefix_report['actual_pose'])
        if target is not None:
            if self.health_check:
                self.health_check()
            start, grid = self.pose(), self.grid()
            # Join the verified route at its planned handoff. The actual pose
            # can differ slightly because departure stops on measured odometry.
            joins_route = (math.dist(start[:2], target[:2]) <= .05
                           and StaticPlanner(grid,radius=self.radius).segment_free(start, target)
                           and (self.keepout is None or
                                self.keepout.segment_clear(start, target, self.radius)))
            if not joins_route:
                return dict(name=item['name'], goal=list(item['pose']), passed=False,
                            arrival_confirmed=False, first_failure_stage='departure_handoff',
                            initial_contact_prefix=prefix_report, actual_pose=start,
                            motion_goal_sent=bool(prefix_report), output_released=bool(prefix_report),
                            error='stopped pose cannot join the verified departure route')
            path = [start[:2], *plan['path_xy'][1:]]
            plan = dict(plan, start_pose=list(start), path_xy=path, contact_handoff=None,
                        path_length_m=sum(math.dist(a, b) for a,b in zip(path, path[1:])))
        if not hasattr(self,'executor'):
            self.executor=NavigationExecutor(RosNavigationTransport(self))
        entry_report=None
        if align_path and phase=='coarse':
            start=self.pose()
            if math.dist(start[:2],item['pose'][:2])>xy_tolerance:
                points=plan['path_xy']
                forward=next((point for point in points if math.dist(start[:2],point)>=.20),points[-1])
                heading=math.atan2(forward[1]-start[1],forward[0]-start[0])
                error=abs(math.atan2(math.sin(heading-start[2]),math.cos(heading-start[2])))
                if error>yaw_tolerance:
                    # Turning and travel consume the original leg's one budget.
                    context=context or ActionContext(self.owner,'field-'+str(time.monotonic()),
                        'navigate',item['name'],0,time.monotonic()+float(plan['timeout_s'])+NavigationExecutor.STOP_BUDGET)
                    entry={'name':item['name']+'__path_heading','pose':[start[0],start[1],heading]}
                    entry_plan=self.plan(start,entry['pose'],self.grid())
                    entry_report=self.executor.execute(entry,entry_plan,context=context,phase='align',
                        xy_tolerance=xy_tolerance,yaw_tolerance=yaw_tolerance)
                    if not entry_report['passed']:
                        return dict(entry_report,name=item['name'],goal=list(item['pose']),phase=phase,
                                    passed=False,arrival_confirmed=False,error='path entry alignment failed',
                                    entry_alignment=entry_report)
                    try:
                        plan=self.plan(self.pose(),item['pose'],self.grid())
                    except (Exception,KeyboardInterrupt) as exc:
                        return dict(entry_report,name=item['name'],goal=list(item['pose']),phase=phase,
                                    passed=False,arrival_confirmed=False,
                                    error='plan after entry turn failed: '+(str(exc) or type(exc).__name__),
                                    entry_alignment=entry_report)
        if station_approach:
            # Apply terminal tolerances before sending the real station goal.
            xy_tolerance = .01 if is_pickup_station(self.keepout,item['name']) else .04
            yaw_tolerance = .07
        report=self.executor.execute(item,plan,context=context,phase=phase,
            xy_tolerance=xy_tolerance,yaw_tolerance=yaw_tolerance,cancel_distance=cancel_distance,
            handoff_released=prefix_report is not None)
        if station_approach and report.get('actual_pose'):
            report['final_error_m'],report['final_yaw_error_rad']=arrival_error(item['pose'],report['actual_pose'])
        if prefix_report is not None:
            report['initial_contact_prefix']=prefix_report
            report['motion_goal_sent']|=prefix_report['motion_command_sent']
        if entry_report is not None:
            report['entry_alignment']=entry_report
            report['motion_goal_sent']|=entry_report['motion_goal_sent']
        report['leg_elapsed_s']=time.monotonic()-started
        if not report['passed']:
            report.setdefault('first_failure_stage',report.get('failure_stage',report['phase']))
            if not report.get('actual_pose'):
                report['actual_pose']=self.last_pose()
        return report

    def follow_path(self,item,plan,context,xy,yaw_tolerance):
        io=RosNavigationTransport(self)
        started=io.now()
        stop_budget=self.executor_stop_budget()
        end=context.deadline if context is not None else started+plan['timeout_s']+stop_budget
        motion_end=min(end-stop_budget,started+plan['timeout_s'])
        report=dict(name=item['name'],goal=list(item['pose']),plan=plan,phase='path',
                    passed=False,arrival_confirmed=False,motion_goal_sent=False,
                    output_lease_opened=False,output_released=False,stationary=False)
        route={}
        completed=False
        parameter='/suat_navigation_gate/route_request'
        try:
            io.ready()
            io.prepare_heartbeat()
            if io.now()>=motion_end:
                raise TimeoutError('navigation deadline reached before opening path')
            if self.health_check:
                self.health_check()
            rospy.set_param(parameter,dict(points=plan['path_xy'],goal=list(item['pose']),
                contact_end=(plan['contact_handoff'][:2] if plan['contact_handoff'] else []),
                xy_tolerance=xy,yaw_tolerance=yaw_tolerance))
            with self.lock:
                self.output_stats=(0,io.now(),0.)
            self.path_transport=io
            try:
                result=io.service('follow_path')
                if not result.success:
                    raise RuntimeError(result.message)
                report['output_lease_opened']=True
                response=json.loads(result.message)
            finally:
                rospy.delete_param(parameter)
            if self.health_check:
                self.health_check()
            io.start_heartbeat(response['token'])
            while not io.shutdown() and io.now()<motion_end:
                if self.health_check:
                    self.health_check()
                with self.lock:
                    state=dict(self.state or {})
                    age=io.now()-self.state_at
                if age>.7:
                    raise RuntimeError('path controller feedback expired')
                current=state.get('route') or {}
                if current.get('id')==response['route_id']:
                    route=current
                    if route['status']=='COMPLETED':
                        completed=True
                        break
                    if route['status']=='FAILED':
                        raise RuntimeError(route['error'])
                io.sleep(.05)
            if not completed:
                raise TimeoutError('navigation deadline reached')
        except (Exception,KeyboardInterrupt) as exc:
            report['error']=str(exc) or type(exc).__name__
            report['timed_out']=isinstance(exc,TimeoutError)
        finally:
            self.path_transport=None
            if report['output_lease_opened']:
                try:
                    if not io.stop():
                        raise RuntimeError('path stop was not acknowledged')
                except Exception as exc:
                    report['stop_error']=str(exc)
                finally:
                    io.stop_heartbeat()
                try:
                    report['stationary']=io.settle(min(end,io.now()+stop_budget))
                    report['output_released']=report['stationary'] and io.release()
                except Exception as exc:
                    report['release_error']=str(exc)
            else:
                io.stop_heartbeat()
        report['actual_pose']=self.last_pose()
        if report['actual_pose'] is not None:
            final_error,final_yaw=arrival_error(item['pose'],report['actual_pose'])
            report.update(final_error_m=final_error,final_yaw_error_rad=final_yaw)
            report['arrival_confirmed']=completed and final_error<=xy and final_yaw<=yaw_tolerance
        report['passed']=(report['arrival_confirmed'] and report['output_released']
                          and not report.get('stop_error'))
        if not report['passed']:
            report.setdefault('error','arrival, stationary odometry or output release was not confirmed')
            report['first_failure_stage']='path'
        with self.lock:
            output=self.output_stats
            report['travel_m']=(self.state or {}).get('travel_m') if report['output_lease_opened'] else 0.
            self.output_stats=None
        report.update(route=route,motion_goal_sent=bool(route.get('moving')),
                      output_messages=output[0] if output else 0,
                      output_max_gap_s=output[2] if output else None,
                      odom_turn_reversals=route.get('reversals',0),
                      odom_peak_mps=route.get('peak_mps',0.),
                      leg_elapsed_s=io.now()-started)
        return report

    def cancel(self):
        io=getattr(self,'path_transport',None)
        if io is not None:
            io.stop()
        if hasattr(self,'executor'):
            self.executor.cancel()

    def close(self):
        self.cancel()
        for subscription in self.subs:
            subscription.unregister()


class RosNavigationTransport:
    """ROS action/parameters/gate/sensors owned by one shared executor."""
    ACTIVE=(0,1,6,7)

    def __init__(self,navigator):
        self.navigator=navigator
        self.generation=None
        self.terminal=None
        self.sent=False
        self.heartbeat=None
        self.heartbeat_quit=threading.Event()
        self.worker=None

    def now(self):return time.monotonic()
    def sleep(self,duration):time.sleep(duration)
    def shutdown(self):return rospy.is_shutdown()

    def ready(self):
        node=self.navigator
        if getattr(node,'health_check',None):node.health_check()
        if not getattr(node,'connected',node.nav is not None):node.connect_control()
        with node.lock:
            if node.state is None or self.now()-node.state_at>.7:
                raise ValueError('fresh navigation gate state missing')
            if node.state.get('owner')!=node.owner:
                raise ValueError('navigation output belongs to another owner')
            if node.state.get('state')=='ARMED':
                raise ValueError('navigation output is already leased')

    @staticmethod
    def service(name):
        return rospy.ServiceProxy('/suat_navigation_gate/'+name,Trigger)()

    def stop(self):return bool(self.service('stop').success)

    def cancel_owned(self):
        if self.sent and self.navigator.nav.get_state() in self.ACTIVE:
            self.navigator.nav.cancel_goal()

    def settle(self,deadline):
        still_since=None
        while self.now()<deadline and not self.shutdown():
            node=self.navigator
            _,linear,angular,age=node.measured()
            with node.lock:
                state=dict(node.state or {})
                state_age=self.now()-node.state_at
            idle=(not self.sent or node.nav.get_state() not in self.ACTIVE)
            stationary=(all(math.isfinite(v) for v in (linear,angular,age)) and
                        0<=age<=.4 and abs(linear)<.015 and abs(angular)<.03)
            ready=(state_age<=.7 and state.get('owner')==node.owner and
                   state.get('state') in ('DISARMED','LATCHED') and state.get('health') is None and
                   state.get('inputs_clear') is True and state.get('active_goal') is False and idle)
            if stationary and ready:
                if still_since is None:still_since=self.now()
                if self.now()-still_since>=.5:return True
            else:still_since=None
            self.sleep(.05)
        return False

    def configure(self,xy,yaw_tolerance):
        changes={'xy_goal_tolerance':xy,'yaw_goal_tolerance':yaw_tolerance}
        mode=self.navigator.planner_mode
        actual=rospy.get_param(PLANNER_NAMESPACE,{})
        if not settings_match(actual,changes):
            from dynamic_reconfigure.client import Client

            actual=Client(PLANNER_NAMESPACE,timeout=2.).update_configuration(changes)
        expected = dict(changes, **planner_speed_limits(self.navigator.speed))
        if mode is not None:
            expected.update(local_planner_settings(mode))
        if not settings_match(actual, expected):
            raise ValueError('local planner did not confirm the fixed profile and goal tolerances')
        return actual

    def arm(self):
        result=self.service('arm')
        if not result.success:raise ValueError('output disabled: '+result.message)
        response=json.loads(result.message)
        profile=speed_profile(self.navigator.speed)
        if (response.get('linear_cap_mps')!=profile['linear']
                or response.get('angular_cap_radps')!=profile['angular']):
            self.stop()
            raise ValueError('navigation gate did not confirm the selected motion limits')
        return response['token']

    def prepare_heartbeat(self):
        if self.heartbeat is None:
            self.heartbeat=rospy.Publisher('/suat/navigation/heartbeat',String,queue_size=1,latch=False)
        deadline=self.now()+3.
        while not self.heartbeat.get_num_connections() and self.now()<deadline and not self.shutdown():
            self.sleep(.05)
        if not self.heartbeat.get_num_connections():raise ValueError('heartbeat not connected')

    def start_heartbeat(self,token):
        self.prepare_heartbeat()
        self.heartbeat_quit.clear()
        def renew():
            sequence=0
            while not self.heartbeat_quit.is_set() and not self.shutdown():
                self.heartbeat.publish(String(data=json.dumps({'token':token,'sequence':sequence})))
                sequence+=1
                self.heartbeat_quit.wait(.1)
        self.worker=threading.Thread(target=renew,daemon=True)
        self.worker.start()

    def stop_heartbeat(self):
        self.heartbeat_quit.set()
        if self.worker:self.worker.join(timeout=.5)
        if self.heartbeat:self.heartbeat.unregister()
        self.worker=self.heartbeat=None

    def send_goal(self,pose,generation):
        if getattr(self.navigator,'health_check',None):self.navigator.health_check()
        self.generation=generation
        self.terminal=None
        goal=MoveBaseGoal();goal.target_pose=self.navigator.stamped(pose)
        def finished(status,result):
            if generation==self.generation:self.terminal=status
        self.sent=True
        with self.navigator.lock:
            self.navigator.output_stats=(0,self.now(),0.)
        self.navigator.nav.send_goal(goal,done_cb=finished)

    def sample(self):
        node=self.navigator
        if getattr(node,'health_check',None):node.health_check()
        odom,linear,angular,age=node.measured()
        pose=node.pose()
        with node.lock:
            state=dict(node.state or {})
            state_age=self.now()-node.state_at
            output=getattr(node,'output_stats',None)
        status=self.terminal if self.terminal is not None else node.nav.get_state()
        status_text=node.nav.get_goal_status_text() if self.terminal is not None else ''
        return {'generation':self.generation,'status':status,'pose':pose,'odom_pose':odom,
                'linear':linear,'angular':angular,'age':age,'gate_state':state.get('state'),
                'gate_age':state_age,'gate_reason':state.get('reason'),'gate_health':state.get('health'),
                'status_text':status_text, 'output_messages':output[0] if output is not None else 0,
                'output_max_gap_s':max(output[2],self.now()-output[1]) if output is not None else None}

    def release(self):
        return bool(self.service('release').success)
