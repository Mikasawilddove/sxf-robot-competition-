#!/usr/bin/env python3
"""Plan a short base-only goal; --execute --site-ready is required for movement."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from suat_mission.navigation.navigation_gate import goal_is_free, motion_bounds_fault
from suat_mission.navigation.navigation_map import load_binding, verify_map
from suat_mission.navigation.navigation_test_geometry import short_goal,arrival_error,arrival_confirmed


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Test:
    def __init__(self, root):
        self.binding = load_binding(root)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.latest_odom = None
        self.state = None
        self.state_at = 0.0
        self.report = {}
        self.lock = threading.RLock()
        self.subs = [rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1),
                     rospy.Subscriber('/suat/navigation/state', String, self.on_state, queue_size=1)]

    def on_odom(self, msg):
        with self.lock:
            self.latest_odom = (time.monotonic(), msg)

    def on_state(self, msg):
        try:
            with self.lock:
                self.state, self.state_at = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def plan(self, forward, left=0.0):
        msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=4)
        grid = verify_map(msg, self.binding)
        transform = self.buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(3))
        age = (rospy.Time.now()-transform.header.stamp).to_sec()
        if not -1.1 <= age <= .5:
            raise ValueError('stale current map pose')
        t, q = transform.transform.translation, transform.transform.rotation
        direction = yaw(q)
        target=short_goal((t.x,t.y,direction),forward,left)
        start = PoseStamped()
        start.header.frame_id, start.header.stamp = 'map', rospy.Time.now()
        start.pose.position.x, start.pose.position.y = t.x, t.y
        start.pose.orientation = q
        goal = MoveBaseGoal()
        goal.target_pose.header = start.header
        goal.target_pose.pose.position.x = target[0]
        goal.target_pose.pose.position.y = target[1]
        goal.target_pose.pose.orientation = q
        report = self.report
        report.update(validation_type='LIVE_PLAN_ONLY', execute_requested=False,
                      start_xy_yaw=[t.x, t.y, direction], goal_xy_yaw=list(target),
                      forward_m=forward, left_m=left, motion_goal_sent=False,
                      planner_profile=rospy.get_param('/suat_navigation_gate/profile', 'UNKNOWN'),
                      map_image_sha256=self.binding['image_sha256'])
        # Navfn may clear the start cell during make_plan. Check the whole
        # configured body against the immutable map before that side effect.
        footprint=rospy.get_param('/move_base/global_costmap/footprint', '[]')
        if footprint not in (None, '', '[]', []):
            raise ValueError('short test currently requires the configured circular footprint')
        radius=float(rospy.get_param('/move_base/global_costmap/robot_radius', .20))
        radius+=float(rospy.get_param('/move_base/global_costmap/footprint_padding', .02))
        report['start_collision_radius_m']=radius
        report['start_footprint_known_free']=goal_is_free(grid,t.x,t.y,radius)
        if not report['start_footprint_known_free']:
            report['failure_stage']='start_footprint'
            raise ValueError('start body footprint overlaps occupied or unknown map cells; use an open staging pose or verify localization and sensor mounting')
        if not goal_is_free(grid, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y):
            raise ValueError('goal footprint is not entirely known free space')
        rospy.wait_for_service('/move_base/make_plan', timeout=4)
        plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)(start, goal.target_pose, 0.0).plan
        if len(plan.poses) < 2:
            # Report the actual blocked start rather than asking for repeated
            # pose estimates or silently changing collision dimensions.
            try:
                costs = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=2)
                if costs.header.frame_id == 'map':
                    c = math.floor((t.x-costs.info.origin.position.x)/costs.info.resolution)
                    r = math.floor((t.y-costs.info.origin.position.y)/costs.info.resolution)
                    if 0 <= c < costs.info.width and 0 <= r < costs.info.height:
                        cost = costs.data[r*costs.info.width+c]
                        report['start_costmap_value'] = cost
                        if cost >= 99:
                            report['failure_stage'] = 'start_collision'
                            raise ValueError('start pose is in the inflated obstacle region; verify body-center clearance or sensor mounting transform')
            except ValueError:
                raise
            except Exception as exc:
                report['start_diagnostic_error'] = str(exc)
            report['failure_stage'] = 'global_plan'
            raise ValueError('no valid path returned')
        end = plan.poses[-1].pose.position
        if math.hypot(end.x-goal.target_pose.pose.position.x, end.y-goal.target_pose.pose.position.y) > .05:
            raise ValueError('planner returned a partial path')
        length = sum(math.hypot(b.pose.position.x-a.pose.position.x, b.pose.position.y-a.pose.position.y)
                     for a, b in zip(plan.poses, plan.poses[1:]))
        if length > .65:
            raise ValueError('path exceeds the first-test distance bound')
        report.update(path_poses=len(plan.poses), path_length_m=length)
        return goal, report

    def execute(self, goal, report, cancel_distance=None):
        nav = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not nav.wait_for_server(rospy.Duration(3)):
            raise RuntimeError('move_base server unavailable')
        for service in ('arm', 'stop', 'release'):
            rospy.wait_for_service('/suat_navigation_gate/'+service, timeout=3)
        arm = rospy.ServiceProxy('/suat_navigation_gate/arm', Trigger)
        stop = rospy.ServiceProxy('/suat_navigation_gate/stop', Trigger)
        release = rospy.ServiceProxy('/suat_navigation_gate/release', Trigger)
        heartbeat = rospy.Publisher('/suat/navigation/heartbeat', String, queue_size=1, latch=False)
        deadline = time.monotonic()+3
        while not heartbeat.get_num_connections() and time.monotonic() < deadline:
            time.sleep(.05)
        if not heartbeat.get_num_connections():
            raise RuntimeError('heartbeat gate not connected')
        # The action client has now registered; allow the graph monitor to see it.
        time.sleep(.3)
        with self.lock:
            if self.latest_odom is None or time.monotonic()-self.latest_odom[0] > .4:
                raise RuntimeError('fresh odometry missing')
            before = self.latest_odom[1].pose.pose.position
            start_odom = (before.x, before.y, yaw(self.latest_odom[1].pose.pose.orientation))
        armed = arm()
        if not armed.success:
            raise RuntimeError('output remains disabled: '+armed.message)
        token = json.loads(armed.message)['token']
        sequence, sent = 0, False
        report.update(validation_type='LIVE_SINGLE_GOAL_TEST', execute_requested=True)
        done_event = threading.Event()
        renewal_error = []
        def renew():
            nonlocal sequence
            while not done_event.is_set() and not rospy.is_shutdown():
                try:
                    heartbeat.publish(String(data=json.dumps({'token': token, 'sequence': sequence})))
                    sequence += 1
                except Exception as exc:
                    renewal_error.append(str(exc))
                    return
                done_event.wait(.1)
        thread = threading.Thread(target=renew, daemon=True)
        thread.start()
        try:
            goal.target_pose.header.stamp = rospy.Time.now()
            nav.send_goal(goal)
            sent = report['motion_goal_sent'] = True
            started = time.monotonic()
            max_displacement = 0.0
            cancel_requested=False
            while time.monotonic()-started < 30 and not rospy.is_shutdown():
                with self.lock:
                    if renewal_error or self.state is None or time.monotonic()-self.state_at > .7:
                        raise RuntimeError('gate state or heartbeat unavailable')
                    # Give the state publisher one cycle to report the new lease.
                    if time.monotonic()-started > .3 and self.state.get('state') != 'ARMED':
                        raise RuntimeError('output gate stopped: '+str(self.state))
                    if self.latest_odom is None or time.monotonic()-self.latest_odom[0] > .4:
                        raise RuntimeError('odometry expired')
                    pos = self.latest_odom[1].pose.pose.position
                    displacement = math.hypot(pos.x-start_odom[0], pos.y-start_odom[1])
                    bound_fault = motion_bounds_fault(start_odom, (pos.x, pos.y, yaw(self.latest_odom[1].pose.pose.orientation)))
                    max_displacement = max(displacement, max_displacement)
                if bound_fault:
                    raise RuntimeError(bound_fault)
                if cancel_distance is not None and displacement>=cancel_distance:
                    stop()
                    nav.cancel_goal()
                    cancel_requested=True
                    break
                if nav.get_state() not in (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('single-goal deadline or shutdown')
            report.update(action_terminal_state=nav.get_state(), elapsed_s=time.monotonic()-started,
                          max_odom_displacement_m=max_displacement)
            report['cancel_test_requested']=cancel_distance is not None
            report['cancel_requested']=cancel_requested
            if cancel_requested:
                nav.wait_for_result(rospy.Duration(2))
                report['action_terminal_state']=nav.get_state()
                report['expected_cancel_terminal']=nav.get_state() in (GoalStatus.PREEMPTED,GoalStatus.RECALLED)
                report['action_succeeded']=False
            else:
                # Stop first. Factory odometry is averaged, so allow it to
                # settle before judging arrival; a stale TF cannot pass.
                stop()
                report['action_succeeded'] = False
                deadline = time.monotonic()+2.5
                while not rospy.is_shutdown():
                    tr=self.buffer.lookup_transform('map','base_footprint',rospy.Time(0),rospy.Duration(1))
                    p,q=tr.transform.translation,tr.transform.rotation
                    actual=(p.x,p.y,yaw(q))
                    tf_age=(rospy.Time.now()-tr.header.stamp).to_sec()
                    with self.lock:
                        measured=self.latest_odom
                        age=time.monotonic()-measured[0]
                        v=measured[1].twist.twist.linear.x;w=measured[1].twist.twist.angular.z
                    report['arrival_xy_error_m'],report['arrival_yaw_error_rad']=arrival_error(report['goal_xy_yaw'],actual)
                    report.update(arrival_pose=actual, arrival_tf_age_s=tf_age,
                                  arrival_linear_mps=v, arrival_angular_radps=w)
                    report['action_succeeded']=nav.get_state()==GoalStatus.SUCCEEDED and -1.1<=tf_age<=.5 and arrival_confirmed(
                        report['goal_xy_yaw'],actual,v,w,age)
                    if report['action_succeeded'] or time.monotonic()>=deadline or nav.get_state()!=GoalStatus.SUCCEEDED:
                        break
                    time.sleep(.1)
        finally:
            # Close final output before cancelling this client's own goal.
            done_event.set()
            thread.join(timeout=.5)
            try:
                report['stop_service'] = stop().success
            except Exception as exc:
                report['stop_service_error'] = str(exc)
            finally:
                if sent and nav.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                    nav.cancel_goal()
            # The factory odometry twist is averaged and settles later than pose.
            # Keep zero output latched while waiting; never release on a fixed delay.
            until = time.monotonic()+4
            report['output_released'] = False
            while time.monotonic() < until and not rospy.is_shutdown():
                time.sleep(.15)
                try:
                    released = release()
                    report['output_released'] = released.success
                    report['release_message'] = released.message
                    if released.success:
                        break
                except Exception as exc:
                    report['release_message'] = str(exc)
            report['physical_stop_validation'] = 'requires independent on-site observation'
            heartbeat.unregister()
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root', type=Path, default=Path.home()/'SUAT/spark')
    parser.add_argument('--forward', type=float, default=.25, help='first test: 0.15..0.50 m forward of live pose')
    parser.add_argument('--left',type=float,default=0.,help='bounded offset toward robot left; positive points toward field interior from A facing D')
    parser.add_argument('--cancel-distance',type=float,help='stop and cancel this test after this much odometry displacement')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--site-ready', action='store_true', help='operator has cleared the path and stowed the arm')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.forward) or not .15 <= args.forward <= .50:
        parser.error('forward must be between .15 and .50 m')
    try:short_goal((0.,0.,0.),args.forward,args.left)
    except ValueError as exc:parser.error(str(exc))
    if args.cancel_distance is not None and (not math.isfinite(args.cancel_distance) or not .03<=args.cancel_distance<math.hypot(args.forward,args.left)):
        parser.error('cancel-distance must be at least .03 m and below the goal distance')
    if args.execute and not args.site_ready:
        parser.error('--execute requires current --site-ready confirmation')
    rospy.init_node('suat_point_navigation_test', anonymous=False)
    report = {}
    try:
        test = Test(args.workspace_root)
        report = test.report
        goal, report = test.plan(args.forward,args.left)
        if args.execute:
            test.execute(goal, report,args.cancel_distance)
        expected=report.get('expected_cancel_terminal') if args.cancel_distance is not None else report.get('action_succeeded')
        return 0 if not args.execute or (expected and report.get('output_released')) else 2
    except Exception as exc:
        report['error'] = str(exc)
        return 2
    finally:
        report['client_time_epoch'] = time.time()
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=True, indent=2), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
