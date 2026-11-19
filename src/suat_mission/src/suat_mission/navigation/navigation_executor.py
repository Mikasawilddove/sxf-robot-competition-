"""One navigation execution boundary, independent of ROS and physical I/O."""
import math
import threading
from suat_mission.navigation.navigation_test_geometry import arrival_confirmed, arrival_error
from suat_mission.navigation.navigation_profiles import path_travel_fault, planner_speed_limits


class NavigationExecutor:
    STOP_BUDGET = 5.0

    def __init__(self, transport):
        self.transport = transport
        self.generation = 0
        self.cancelled = threading.Event()
        self.command_lock = threading.RLock()
        self.owns_output = False
        self.busy = False

    def cancel(self):
        self.cancelled.set()
        with self.command_lock:
            if self.owns_output:
                self.transport.stop()
                self.transport.cancel_owned()

    def _check(self):
        if self.cancelled.is_set() or self.transport.shutdown():
            raise ValueError('navigation cancelled or shutting down')

    def execute(self, item, plan, phase='coarse', xy_tolerance=.08, yaw_tolerance=.12,
                context=None, cancel_distance=None, controller_speed=None):
        io = self.transport
        report = {'name': item['name'], 'goal': list(item['pose']), 'plan': plan,
                  'motion_goal_sent': False, 'arrival_confirmed': False,
                  'phase': phase, 'passed': False}
        with self.command_lock:
            if self.busy:
                report['error'] = 'another navigation request is already in flight'
                return report
            self.busy = True
            self.generation += 1
            generation = self.generation
            report['generation'] = generation
        try:
            armed = False
            started = io.now()
            parent_deadline = None
            try:
                self._check()
                timeout = float(plan['timeout_s'])
                if not math.isfinite(timeout) or timeout <= 0:
                    raise ValueError('invalid navigation timeout')
                parent_deadline = context.deadline if context is not None else started + timeout + self.STOP_BUDGET
                if not math.isfinite(parent_deadline):
                    raise ValueError('invalid parent navigation deadline')
                work_deadline = min(started + timeout, parent_deadline - self.STOP_BUDGET)
                report['parent_deadline'] = parent_deadline
                if work_deadline <= io.now():
                    raise ValueError('insufficient navigation deadline budget for stopped handoff')
                io.ready()
                if (phase not in ('coarse', 'align') or not all(math.isfinite(v) and v > 0
                        for v in (xy_tolerance, yaw_tolerance))):
                    raise ValueError('invalid navigation phase or tolerances')
                report['stop_requested'] = bool(io.stop())
                report['output_released'] = bool(report['stop_requested'] and
                    io.settle(min(parent_deadline, io.now()+self.STOP_BUDGET)) and io.release())
                if not report['output_released']:
                    raise ValueError('navigation handoff is not stopped and released')
                expected = {'xy_goal_tolerance': xy_tolerance, 'yaw_goal_tolerance': yaw_tolerance}
                report['tolerances'] = dict(expected)
                if controller_speed is None:
                    configured = io.configure(xy_tolerance, yaw_tolerance)
                else:
                    limits = planner_speed_limits(controller_speed)
                    expected.update(limits)
                    report['controller_speed_profile'] = controller_speed
                    report['controller_limits'] = limits
                    configured = io.configure(xy_tolerance, yaw_tolerance, controller_speed=controller_speed)
                if any(key not in configured or not math.isfinite(float(configured[key]))
                       or abs(float(configured[key])-value) > 1e-8 for key,value in expected.items()):
                    raise ValueError('planner configuration readback differs from requested phase')
                if type(configured.get('use_dwa')) is bool:
                    report['local_planner_mode']='dwa' if configured['use_dwa'] else 'rollout'
                    report['sampling_readback']={key:configured[key] for key in (
                        'sim_time','sim_granularity','angular_sim_granularity','vx_samples','vth_samples',
                        'path_distance_bias','goal_distance_bias','occdist_scale') if key in configured}
                if cancel_distance is not None and (not math.isfinite(cancel_distance) or
                                                     not .03 <= cancel_distance <= .50):
                    raise ValueError('cancel distance must be .03..0.50 metres')
                origin = io.sample()['odom_pose']
                with self.command_lock:
                    self._check()
                    if io.now() >= work_deadline:
                        raise ValueError('navigation deadline expired during preparation')
                    token = io.arm()
                    armed_at = io.now()
                    armed = self.owns_output = True
                    io.start_heartbeat(token)
                    self._check()
                    if io.now() >= work_deadline:
                        raise ValueError('navigation deadline expired before goal submission')
                    # A transport error can occur after the request has left this
                    # process. Retain the attempted goal for cleanup and reporting.
                    report['motion_goal_sent'] = True
                    report['goal_acknowledged'] = False
                    io.send_goal(item['pose'], generation)
                    report['goal_acknowledged'] = True
                previous = origin
                travel = 0.
                progress_at = io.now()
                best_xy = best_yaw = float('inf')
                while True:
                    self._check()
                    if io.now() >= work_deadline:
                        raise ValueError('navigation deadline reached')
                    sample = io.sample()
                    if sample['generation'] != generation:
                        raise ValueError('result belongs to an older navigation goal')
                    if (not all(math.isfinite(sample[k]) for k in ('gate_age', 'age'))
                            or not 0 <= sample['gate_age'] <= .7 or not 0 <= sample['age'] <= .4):
                        raise ValueError('navigation output state or odometry expired')
                    # The state topic updates asynchronously after arm. Preparation
                    # and dynamic reconfigure must not consume that startup window.
                    if io.now()-armed_at > .35 and sample['gate_state'] != 'ARMED':
                        raise ValueError('navigation output gate stopped')
                    current = sample['odom_pose']
                    if previous is not None:
                        fault, travel = path_travel_fault(previous, current, travel,
                            limit=min(8., max(.6, float(plan['path_length_m'])*1.8+.4)))
                        if fault:
                            raise ValueError(fault)
                    previous = current
                    report.update(action_terminal_state=sample['status'], actual_pose=list(sample['pose']),
                                  linear_mps=sample['linear'], angular_radps=sample['angular'], travel_m=travel)
                    report['peak_linear_mps']=max(report.get('peak_linear_mps',0.),abs(sample['linear']))
                    report['peak_angular_radps']=max(report.get('peak_angular_radps',0.),abs(sample['angular']))
                    if cancel_distance is not None and math.dist(current[:2], origin[:2]) >= cancel_distance:
                        io.stop()
                        io.cancel_owned()
                        until = min(work_deadline, io.now()+2.)
                        while io.now() < until:
                            cancelled = io.sample()
                            if cancelled['generation'] == generation and cancelled['status'] not in (0,1,6,7):
                                report['cancel_confirmed'] = cancelled['status'] in (2,8)
                                report['action_terminal_state'] = cancelled['status']
                                break
                            self._check()
                            io.sleep(.05)
                        break
                    if sample['status'] not in (0, 1, 6, 7):
                        report['arrival_confirmed'] = sample['status'] == 3 and arrival_confirmed(
                            item['pose'], sample['pose'], sample['linear'], sample['angular'], sample['age'],
                            xy_tolerance, yaw_tolerance)
                        break
                    xy, angle = arrival_error(item['pose'], sample['pose'])
                    if xy < best_xy-max(.002, xy_tolerance*.2) or angle < best_yaw-max(.01, yaw_tolerance*.2):
                        progress_at = io.now()
                        best_xy, best_yaw = min(best_xy, xy), min(best_yaw, angle)
                    if io.now()-progress_at > 20.:
                        raise ValueError('no navigation progress for 20 seconds')
                    io.sleep(.05)
            except (Exception, KeyboardInterrupt) as exc:
                report['error'] = str(exc) or type(exc).__name__
            finally:
                if armed:
                    report['output_released'] = False
                    report['stop_requested'] = False
                    try:
                        report['stop_requested'] = bool(io.stop())
                        io.cancel_owned()
                    except Exception as exc:
                        report['stop_error'] = str(exc)
                    finally:
                        try:
                            io.stop_heartbeat()
                        except Exception as exc:
                            report['heartbeat_error'] = str(exc)
                    try:
                        report['output_released'] = bool(io.settle(min(parent_deadline,
                            io.now()+self.STOP_BUDGET)) and io.release())
                    except Exception as exc:
                        report['release_error'] = str(exc)
                    self.owns_output = not report['output_released']
            if parent_deadline is not None and io.now() >= parent_deadline:
                report.setdefault('error', 'parent navigation deadline reached during cleanup')
            if (cancel_distance is None and not report.get('error') and report.get('output_released')
                    and report.get('action_terminal_state') == 3):
                try:
                    fresh = io.sample()
                    self._check()
                    if io.now() >= parent_deadline:
                        raise ValueError('parent navigation deadline reached during stopped arrival readback')
                    report['arrival_confirmed'] = (fresh['generation'] == generation
                        and fresh['status'] == 3 and math.isfinite(fresh['gate_age'])
                        and 0 <= fresh['gate_age'] <= .7 and arrival_confirmed(
                            item['pose'], fresh['pose'], fresh['linear'], fresh['angular'], fresh['age'],
                            xy_tolerance, yaw_tolerance))
                    report.update(actual_pose=list(fresh['pose']), linear_mps=fresh['linear'],
                                  angular_radps=fresh['angular'])
                except Exception as exc:
                    report['arrival_confirmed'] = False
                    report['error'] = 'stopped arrival readback failed: '+str(exc)
            report['elapsed_s'] = io.now()-started
            completed = report.get('cancel_confirmed', False) if cancel_distance is not None else report['arrival_confirmed']
            report['passed'] = bool(not report.get('error') and completed and report.get('stop_requested')
                                    and report.get('output_released') and not any(
                                        report.get(key) for key in ('stop_error','release_error','heartbeat_error')))
            return report
        finally:
            with self.command_lock:
                self.busy = False
