"""One exclusive SwiftPro connection for a complete experimental mission.

The original ROS writer is paused once and restored after the pump is off.
Joint readings are device feedback, not independent cup calibration or proof of holding.
"""
import base64
from dataclasses import asdict
import fcntl
import json
import math
import os
from pathlib import Path
import re
import select
import shutil
import struct
import subprocess
import tempfile
import termios
import time
import threading
import xmlrpc.client

import rospy
from suat_mission.manipulation.arm_feedback import SwiftProParser
from suat_mission.manipulation.arm_path import travel_dwell
from suat_mission.manipulation.arm_recovery import matched_reply_or_fault, recover_pose
from suat_mission.contracts import ActionContext


class SerialPort:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.original = termios.tcgetattr(self.fd)
            cfg = termios.tcgetattr(self.fd)
            cfg[0] = cfg[1] = cfg[3] = 0
            cfg[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            cfg[4] = cfg[5] = termios.B115200
            cfg[6][termios.VMIN] = cfg[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, cfg)
        except BaseException:
            os.close(self.fd)
            raise

    def read(self):
        if not select.select([self.fd], [], [], .05)[0]:
            return b''
        count = struct.unpack('I', fcntl.ioctl(self.fd, termios.FIONREAD, struct.pack('I', 0)))[0]
        try:
            return os.read(self.fd, max(1, min(count, 8192)))
        except BlockingIOError:
            return b''

    def write(self, data):
        if not select.select([], [self.fd], [], .3)[1]:
            raise TimeoutError('arm serial write timed out')
        if os.write(self.fd, data) != len(data):
            raise RuntimeError('partial arm serial write')

    def close(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
        finally:
            os.close(self.fd)


class ArmSession:
    """Serial backend retained across all empty and loaded arm motions."""
    READ_ONLY = frozenset(('P2234', 'P2231', 'P2200', 'M2200', 'M2221', 'M2222'))

    def __init__(self, run_dir, feed=1000.):
        self.run = Path(run_dir)
        self.feed = feed
        self.fast_feed=feed
        self.contact_feed=feed
        self.loaded_feed_limit = None
        self.correction_feed=min(feed,1000.)
        self.correction_tolerance_mm=5.
        self.use_recovery_feed = False
        self.recovering_pose = False
        self.port = None
        self.paused = False
        self.pump_may_be_on = False
        self.startup_pump_unread = False
        self.handle = None
        self.owner_fd = None
        self.initialized = False
        self.sequence = 0
        self.active_deadline = None
        self.permitted = None
        self.write_if_running = None
        self.parser = SwiftProParser()
        self.records = []
        self.io_lock = threading.RLock()
        self.master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))

    def now(self):
        return time.monotonic()

    def _receive(self):
        data = self.handle.read()
        if not data:
            return []
        at = self.now()
        self.records.append({'time': at, 'received_base64': base64.b64encode(data).decode()})
        return self.parser.feed(data, at)

    def ask(self, command, timeout=2., *, numbered=True, recovery=False):
        with self.io_lock:
            return self._ask(command,timeout,numbered,recovery)

    def _ask(self, command, timeout, numbered, recovery):
        if not command.split() or '\r' in command or '\n' in command:
            raise ValueError('one serial command is required')
        if recovery and command != 'M2231 V0':
            raise ValueError('only pump off is supported for explicit recovery')
        guarded = (command.split()[0] not in self.READ_ONLY
                   and command not in ('M2000', 'M2120 V0') and not recovery)
        if guarded and self.permitted is not None and not self.permitted():
            raise RuntimeError('competition action was stopped before arm command')
        if guarded and command != 'M17' and not self.initialized:
            raise RuntimeError('arm motion must be initialized before task commands')
        if numbered:
            self.sequence += 1
            sequence = self.sequence
            wire = '#%d %s' % (sequence, command)
        else:
            sequence = None
            wire = command
        at = self.now()
        deadline = at + timeout
        if self.active_deadline is not None and command != 'M2000':
            deadline = min(deadline, self.active_deadline)
        if at >= deadline:
            raise TimeoutError('arm action deadline reached')
        def transmit():
            if self.now() >= deadline:
                raise TimeoutError('arm action deadline reached')
            self.records.append({'time': at, 'sequence': sequence, 'command': command})
            if command.split() == ['M2231', 'V1']:
                self.pump_may_be_on = True  # A partial write can also turn on suction.
            self.handle.write((wire + '\r\n').encode('ascii'))

        if guarded and self.write_if_running is not None:
            self.write_if_running(transmit)
        else:
            transmit()
        while self.now() < deadline:
            if guarded and self.permitted is not None and not self.permitted():
                raise RuntimeError('competition action was stopped while waiting for arm reply')
            frame = matched_reply_or_fault(self._receive(), sequence, at)
            if frame is not None:
                self.records.append({'time': self.now(), 'reply': asdict(frame)})
                return frame
        raise TimeoutError('no arm reply: ' + command)

    @staticmethod
    def values(frame, names):
        if len(frame.payload) != len(names) or [part[0] for part in frame.payload] != list(names):
            raise ValueError('unexpected arm reply fields')
        values = [float(part[1:]) for part in frame.payload]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('invalid arm numeric reply')
        return values

    def _claim_port(self):
        # Lock before opening the tty. A competing open can reset the controller.
        device = Path(self.port).resolve().name
        path = Path(tempfile.gettempdir()) / ('suat-arm-' + device + '.lock')
        fd = os.open(path, os.O_CREAT | os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(fd)
            raise
        self.owner_fd = fd

    def _release_port(self):
        if self.owner_fd is not None:
            fd, self.owner_fd = self.owner_fd, None
            os.close(fd)

    def open(self, *, preparing=False, check_preparing=None):
        def check():
            if check_preparing is not None:
                check_preparing()

        with self.io_lock:
            check()
            if self.handle is None:
                if self.owner_fd is not None:
                    raise RuntimeError('previous serial initialization requires cleanup')
                try:
                    self._open_port(check)
                except BaseException:
                    if self.handle is None and not self.paused:
                        self._release_port()
                    raise
            if not preparing:
                self.enable_motion()

    def enable_motion(self):
        with self.io_lock:
            if not self.initialized:
                # A pose read during Prepare can leave an expired action
                # deadline. Start uses the current run deadline and reply limit.
                self.active_deadline = None
                self.ask('M17')
                if self.values(self.ask('M2200'), 'V') != [0.]:
                    raise RuntimeError('arm did not become idle after initialization')
                self.initialized = True

    def _open_port(self, check):
        source = Path('/home/spark/spark_noetic/src/spark_driver/arm/uArm/swiftpro/src/swiftpro_write_node.cpp')
        ports = re.findall(r'_serial\.setPort\("([^"\n]+)"\)', source.read_text())
        if len(ports) != 1 or not shutil.which('fuser'):
            raise RuntimeError('cannot identify the existing arm serial port')
        self.port = ports[0]
        self._claim_port()
        check()
        state = self.master.getSystemState(rospy.get_name())[2]
        publishers = dict(state[0])
        topics = ('position_write_topic', 'swiftpro_status_topic', 'angle4th_topic', 'gripper_topic', 'pump_topic')
        if any(publishers.get('/' + topic) for topic in topics):
            raise RuntimeError('another arm command publisher is active')
        code, _, uri = self.master.lookupNode(rospy.get_name(), '/swiftpro_write_node')
        if code == 1:
            owner = xmlrpc.client.ServerProxy(uri)
            pid = owner.getPid(rospy.get_name())[2]
            check()
            owner.shutdown(rospy.get_name(), 'one continuous measured arm session')
            self.paused = True
            until = self.now() + 4
            while Path('/proc/%d' % pid).exists() and self.now() < until:
                check()
                time.sleep(.05)
            if Path('/proc/%d' % pid).exists():
                raise RuntimeError('old arm owner did not exit')
        held = subprocess.run(['fuser', self.port], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3)
        if held.returncode != 1 or held.stdout.strip():
            raise RuntimeError('arm serial device still in use')
        check()
        self.handle = SerialPort(self.port)
        # Until the device reports its state, do not assume suction is off.
        self.pump_may_be_on = True
        self.startup_pump_unread = True
        until = self.now() + 4
        while self.now() < until:
            check()
            self._receive()
        self.sequence = int(self.now() * 1000) % 100000000 + 1000
        if self.values(self.ask('P2234'), 'V') != [1.]:
            raise RuntimeError('arm power not present')
        self.pump_may_be_on = self.values(self.ask('P2231'), 'V') != [0.]
        self.startup_pump_unread = False
        if self.pump_may_be_on:
            raise RuntimeError('pump not off at mission start')
        check()
        if self.values(self.ask('M2200'), 'V') != [0.]:
            raise RuntimeError('arm already moving')
        # Disable periodic telemetry here. Servo attach waits for Start.
        check()
        self.ask('M2120 V0')
        self.records.append({'time': self.now(), 'event': 'CONTINUOUS_SESSION_OPEN'})

    def validate_target(self, xyz, deadline):
        self.active_deadline = deadline
        return self.values(self.ask('M2222 X%.4f Y%.4f Z%.4f' % tuple(xyz)), 'V') == [1.]

    def read_pose(self, deadline):
        self.active_deadline = deadline
        joint = self.ask('P2200')
        angles = self.values(joint, 'BLR')
        fk = self.ask('M2221 B%.4f L%.4f R%.4f' % tuple(angles))
        return {'xyz': self.values(fk, 'XYZ'), 'source': 'P2200+M2221', 'joints_deg': angles,
                'joint_sequence': joint.sequence, 'fk_sequence': fk.sequence,
                'joint_received_at': joint.received_at, 'received_at': fk.received_at}

    def move(self, xyz, deadline):
        self.active_deadline = deadline
        if self.values(self.ask('M2200'), 'V') != [0.]:
            raise RuntimeError('arm moving before a new command')
        start = self.read_pose(deadline)['xyz']
        started = self.now()
        if self.recovering_pose:
            self.ask('M2000')
            self.ask('G0 X%.4f Y%.4f Z%.4f F%.0f' % (*xyz, self.feed), timeout=30)
        else:
            # The teammate's ROS writer sends this exact Cartesian wire format.
            self.ask('G0 X%.2f Y%.2f Z%.2f F%.0f' % (*xyz, self.feed), timeout=30, numbered=False)
        earliest_settled = started + travel_dwell(math.dist(start,xyz), self.feed)
        if not self.recovering_pose:
            # Let the controller execute the continuous G0 before querying joint/FK state.
            while self.now() < min(deadline, earliest_settled):
                if self.permitted is not None and not self.permitted():
                    raise RuntimeError('competition action was stopped during arm travel')
                time.sleep(.05)
        idle_count = 0
        previous = None
        while self.now() < deadline:
            feedback = self.read_pose(deadline)
            measured = feedback['xyz']
            moving = self.values(self.ask('M2200'), 'V')[0]
            stable = previous is not None and math.dist(measured, previous) < 1.
            idle_count = idle_count + 1 if moving == 0 and stable and self.now() >= earliest_settled else 0
            if idle_count >= 3:
                return feedback
            previous = measured
            time.sleep(.1)
        raise TimeoutError('arm did not settle before deadline')

    def stop(self, deadline):
        with self.io_lock:
            if self.handle is None:
                return {'acknowledged': False, 'controller_idle': False}
            self.active_deadline = deadline
            frame = self.ask('M2000')
            idle = self.values(self.ask('M2200'), 'V') == [0.]
            return {'acknowledged': frame.kind == 'reply', 'controller_idle': idle, 'stop_sequence': frame.sequence}

    def go(self, stage, xyz, *, loaded=False, deadline=None, feed_limit=None):
        end=min(self.now()+75., deadline) if deadline is not None else self.now()+75.
        ctx = ActionContext(self.run.name, stage, 'arm_pose', '', 0, end)
        self.feed=self.contact_feed if stage in ('contact','lift','release_height') else self.fast_feed
        loaded_limit = self.loaded_feed_limit if loaded or self.pump_may_be_on else None
        limits = [limit for limit in (loaded_limit, feed_limit) if limit is not None]
        self.feed = min([self.feed] + limits)
        requested_feed = self.feed
        self.recovering_pose = stage == 'unfold_empty_arm' or self.use_recovery_feed
        try:
            fast_result = None
            if not self.recovering_pose:
                result = recover_pose(self, xyz, ctx, tolerance_mm=self.correction_tolerance_mm,
                                      max_moves=1, stop_budget_s=3., stop_after_success=False)
                stop = result.get('stop', {})
                measurements = result['measurements']
                if (result.get('error') == 'bounded recovery exhausted before measured target convergence'
                        and stop.get('acknowledged') is True and stop.get('controller_idle') is True
                        and len(measurements) >= 2
                        and math.dist(measurements[0]['xyz'], measurements[-1]['xyz']) < 1.):
                    fast_result = result
                    self.use_recovery_feed = self.recovering_pose = True
            if self.recovering_pose:
                self.feed = min(self.feed, self.correction_feed, 1000.)
                # Feedback compensation is for empty repositioning only. A
                # descent, a loaded move or a possible hold sends one endpoint.
                compensate = not (loaded or self.pump_may_be_on or stage == 'contact')
                recovery = {'max_correction_step_mm': 10., 'max_command_offset_mm': 25.} if compensate else {}
                if fast_result is not None:
                    self.records.append({'event': 'FEED_FALLBACK', 'stage': stage,
                                         'requested_feed': requested_feed, 'retry_feed': self.feed,
                                         'settled_xyz_mm': fast_result['measurements'][-1]['xyz']})
                result = recover_pose(self, xyz, ctx, tolerance_mm=self.correction_tolerance_mm,
                                      max_moves=4 if compensate else 1, stop_budget_s=3., **recovery)
                if fast_result is not None:
                    result['initial_travel'] = fast_result
        finally:
            self.recovering_pose = False
            self.active_deadline = None
        self.records.append({'event': 'POSE_RESULT', 'stage': stage, 'loaded': loaded,
                             'requested_feed_mm_min': requested_feed,
                             'travel_feed_mm_min': self.feed, 'result': result})
        if result['outcome'] != 'measured_target_reached':
            error = result.get('error', 'measured pose did not reach target')
            if result.get('final_error_mm') is not None:
                error += ' (measured error %.1f mm)' % result['final_error_mm']
            raise RuntimeError(stage + ': ' + error)
        return result

    def pump(self, on, *, deadline=None, recovery=False):
        self.active_deadline = deadline
        self.ask('M2231 V%d' % (1 if on else 0), recovery=recovery)
        settled=self.now()+(2.5 if on else 3.)
        while self.now()<settled:
            if deadline is not None and self.now()>=deadline:
                raise TimeoutError('pump action deadline reached')
            if on and self.permitted is not None and not self.permitted():
                raise RuntimeError('pump action interrupted; holding remains uncertain')
            time.sleep(.05)
        raw = self.values(self.ask('P2231'), 'V')[0]
        self.records.append({'event': 'PUMP_CURRENT', 'requested_on': on, 'value': raw,
                             'interpretation': 'pump current only; holding not established'})
        if on and raw == 0:
            raise RuntimeError('pump reports stopped or low current')
        if not on:
            if raw != 0:
                raise RuntimeError('pump still reports current after off command')
            self.pump_may_be_on = False
        return raw

    def close(self):
        """Do not restart the serial driver if an object might still be held."""
        with self.io_lock:
            return self._close()

    def _close(self):
        status = {'pump_may_be_on': self.pump_may_be_on, 'factory_restored': False}
        if self.handle is not None:
            try:
                self.active_deadline = None
                self.ask('M2000')
            except Exception as exc:
                status['stop_error'] = str(exc)
            if self.startup_pump_unread:
                try:
                    self.pump_may_be_on = self.values(self.ask('P2231'), 'V') != [0.]
                    self.startup_pump_unread = False
                    status['pump_may_be_on'] = self.pump_may_be_on
                except Exception as exc:
                    status['startup_pump_query_error'] = str(exc)
            if not self.pump_may_be_on:
                self.handle.close()
                self.handle = None
                self.initialized = False
        if self.paused and not self.pump_may_be_on and self.write_if_running is not None:
            # The factory driver sends M17 when launched. In a formal session
            # teardown must release ownership without restarting that driver.
            self.paused = False
            status['factory_restart_deferred'] = True
        if self.paused and not self.pump_may_be_on:
            held = subprocess.run(['fuser', self.port], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3)
            if held.returncode != 1 or held.stdout.strip():
                raise RuntimeError('arm serial port still held; writer not restarted')
            with (self.run / 'arm_writer_restore.log').open('wb') as output:
                proc = subprocess.Popen(['rosrun', 'swiftpro', 'swiftpro_write_node'], stdout=output,
                                        stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
            time.sleep(4)
            status.update(factory_restored=proc.poll() is None, factory_pid=proc.pid)
            if status['factory_restored']:self.paused=False
        if self.handle is None and not self.paused:
            self._release_port()
        status['ownership_retained'] = self.owner_fd is not None
        (self.run / 'arm_serial_trace.json').write_text(json.dumps(self.records, indent=2), encoding='utf-8')
        return status
