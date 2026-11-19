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
import termios
import time
import threading
import xmlrpc.client

import rospy
from suat_mission.manipulation.arm_feedback import SwiftProParser
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
    def __init__(self, run_dir, feed=1000.):
        self.run = Path(run_dir)
        self.feed = feed
        self.fast_feed=feed
        self.contact_feed=feed
        self.correction_feed=min(feed,1000.)
        self.correction_tolerance_mm=5.
        self.port = None
        self.paused = False
        self.pump_may_be_on = False
        self.handle = None
        self.sequence = 0
        self.active_deadline = None
        self.permitted = None
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

    def ask(self, command, timeout=2.):
        with self.io_lock:
            return self._ask(command,timeout)

    def _ask(self, command, timeout):
        if (self.permitted is not None and command.startswith(('G0 ', 'M2231 V1'))
                and not self.permitted()):
            raise RuntimeError('competition action was stopped before arm command')
        self.sequence += 1
        sequence = self.sequence
        at = self.now()
        deadline = at + timeout
        if self.active_deadline is not None and command != 'M2000':
            deadline = min(deadline, self.active_deadline)
        if at >= deadline:
            raise TimeoutError('arm action deadline reached')
        self.records.append({'time': at, 'sequence': sequence, 'command': command})
        self.handle.write(('#%d %s\r\n' % (sequence, command)).encode('ascii'))
        while self.now() < deadline:
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

    def open(self):
        source = Path('/home/spark/spark_noetic/src/spark_driver/arm/uArm/swiftpro/src/swiftpro_write_node.cpp')
        ports = re.findall(r'_serial\.setPort\("([^"\n]+)"\)', source.read_text())
        if len(ports) != 1 or not shutil.which('fuser'):
            raise RuntimeError('cannot identify the existing arm serial port')
        self.port = ports[0]
        state = self.master.getSystemState(rospy.get_name())[2]
        publishers = dict(state[0])
        topics = ('position_write_topic', 'swiftpro_status_topic', 'angle4th_topic', 'gripper_topic', 'pump_topic')
        if any(publishers.get('/' + topic) for topic in topics):
            raise RuntimeError('another arm command publisher is active')
        code, _, uri = self.master.lookupNode(rospy.get_name(), '/swiftpro_write_node')
        if code == 1:
            owner = xmlrpc.client.ServerProxy(uri)
            pid = owner.getPid(rospy.get_name())[2]
            owner.shutdown(rospy.get_name(), 'one continuous measured arm session')
            self.paused = True
            until = self.now() + 4
            while Path('/proc/%d' % pid).exists() and self.now() < until:
                time.sleep(.05)
            if Path('/proc/%d' % pid).exists():
                raise RuntimeError('old arm owner did not exit')
        held = subprocess.run(['fuser', self.port], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if held.returncode != 1 or held.stdout.strip():
            raise RuntimeError('arm serial device still in use')
        self.handle = SerialPort(self.port)
        # Until the device reports its state, do not assume suction is off.
        self.pump_may_be_on = True
        until = self.now() + 4
        while self.now() < until:
            self._receive()
        self.sequence = int(self.now() * 1000) % 100000000 + 1000
        if self.values(self.ask('P2234'), 'V') != [1.]:
            raise RuntimeError('arm power not present')
        if self.values(self.ask('P2231'), 'V') != [0.]:
            raise RuntimeError('pump not off at mission start')
        self.pump_may_be_on = False
        if self.values(self.ask('M2200'), 'V') != [0.]:
            raise RuntimeError('arm already moving')
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
        self.ask('M2000')
        started = self.now()
        self.ask('G0 X%.4f Y%.4f Z%.4f F%.0f' % (*xyz, self.feed), timeout=30)
        # Firmware idle can precede physical settling; avoid interrupting a move
        # merely because three rapid M2200 queries returned idle.
        table=((1000.,20.),(10000.,44.3),(40000.,78.7),(80000.,95.4))
        speed=table[-1][1]
        if self.feed<=table[0][0]:speed=table[0][1]
        else:
            for (f0,s0),(f1,s1) in zip(table,table[1:]):
                if self.feed<=f1:
                    speed=s0+(s1-s0)*(self.feed-f0)/(f1-f0);break
        # Use the measured feed table, not F/60: this firmware saturates.
        earliest_settled = started + max(.45,math.dist(start,xyz)/speed+.55)
        idle_count = 0
        previous = None
        while self.now() < deadline:
            measured = self.read_pose(deadline)['xyz']
            moving = self.values(self.ask('M2200'), 'V')[0]
            stable = previous is not None and math.dist(measured, previous) < 1.
            idle_count = idle_count + 1 if moving == 0 and stable and self.now() >= earliest_settled else 0
            if idle_count >= 3:
                return
            previous = measured
            time.sleep(.1)
        raise TimeoutError('arm did not settle before deadline')

    def stop(self, deadline):
        self.active_deadline = deadline
        frame = self.ask('M2000')
        idle = self.values(self.ask('M2200'), 'V') == [0.]
        return {'acknowledged': frame.kind == 'reply', 'controller_idle': idle, 'stop_sequence': frame.sequence}

    def go(self, stage, xyz, *, loaded=False, deadline=None):
        end=min(self.now()+75., deadline) if deadline is not None else self.now()+75.
        ctx = ActionContext(self.run.name, stage, 'arm_pose', '', 0, end)
        self.feed=self.contact_feed if stage in ('contact','lift','release_height') else self.fast_feed
        if not self.validate_target(xyz,end):raise RuntimeError(stage+': controller IK rejected target')
        initial=self.read_pose(end)['xyz']
        if math.dist(initial,xyz)>self.correction_tolerance_mm:
            self.move(xyz,end-3.)
        # Long travel may be fast. Small measured corrections retain the only
        # feed that converged in the field report, inside the same session.
        self.feed=self.correction_feed
        result = recover_pose(self, xyz, ctx, tolerance_mm=self.correction_tolerance_mm, max_moves=3,
                              stop_budget_s=3., max_correction_step_mm=5. if loaded else 10.,
                              max_command_offset_mm=10. if loaded else 25.)
        self.active_deadline = None
        self.records.append({'event': 'POSE_RESULT', 'stage': stage, 'loaded': loaded, 'result': result})
        if result['outcome'] != 'measured_target_reached':
            raise RuntimeError(stage + ': ' + result.get('error', 'measured pose did not reach target'))
        return result

    def pump(self, on, *, deadline=None):
        self.active_deadline = deadline
        if on:
            self.pump_may_be_on = True  # possible write must not be mistaken for off
        self.ask('M2231 V%d' % (1 if on else 0))
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
        status = {'pump_may_be_on': self.pump_may_be_on, 'factory_restored': False}
        if self.handle is not None:
            try:
                self.active_deadline = None
                self.ask('M2000')
            except Exception as exc:
                status['stop_error'] = str(exc)
            if not self.pump_may_be_on:
                self.handle.close()
                self.handle = None
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
        (self.run / 'arm_serial_trace.json').write_text(json.dumps(self.records, indent=2), encoding='utf-8')
        return status
