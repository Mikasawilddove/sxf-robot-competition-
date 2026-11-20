"""Formal ROS 1 startup through fake ROS, recorder and SwiftPro transport."""
from contextlib import ExitStack
from dataclasses import dataclass
import importlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/suat_mission/src'))
importlib.import_module('numpy')


class Message:
    def __init__(self, data=None):
        self.data = data


def module(name, **attributes):
    obj = ModuleType(name)
    vars(obj).update(attributes)
    return obj


class Port:
    def __init__(self, path, commands):
        self.commands = commands
        self.reply = b''
        self.closed = False
        self.fault_on = None

    def write(self, wire):
        parts = wire.decode().strip().split(' ', 1)
        numbered = parts[0].startswith('#')
        prefix, command = ('$' + parts[0][1:] + ' ', parts[1]) if numbered else ('', ' '.join(parts))
        self.commands.append(command)
        payload = {'P2234': ' V1', 'P2231': ' V0', 'M2200': ' V0', 'P2200': ' B90 L90 R30'}
        result = ' X200 Y0 Z150' if command.startswith('M2221 ') else payload.get(command, '')
        self.reply = (prefix + ('E4' if command == self.fault_on else 'ok' + result) + '\r\n').encode()

    def read(self):
        reply, self.reply = self.reply, b''
        return reply

    def close(self):
        self.closed = True


class StartupFixture(unittest.TestCase):
    port_type = Port
    isolated_startup = True

    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.folder = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.config = {'execution_backend': 'serial_rgbd', 'mission_scope': 'cake-only'}
        self.commands = []
        self.ports = []
        ros = module('rospy', get_param=lambda key, default=None: self.config if key == '~' else default,
                     get_name=lambda: '/suat_mission', is_shutdown=lambda: False,
                     Publisher=Mock(), Subscriber=Mock(), Service=Mock(), Timer=Mock(), Duration=lambda x: x,
                     on_shutdown=Mock(), loginfo=Mock(), logwarn=Mock(), logerr=Mock())
        stubs = {
            'rospy': ros, 'actionlib': module('actionlib'),
            'actionlib_msgs.msg': module('actionlib_msgs.msg', GoalStatus=object),
            'std_msgs.msg': module('std_msgs.msg', String=Message, Bool=Message),
            'geometry_msgs.msg': module('geometry_msgs.msg', Twist=Message, PoseStamped=object),
            'move_base_msgs.msg': module('move_base_msgs.msg', MoveBaseAction=object,
                                         MoveBaseGoal=object, MoveBaseActionResult=object),
            'nav_msgs.msg': module('nav_msgs.msg', OccupancyGrid=object, Odometry=object),
            'nav_msgs.srv': module('nav_msgs.srv', GetPlan=object),
            'std_srvs.srv': module('std_srvs.srv', Trigger=object,
                                   TriggerResponse=lambda success, message: SimpleNamespace(success=success, message=message)),
            'swiftpro.msg': module('swiftpro.msg', position=Message, status=Message),
            'vision_msgs.msg': module('vision_msgs.msg', Detection2DArray=object),
            'tf2_ros': module('tf2_ros'),
        }
        if sys.platform == 'win32':
            stubs.update(fcntl=module('fcntl', LOCK_EX=2, LOCK_NB=4, flock=Mock()),
                         termios=module('termios'))
        self.stack.enter_context(patch.dict(sys.modules, stubs))
        self.adapter = importlib.import_module('suat_mission.automation.mission_node')
        self.runtime = importlib.import_module('suat_mission.automation.serial_mission')
        self.arm = importlib.import_module('suat_mission.manipulation.arm_session')
        now = time.monotonic()

        def tick():
            nonlocal now
            now += .02
            return now

        clock = SimpleNamespace(monotonic=tick, sleep=lambda _: None)
        self.clock = clock
        self.ros = ros
        for target in (self.arm, self.runtime, self.adapter):
            self.stack.enter_context(patch.object(target, 'time', clock))
        self.stack.enter_context(patch.object(self.arm.tempfile, 'gettempdir', return_value=str(self.folder)))
        original_read = Path.read_text
        self.device = self.folder / 'ttyMock'
        self.stack.enter_context(patch.object(Path, 'read_text', lambda path, *a, **k:
            f'_serial.setPort("{self.device.as_posix()}")' if path.name == 'swiftpro_write_node.cpp'
            else original_read(path, *a, **k)))
        self.stack.enter_context(patch.object(self.arm.shutil, 'which', return_value='/mock/fuser'))
        self.stack.enter_context(patch.object(self.arm.subprocess, 'run', return_value=SimpleNamespace(returncode=1, stdout=b'')))
        self.stack.enter_context(patch.object(self.arm.xmlrpc.client, 'ServerProxy', return_value=SimpleNamespace(
            getSystemState=lambda _: (1, '', [[], [], []]), lookupNode=lambda *_: (0, '', ''))))

        def make_port(path):
            port = self.port_type(path, self.commands)
            self.ports.append(port)
            return port

        self.stack.enter_context(patch.object(self.arm, 'SerialPort', side_effect=make_port))
        def configure(robot):
            runtime = self.runtime.SerialMissionRuntime.__new__(self.runtime.SerialMissionRuntime)
            runtime.config, runtime.run = robot.config, self.folder
            runtime.session, runtime.arm_pose, runtime.active_pub = None, None, None
            runtime.held = None
            runtime.events = []
            runtime.ensure = lambda: robot._ensure_running(check_attachment=False)
            runtime.event = lambda *a, **k: None
            robot.serial_runtime = runtime
            robot.observation_timeout = .1

        if self.isolated_startup:
            self.stack.enter_context(patch.object(self.adapter, 'load_binding', return_value={}))
            self.stack.enter_context(patch.object(self.adapter, 'inspect_config',
                return_value={'configuration_complete': True}))
            self.stack.enter_context(patch.object(self.runtime, 'resolve_serial_config', lambda config, _: config))
            self.stack.enter_context(patch.object(self.adapter.SerialRosRobot, '_load_config', configure))
            self.stack.enter_context(patch.object(self.adapter.SerialRosRobot, '_serial_scene', return_value=object()))
            self.stack.enter_context(patch.object(self.adapter.SerialRosRobot, 'prepare_navigation', return_value=None))
        self.node = self.adapter.Node()
        self.recorder = Mock(path=self.folder, finished=False)
        self.node._make_recorder = lambda: setattr(self.node, 'recorder', self.recorder)

    def command(self, name, **extra):
        self.node.command(Message(json.dumps({'command': name, **extra})))

    def join(self):
        self.node.thread.join(2)
        self.assertFalse(self.node.thread.is_alive())

    def prepare(self):
        self.command('prepare')
        self.join()
        self.assertTrue(self.node.prepared, self.node.error)
        return self.node.robot.serial_runtime.session


class FormalStartup(StartupFixture):
    def test_prepare_reaches_ready_through_serial_handshake(self):
        session = self.prepare()
        self.assertIn('P2200', self.commands)
        self.assertIn('M2120 V0', self.commands)
        self.assertFalse(session.initialized)
        self.assertNotIn('M17', self.commands)
        self.assertFalse(any(command.startswith(('G0', 'M2231')) for command in self.commands))
        self.command('reset')
        self.assertTrue(self.ports[0].closed)

    def test_prepare_rejects_g0_and_pump_on_without_changing_pump_state(self):
        session = self.prepare()
        for action in (lambda: session.ask('G0 X200 Y0 Z150 F1000'), lambda: session.pump(True),
                       session.enable_motion):
            with self.assertRaisesRegex(RuntimeError, 'stopped before arm command'):
                action()
        self.assertFalse(session.pump_may_be_on)
        self.assertFalse(any(command.startswith(('G0', 'M2231', 'M17')) for command in self.commands))
        self.command('reset')

    def test_running_requires_live_token_and_respects_deadline(self):
        gate = self.node.gate
        time_now = [10.]
        gate.clock = lambda: time_now[0]
        gate.prepare(deadline_seconds=10., entered_at=7.)
        token = gate.start()
        write = Mock()
        for invalid in (None, '', 'stale'):
            self.assertFalse(gate.permitted(invalid))
            with self.assertRaises(RuntimeError):
                gate.write_if_running(invalid, write)
        gate.write_if_running(token, write)
        time_now[0] = 17.
        self.assertFalse(gate.permitted(token))
        with self.assertRaises(RuntimeError):
            gate.write_if_running(token, write)
        write.assert_called_once()

    def test_running_attaches_before_mission_and_after_prepare_deadline(self):
        session = self.prepare()
        session.active_deadline = session.now() - 1.
        seen = []

        def execute():
            seen.append((self.node.gate.state, self.node.gate.permitted(self.node.robot.run_token)))
            session.ask('G0 X200 Y0 Z150 F1000')
            return Report()

        with patch.object(self.adapter, 'Mission', return_value=SimpleNamespace(run=execute)) as mission:
            self.command('start', run_id=self.node.gate.run_id)
            self.join()
            mission.assert_called_once()
        self.assertEqual(seen, [('running', True)])
        self.assertEqual(self.commands.count('M17'), 1)
        attach = self.commands.index('M17')
        self.assertIn('P2200', self.commands[attach+1:self.commands.index('G0 X200 Y0 Z150 F1000')])
        self.assertIsNone(self.node.gate.active)
        self.assertEqual(self.node.gate.state, 'finished')
        self.assertTrue(self.ports[0].closed)

    def test_mismatched_and_duplicate_start_create_one_worker(self):
        self.prepare()
        original = self.node.thread
        self.command('start', run_id='old-run')
        self.assertIs(self.node.thread, original)
        entered, finish = threading.Event(), threading.Event()

        def execute():
            entered.set()
            finish.wait(2)
            return Report()

        with patch.object(self.adapter, 'Mission', return_value=SimpleNamespace(run=execute)) as mission:
            try:
                self.command('start')
                self.assertTrue(entered.wait(2))
                worker = self.node.thread
                self.command('start')
                self.command('prepare')
                self.command('reset')
                self.assertIs(self.node.thread, worker)
                self.assertEqual(self.node.gate.start_attempts, 1)
                self.assertEqual(len(self.ports), 1)
            finally:
                finish.set()
                self.join()
            mission.assert_called_once()

    def test_stop_during_prepare_cancels_without_closing_from_stop_callback(self):
        entered, finish = threading.Event(), threading.Event()

        def preparing_navigation():
            entered.set()
            finish.wait(2)

        with patch.object(self.adapter.SerialRosRobot, 'prepare_navigation', side_effect=preparing_navigation):
            try:
                self.command('prepare')
                self.assertTrue(entered.wait(2))
                worker = self.node.thread
                self.command('prepare')
                self.assertIs(self.node.thread, worker)
                self.command('stop')
                self.command('reset')
                self.assertEqual(self.node.gate.state, 'stopped')
                self.assertFalse(self.ports[0].closed)
            finally:
                finish.set()
                self.join()
        self.assertFalse(self.node.prepared)
        self.assertIsNone(self.node.gate.active)
        self.assertIsNone(self.node.robot)
        self.assertTrue(self.ports[0].closed)
        self.assertNotIn('M17', self.commands)

    def test_stop_during_port_boot_releases_a_confirmed_empty_port(self):
        make_port = self.arm.SerialPort.side_effect
        entered, finish = threading.Event(), threading.Event()

        def paused_port(path):
            port = make_port(path)
            original = port.read

            def first_read():
                port.read = original
                entered.set()
                finish.wait(2)
                return original()

            port.read = first_read
            return port

        self.arm.SerialPort.side_effect = paused_port
        try:
            self.command('prepare')
            self.assertTrue(entered.wait(2))
            self.command('stop')
            self.assertEqual(self.node.gate.state, 'stopped')
            self.assertFalse(self.ports[0].closed)
        finally:
            finish.set()
            self.join()
        self.assertIn('P2231', self.commands)
        self.assertNotIn('M17', self.commands)
        self.assertTrue(self.ports[0].closed)
        self.assertIsNone(self.node.robot)

    def test_stop_interrupts_arm_reply_wait_and_keeps_safety_stop(self):
        session = self.prepare()
        port = self.ports[0]
        sent, revoked = threading.Event(), threading.Event()
        original_stop, original_write, original_read = self.node.gate.stop, port.write, port.read
        errors = []

        def stop():
            token = original_stop()
            revoked.set()
            return token

        def write(wire):
            original_write(wire)
            if b'G0 ' in wire:
                port.reply = b''
                sent.set()

        def read():
            if sent.is_set() and not revoked.is_set():
                revoked.wait(2)
            return original_read()

        def execute():
            for command in ('G0 X200 Y0 Z150 F1000', 'G0 X201 Y0 Z150 F1000', 'M2231 V1', 'M17'):
                try:
                    session.ask(command)
                except RuntimeError as exc:
                    errors.append(str(exc))
            return Report()

        with patch.object(self.node.gate, 'stop', stop), patch.object(port, 'write', write), \
                patch.object(port, 'read', read), \
                patch.object(self.adapter, 'Mission', return_value=SimpleNamespace(run=execute)):
            self.command('start')
            self.assertTrue(sent.wait(2))
            self.command('stop')
            self.join()
        self.assertEqual(len(errors), 4)
        self.assertIn('stopped while waiting', errors[0])
        self.assertEqual(sum(command.startswith('G0') for command in self.commands), 1)
        self.assertNotIn('M2231 V1', self.commands)
        self.assertIn('M2000', self.commands)
        self.assertEqual(self.node.gate.state, 'stopped')
        self.assertIsNone(self.node.gate.active)

    def test_prepare_serial_initialization_failure_closes_ownership(self):
        make_port = self.arm.SerialPort.side_effect

        def faulty_port(path):
            port = make_port(path)
            port.fault_on = 'M2120 V0'
            return port

        self.arm.SerialPort.side_effect = faulty_port
        try:
            self.command('prepare')
            self.join()
        finally:
            self.arm.SerialPort.side_effect = make_port
        self.assertIn('protocol fault', self.node.error)
        self.assertTrue(self.ports[0].closed)
        self.assertIsNone(self.node.robot)
        self.assertIsNone(self.node.gate.active)
        self.assertEqual(self.node.gate.state, 'stopped')
        self.command('reset')
        self.prepare()
        self.command('reset')

    def test_start_attach_fault_aborts_before_mission_and_releases(self):
        session = self.prepare()
        self.ports[0].fault_on = 'M17'
        with patch.object(self.adapter, 'Mission') as mission:
            self.command('start')
            self.join()
            mission.assert_not_called()
        self.assertIsNone(self.node.gate.active)
        self.assertEqual(self.node.gate.state, 'stopped')
        self.assertFalse(session.initialized)
        self.assertTrue(self.ports[0].closed)

    def test_start_recorder_or_thread_failure_rolls_back_token_and_owner(self):
        for failure in ('recorder', 'thread'):
            with self.subTest(failure=failure):
                self.prepare()
                target, name = (self.recorder, 'start') if failure=='recorder' else (threading.Thread, 'start')
                with patch.object(target, name, side_effect=RuntimeError(failure)):
                    self.command('start')
                self.assertEqual(self.node.gate.state, 'stopped')
                self.assertIsNone(self.node.gate.active)
                self.assertFalse(self.node.prepared)
                self.assertIsNone(self.node.robot)
                self.assertTrue(self.ports[-1].closed)
                self.assertIn(failure, self.node.error)
                self.command('reset')

    def test_second_serial_executor_is_refused_before_open(self):
        if sys.platform == 'win32':
            self.arm.fcntl.flock.side_effect = [None, BlockingIOError('port lease held')]
        self.prepare()
        competing = self.arm.ArmSession(self.folder)
        with self.assertRaises(BlockingIOError):
            competing.open(preparing=True)
        self.assertEqual(len(self.ports), 1)
        self.assertFalse(self.ports[0].closed)
        self.assertIsNone(competing.owner_fd)
        self.command('reset')

    def test_stop_releases_port_without_factory_restart_that_would_attach(self):
        owner = SimpleNamespace(getPid=lambda _: (1, '', 999999999), shutdown=Mock())
        master = SimpleNamespace(getSystemState=lambda _: (1, '', [[], [], []]),
                                 lookupNode=lambda *_: (1, '', 'mock://writer'))
        with patch.object(self.arm.xmlrpc.client, 'ServerProxy',
                          side_effect=lambda uri: owner if uri == 'mock://writer' else master), \
                patch.object(self.arm.subprocess, 'Popen') as spawn:
            session = self.prepare()
            self.assertTrue(session.paused)
            self.command('stop')
            owner.shutdown.assert_called_once()
            spawn.assert_not_called()
        self.assertTrue(self.ports[0].closed)
        self.assertIsNone(session.owner_fd)
        self.assertIn('M2000', self.commands)
        self.assertNotIn('M17', self.commands)

    def test_failed_close_keeps_owner_until_explicit_recovery(self):
        session = self.prepare()
        with patch.object(self.ports[0], 'close', side_effect=RuntimeError('simulated close failure')):
            self.command('stop')
        self.assertTrue(self.node.robot.recovery_required)
        self.assertIsNotNone(session.owner_fd)
        self.command('reset')
        self.command('prepare')
        self.assertEqual(len(self.ports), 1)
        self.assertEqual(self.node.gate.state, 'stopped')
        self.assertTrue(self.node.release_for_recovery(None).success)
        self.assertIsNone(session.owner_fd)
        self.assertIsNone(self.node.robot)

    def test_explicit_recovery_can_turn_off_pump_while_stopped(self):
        session = self.prepare()
        # Simulate interrupted holding without writing a pump-on command.
        session.pump_may_be_on = True
        self.command('stop')
        self.assertTrue(self.node.robot.recovery_required)
        with self.assertRaises(RuntimeError):
            session.pump(False)
        self.assertTrue(self.node.release_for_recovery(None).success)
        self.assertEqual(self.commands.count('M2231 V0'), 1)
        self.assertTrue(self.ports[0].closed)


@dataclass
class Report:
    status: str = 'completed'
    error: str = ''
