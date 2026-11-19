"""Offline SwiftPro feedback candidate, based on an identified public firmware.

No device is opened, no ROS topic is published, and no robot capability is certified.
"""
from dataclasses import asdict, dataclass
import math
import re

from suat_mission.contracts import ActionContext


@dataclass(frozen=True)
class ProtocolFrame:
    kind: str
    values: tuple
    first_byte_at: float
    received_at: float
    raw: str
    sequence: int = None
    error_code: int = None
    payload: tuple = ()


class SwiftProParser:
    """Incremental LF-framed byte parser; timestamps are host monotonic times."""
    def __init__(self, max_line_bytes=256):
        self.buffer = bytearray()
        self.first_byte_at = None
        self.max_line_bytes = max_line_bytes
        self.discarding = False

    def feed(self, data, received_at):
        result = []
        for byte in data:
            if self.first_byte_at is None:
                self.first_byte_at = received_at
            if byte == 10:
                try:
                    line = self.buffer.decode('ascii').rstrip('\r')
                except UnicodeDecodeError:
                    line = '<non-ascii>'
                tokens = line.split()
                kind, sequence, error, payload, values = 'unknown', None, None, (), ()
                if tokens and tokens[0] == '@3':
                    payload = tuple(tokens[1:])
                    try:
                        values = _numbers(payload, 'XYZR')
                        kind = 'position_report'
                    except ValueError:
                        kind = 'invalid'
                elif tokens and tokens[0] == '@9':
                    kind, payload = 'execution_report', tuple(tokens[1:])
                    if payload != ('V0',):
                        kind = 'invalid'
                elif tokens and re.fullmatch(r'\$[1-9][0-9]*', tokens[0]):
                    sequence = int(tokens[0][1:])
                    payload = tuple(tokens[2:])
                    if len(tokens) > 1 and tokens[1] == 'ok':
                        kind = 'reply'
                    elif len(tokens) > 1 and re.fullmatch(r'E[0-9]+', tokens[1]):
                        kind, error = 'error', int(tokens[1][1:])
                elif tokens and tokens[0] == 'ok':
                    kind, payload = 'unnumbered_reply', tuple(tokens[1:])
                elif tokens and re.fullmatch(r'E[0-9]+', tokens[0]):
                    kind, error, payload = 'unnumbered_error', int(tokens[0][1:]), tuple(tokens[1:])
                if self.discarding or line == '<non-ascii>' or any(ord(c) < 32 for c in line):
                    kind, values, payload = 'invalid', (), ()
                if line or self.discarding:
                    result.append(ProtocolFrame(kind, values, self.first_byte_at, received_at, line,
                                                sequence, error, payload))
                self.buffer.clear()
                self.first_byte_at = None
                self.discarding = False
            else:
                if len(self.buffer) < self.max_line_bytes and not self.discarding:
                    self.buffer.append(byte)
                else:
                    self.discarding = True
        return result


def _numbers(tokens, names):
    if len(tokens) != len(names) or any(token[:1] != name for token, name in zip(tokens, names)):
        raise ValueError('wrong field names or field count')
    try:
        values = tuple(float(token[1:]) for token in tokens)
    except (ValueError, TypeError):
        raise ValueError('invalid numeric field')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('non-finite numeric field')
    return values


@dataclass(frozen=True)
class ArrivalPolicy:
    """Caller-supplied tolerances in the declared target space; no field defaults."""
    error_tolerance: float
    stability_tolerance: float
    stable_samples: int
    stable_duration: float

    def __post_init__(self):
        if (not all(_positive(value) for value in (self.error_tolerance, self.stability_tolerance,
                                                   self.stable_duration))
                or type(self.stable_samples) is not int or self.stable_samples < 2):
            raise ValueError('arrival policy requires positive finite limits, at least two samples and a time span')


class SingleOwnerChannel:
    """Bind one session to an existing transport; never open or reconnect a port.

    The main driver must create exactly one channel for its actual port. This
    cannot police another process or someone opening the same device elsewhere.
    Transport contract: write(bytes)->count, read_available()->bytes, both bounded.
    """
    def __init__(self, transport):
        self.transport = transport
        self.owner = None

    def claim(self, owner):
        if self.owner is not None:
            raise ValueError('channel already has an owner')
        self.owner = owner

    def write(self, owner, payload):
        if owner is not self.owner:
            raise ValueError('not the channel owner')
        return self.transport.write(payload)

    def read_available(self, owner):
        if owner is not self.owner:
            raise ValueError('not the channel owner')
        return self.transport.read_available()


class ArmFeedbackSession:
    """Single-owner, transport-injected candidate. It never certifies hardware.

    Constructing the session performs no I/O. Only an explicit send/poll uses the
    injected owner. The caller remains responsible for permission to issue motion.
    """
    def __init__(self, channel, clock, *, max_observation_age):
        if not _positive(max_observation_age):
            raise ValueError('a finite positive observation age is required')
        channel.claim(self)
        self.channel = channel
        self.clock = clock
        self.max_age = max_observation_age
        self.parser = SwiftProParser()
        self.sequence = 0
        self.pending = None
        self.records = []
        self.events = []
        self.context = None
        self.command_state = 'unknown'
        self.arrival = 'unknown'
        self.motion_sequence = None
        self.motion_accepted = False
        self.motion_sent_at = None
        self.measurement = None
        self.samples = []
        self.internal_position = None
        self.execution = None
        self.async_position = None
        self.pump_current = None
        self.health = 'active'
        self.reason = ''
        self.retired_goals = set()
        self.requested_feedrate = None
        self.last_time = None
        self.versions = {}

    def begin_action(self, context, target, target_kind, policy):
        now = self._check_time()
        if self.health != 'active' or self.context is not None or self.pending is not None:
            raise ValueError('session is unavailable or has an active action')
        if (not isinstance(context, ActionContext)
                or not all(isinstance(value, str) and value.strip() for value in
                           (context.run_id, context.goal_id, context.action))
                or type(context.scene_version) is not int or context.scene_version < 0
                or type(context.deadline) not in (int, float) or not math.isfinite(context.deadline)
                or context.deadline <= now or (context.run_id, context.goal_id) in self.retired_goals):
            raise ValueError('invalid, expired or reused action context')
        if (target_kind not in ('joint_deg', 'cartesian_mm') or not isinstance(policy, ArrivalPolicy)
                or not isinstance(target, (list, tuple)) or len(target) != 3
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in target)):
            raise ValueError('finite 3-axis target, declared space and explicit policy required')
        self.context = context
        self.target = tuple(target)
        self.target_kind = target_kind
        self.policy = policy
        self.arrival = 'pending'
        self.motion_sequence = None
        self.motion_accepted = False
        self.motion_sent_at = None
        self.measurement = None
        self.samples = []
        self.internal_position = None
        self.execution = None
        self.async_position = None
        self.pump_current = None
        self.command_state = 'unknown'
        self.requested_feedrate = None

    def end_action(self, context):
        """Archive evidence, without declaring that any physical action succeeded."""
        self._check_time()
        if context != self.context or self.pending is not None or self.health != 'active':
            raise ValueError('cannot end a mismatched, pending or failed action')
        result = self.assessment()
        self.retired_goals.add((context.run_id, context.goal_id))
        self.context = None
        self.arrival = 'unknown'
        return result

    def cancel(self, reason='stop requested'):
        """Latch software output only. No physical stop, hold or pump command."""
        self._fail('cancelled', str(reason) + '; physical stop/hold and holding are unknown')

    def send(self, command, context, *, timeout):
        now = self._check_time()
        if self.health != 'active':
            raise ValueError('feedback session is ' + self.health)
        if context != self.context:
            raise ValueError('action context mismatch')
        if not _positive(timeout):
            raise ValueError('a finite positive request timeout is required')
        _validate_command(command)
        if self.pending is not None:
            raise ValueError('one request at a time')
        self.sequence += 1
        payload = ('#{} {}\r\n'.format(self.sequence, command)).encode('ascii')
        record = dict(sequence=self.sequence, command=command, context=asdict(context),
                      sent_at=now, deadline=min(now + timeout, context.deadline), state='sent')
        self.pending = record
        self.records.append(record)
        try:
            count = self.channel.write(self, payload)
            record['written_bytes'] = count
            if type(count) is not int or count != len(payload):
                raise OSError('partial or unknown write count')
        except OSError as exc:
            self._fail('disconnected', 'write failed; no retry and physical state unknown')
            raise ConnectionError('arm write failed or incomplete') from exc
        self.command_state = 'sent'
        if command.startswith('G0 '):
            self.motion_sequence = self.sequence
            self.motion_sent_at = now
            self.motion_accepted = False
            self.samples = []
            self.arrival = 'pending'
            self.requested_feedrate = _numbers(command.split()[1:], 'XYZF')[-1]
        self._check_time()
        return dict(record)

    def poll(self):
        self._check_time()
        if self.health == 'disconnected':
            return []
        try:
            data = self.channel.read_available(self)
        except OSError:
            self._fail('disconnected', 'read failed; no reconnect and physical state unknown')
            return []
        frames = self.parser.feed(data, self._check_time())
        for frame in frames:
            self.events.append(asdict(frame))
            if self.health != 'active':
                continue
            if frame.kind in ('unknown', 'invalid', 'unnumbered_error'):
                self._fail('failed', 'unrecognized, malformed or uncorrelated fault report')
                continue
            if frame.kind == 'position_report':
                self.async_position = dict(source='@3', values=frame.values,
                                           received_at=frame.received_at, first_byte_at=frame.first_byte_at,
                                           usable_for_arrival=False,
                                           reason='no controller timestamp or request sequence')
            elif frame.kind == 'execution_report':
                self.execution = dict(source='@9', state='idle', received_at=frame.received_at)
            if (self.pending is not None and frame.sequence == self.pending['sequence']
                    and frame.first_byte_at >= self.pending['sent_at']):
                record = self.pending
                if frame.kind == 'error':
                    record['error_code'] = frame.error_code
                    self._fail('failed', 'controller E{}'.format(frame.error_code))
                elif frame.kind == 'reply':
                    try:
                        self._accept_reply(record, frame)
                    except ValueError as exc:
                        self._fail('failed', 'invalid matched reply: ' + str(exc))
                        continue
                    record['state'] = 'accepted'
                    self.pending = None
                    self.command_state = 'accepted'
        return frames

    def _accept_reply(self, record, frame):
        command = record['command']
        if record['sequence'] == self.motion_sequence:
            if frame.payload:
                raise ValueError('motion ACK has unexpected payload')
            self.motion_accepted = True
        elif command == 'P2200':
            values = _numbers(frame.payload, 'BLR')
            self.measurement = dict(source='P2200', values=values,
                                    query_sequence=frame.sequence, received_at=frame.received_at)
            if (self.motion_accepted and record['sent_at'] > self.motion_sent_at
                    and self.target_kind == 'joint_deg'):
                self._observe_arrival(values, frame.received_at)
        elif command == 'P2220':
            self.internal_position = dict(source='P2220', values=_numbers(frame.payload, 'XYZ'),
                                          received_at=frame.received_at, usable_for_arrival=False)
        elif command == 'M2200':
            value, = _numbers(frame.payload, 'V')
            if value not in (0, 1):
                raise ValueError('invalid moving state')
            self.execution = dict(source='M2200', state='moving' if value else 'idle',
                                  received_at=frame.received_at)
        elif command == 'P2231':
            value, = _numbers(frame.payload, 'V')
            states = {0: 'stopped_or_low_current', 1: 'working_current', 2: 'grabbing_current'}
            if value not in states:
                raise ValueError('invalid pump current state')
            self.pump_current = dict(source='P2231', state=states[value], raw_value=int(value),
                                     query_sequence=frame.sequence, received_at=frame.received_at,
                                     scope='pump_current_inference_only')
        elif command.startswith('P2206 '):
            values = _numbers(frame.payload, 'V')
            self.measurement = dict(source='P2206', values=values, axis=int(command[-1]),
                                    query_sequence=frame.sequence, received_at=frame.received_at,
                                    usable_for_arrival=False, reason='single joint is not a simultaneous full pose')
            self.samples = []
            self.arrival = 'pending'
        elif command in ('P2201', 'P2202', 'P2203', 'P2204', 'P2205'):
            if len(frame.payload) != 1 or not frame.payload[0].startswith('V') or len(frame.payload[0]) < 2:
                raise ValueError('invalid version/identity reply')
            self.versions[command] = dict(value=frame.payload[0][1:], query_sequence=frame.sequence,
                                          received_at=frame.received_at, capability_verified=False)
        elif frame.payload:
            raise ValueError('ACK has unexpected payload')

    def _fail(self, state, reason):
        self.health = state
        self.reason = reason
        self.command_state = state
        self.arrival = state
        if self.pending is not None:
            self.pending['state'] = state
        self.pending = None
        self.motion_accepted = False
        self.samples = []
        self.measurement = None
        self.pump_current = None
        self.execution = None
        self.internal_position = None
        self.async_position = None

    def _check_time(self):
        now = self.clock()
        if (type(now) not in (int, float) or not math.isfinite(now)
                or (self.last_time is not None and now < self.last_time)):
            self._fail('failed', 'invalid or reversed monotonic clock')
            raise ValueError('invalid or reversed monotonic clock')
        self.last_time = now
        if self.health != 'active':
            return now
        if self.pending is not None and now >= self.pending['deadline']:
            self._fail('timeout', 'request deadline reached; physical motion and holding remain unknown')
        elif self.context is not None and now >= self.context.deadline:
            self._fail('timeout', 'action deadline reached; physical motion and holding remain unknown')
        if self.health == 'active':
            for name in ('measurement', 'pump_current', 'execution', 'internal_position', 'async_position'):
                observation = getattr(self, name)
                if observation is not None and not 0 <= now - observation['received_at'] < self.max_age:
                    setattr(self, name, None)
            if self.samples and not 0 <= now - self.samples[-1][0] < self.max_age:
                self.samples = []
                self.arrival = 'unknown'
        return now

    def _observe_arrival(self, values, observed_at):
        self.arrival = 'pending'
        if math.dist(values, self.target) > self.policy.error_tolerance:
            self.samples = []
            self.arrival = 'pending'
            return
        if any(math.dist(values, sample[1]) > self.policy.stability_tolerance for sample in self.samples):
            self.samples = []
        self.samples.append((observed_at, values))
        if (len(self.samples) >= self.policy.stable_samples
                and observed_at - self.samples[0][0] >= self.policy.stable_duration):
            self.arrival = 'candidate'

    def assessment(self):
        self._check_time()
        return dict(context=asdict(self.context) if self.context else None,
                    session_state=self.health, reason=self.reason,
                    stop_hold_capability='unknown',
                    stage_speed=dict(requested_feedrate_mm_min=self.requested_feedrate,
                                     execution_verified=False),
                    command_state=self.command_state, arrival=self.arrival, hardware_ready=False,
                    measurement=dict(self.measurement) if self.measurement else None,
                    versions={command: dict(value) for command, value in self.versions.items()},
                    internal_position=dict(self.internal_position) if self.internal_position else None,
                    execution_state=self.execution['state'] if self.execution else 'unknown',
                    async_position=dict(self.async_position) if self.async_position else None,
                    pump_current=dict(self.pump_current) if self.pump_current else dict(state='unknown'),
                    feedback_source_verified=False, holding='unknown', release='unknown', placement='unknown')


def _positive(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def _validate_command(command):
    if (not isinstance(command, str) or not command or len(command) > 200
            or any(ord(char) < 32 or ord(char) > 126 for char in command)):
        raise ValueError('invalid command encoding')
    tokens = command.split(' ')
    if command in ('P2200', 'P2201', 'P2202', 'P2203', 'P2204', 'P2205', 'P2220', 'M2200', 'P2231',
                   'M2231 V0', 'M2231 V1') or re.fullmatch(r'P2206 N[0-3]', command):
        return
    if tokens[0] == 'G0':
        values = _numbers(tokens[1:], 'XYZF')
        if values[-1] > 0:
            return
    raise ValueError('unsupported command; explicit XYZ and positive F are required for a move')
