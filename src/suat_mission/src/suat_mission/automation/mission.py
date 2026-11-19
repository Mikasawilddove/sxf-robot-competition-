"""Competition state machine independent of ROS and the hardware transport."""
import time
import math
import uuid
from dataclasses import dataclass, field
from typing import Dict, Mapping, Sequence

from suat_mission.vision.perception import OBJECTS, select_target, vote_board
from suat_mission.contracts import ActionLedger, Outcome
from suat_mission.automation.world import ObjectState, WorldModel
from suat_mission.automation.scope import select_scope


class MissionError(RuntimeError):
    pass


class ActionFailure(MissionError):
    def __init__(self, result):
        self.result = result
        super().__init__('{} {}: {}'.format(result.context.action, result.outcome.value, result.evidence))


@dataclass
class MissionConfig:
    zones: Sequence[str] = ("B", "C", "D")
    sorting_points: Sequence[str] = ("Sorting_W",)
    board_votes: int = 3
    board_threshold: float = 0.6
    board_axis: str = 'x'
    board_reverse: bool = False
    target_threshold: float = 0.6
    target_thresholds: Mapping[str, float] = field(default_factory=dict)
    target_roi: object = None
    each_count: int = 3
    deadline_seconds: float = 600.0
    action_timeout: float = 60.0
    observation_max_age: float = 2.0
    mission_scope: str = 'full'
    max_pick_retries: int = 1
    finish_margin_seconds: float = 0.0
    initial_cycle_budget_seconds: float = 120.0  # Initial estimate, not measured evidence.
    allow_cake_degradation: bool = True


@dataclass
class MissionReport:
    run_id: str = ''
    status: str = "not_started"
    delivered: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in ("cake",) + OBJECTS})
    board: Dict[str, str] = field(default_factory=dict)
    board_evidence: dict = field(default_factory=dict)
    events: list = field(default_factory=list)
    error: str = ""
    actions: list = field(default_factory=list)
    objects: list = field(default_factory=list)
    skipped: list = field(default_factory=list)
    degraded: bool = False
    cycle_seconds: list = field(default_factory=list)


class Mission:
    def __init__(self, robot, config=None, clock=time.monotonic, run_id=None, started_at=None, event_sink=None,
                 deadline=None):
        self.robot = robot
        self.config = config or MissionConfig()
        self.clock = clock
        self.report = MissionReport(run_id=run_id or uuid.uuid4().hex)
        self.end_time = None
        self.started_at = started_at
        self.event_sink = event_sink
        self.original_deadline = deadline
        self.ledger = ActionLedger(self.report.run_id, clock)
        self.world = WorldModel(clock, self.config.observation_max_age)
        self.attempts = {}

    def _action(self, action, function, *args, object_id=''):
        self._check()
        context = self.ledger.begin(action, min(self.end_time, self.clock() + self.config.action_timeout),
                                    object_id, self.world.version)
        result = function(*args, context=context)
        # Stop/deadline is checked before accepting even a correctly correlated result.
        self._check()
        result = self.ledger.finish(result)
        if result.outcome != Outcome.CONFIRMED:
            raise ActionFailure(result)
        return result

    def _event(self, text):
        self.report.events.append(text)
        if self.event_sink is not None:
            self.event_sink(text)
        self.robot.log(text)

    def _check(self):
        if self.robot.is_stopped():
            raise MissionError("stop requested")
        if self.clock() >= self.end_time:
            raise MissionError("competition deadline reached")

    def _nav(self, name):
        self._check()
        self._event("navigate:" + name)
        self._action('navigate', self.robot.navigate, name, max(0.0, self.end_time - self.clock()))
        self._check()

    def _read_board(self):
        station_selector = getattr(self.robot, 'board_stations', None)
        stations = station_selector() if callable(station_selector) else ('Classification_area',)
        reader = getattr(self.robot, 'read_board_mapping', None)
        attempts = []
        for station in stations:
            try:
                self._nav(station)
            except ActionFailure as exc:
                if exc.result.outcome != Outcome.UNREACHABLE:
                    raise
                attempts.append({'station': station, 'status': 'unreachable'})
                self.report.board_evidence = {'station_attempts': attempts}
                self._event('board_station_unreachable:' + station)
                continue
            if callable(getattr(self.robot, 'prepare_observation', None)):
                self.robot.prepare_observation('board')
            self._check()
            if callable(reader):
                args = {'station': station} if callable(station_selector) else {}
                decision = reader(run_id=self.report.run_id, **args)
                self._check()
                attempts.append({'station': station, 'status': decision.reason})
                self.report.board_evidence = dict(decision.evidence, station=station, station_attempts=attempts)
                if not decision.accepted:
                    self._event('board_station_rejected:' + station + ':' + decision.reason)
                    continue
                self.report.board = dict(decision.mapping)
            else:
                frames = self.robot.read_board_frames()
                self._check()
                self.report.board = dict(vote_board(frames, self.config.zones,
                                                   self.config.board_votes, self.config.board_threshold,
                                                   axis=self.config.board_axis, reverse=self.config.board_reverse))
            if set(self.report.board) != set(OBJECTS) or set(self.report.board.values()) != {'B','C','D'}:
                raise MissionError('incomplete classification-board mapping')
            self._event('board:' + str(self.report.board))
            return
        raise MissionError('no board station passed navigation and formal reading')

    def _find(self, label):
        for waypoint in self.config.sorting_points:
            self._nav(waypoint)
            if callable(getattr(self.robot, 'prepare_observation', None)):
                self.robot.prepare_observation('objects')
            self._check()
            candidates = self.world.observe(self.robot.detect_objects())
            self._check()
            threshold = self.config.target_thresholds.get(label, self.config.target_threshold)
            candidates = [d for d in candidates if d.graspable is not False and not d.occluded_by]
            target = select_target(candidates, label, threshold, self.config.target_roi)
            if target is not None:
                return target
        return None

    def _find_any(self, labels):
        for waypoint in self.config.sorting_points:
            self._nav(waypoint)
            if callable(getattr(self.robot, 'prepare_observation', None)):
                self.robot.prepare_observation('objects')
            self._check()
            candidates = self.world.observe(self.robot.detect_objects())
            candidates = [d for d in candidates if d.label in labels and d.graspable is not False
                          and not d.occluded_by and select_target([d], d.label,
                          self.config.target_thresholds.get(d.label, self.config.target_threshold),
                          self.config.target_roi) is not None]
            if candidates:
                cost = getattr(self.robot, 'delivery_cost', None)
                return min(candidates, key=lambda d: (cost(d, self.report.board[d.label]), -d.score)) if callable(cost) else max(candidates, key=lambda d: d.score)
        return None

    def _empty_confirmed(self):
        method = getattr(self.robot, 'empty_confirmed', None)
        return callable(method) and method() is True

    def _cycle_fits(self):
        recent = self.report.cycle_seconds[-3:]
        estimate = max(recent)*1.15 if recent else self.config.initial_cycle_budget_seconds
        return self.end_time-self.clock() >= estimate+self.config.finish_margin_seconds

    def _deliver(self, label, zone, target=None, cycle_started=None):
        began = self.clock() if cycle_started is None else cycle_started
        target = target or self._find(label)
        if target is None:
            self._event("not_found:" + label)
            return False
        self._check()
        self.world.reserve(target.object_id)
        self.attempts[target.object_id] = self.attempts.get(target.object_id, 0)+1
        self._event("pick:" + label)
        try:
            result = self._action('pick', self.robot.pick, target, label, object_id=target.object_id)
            self.world.transition(target.object_id, ObjectState.RESERVED, ObjectState.HELD_CONFIRMED,
                                  result.evidence)
            self._nav("Collection_" + zone)
            self._check()
            self._event("drop:" + label + ":" + zone)
            result = self._action('place', self.robot.drop, zone, label, object_id=target.object_id)
            self.world.transition(target.object_id, ObjectState.HELD_CONFIRMED, ObjectState.PLACED_CONFIRMED,
                                  result.evidence, zone)
        except ActionFailure as exc:
            if exc.result.context.action == 'pick' and exc.result.outcome in (Outcome.NOT_ACQUIRED, Outcome.UNREACHABLE):
                exhausted = self.attempts[target.object_id] > self.config.max_pick_retries
                self.world.release_empty(target.object_id, exc.result.evidence, blocked=exhausted)
                if exhausted:
                    self.report.skipped.append({'object_id':target.object_id, 'label':label,
                                                'reason':exc.result.evidence})
                self._event(('skip:' if exhausted else 'reobserve:')+target.object_id)
                return False
            self.world.uncertain(target.object_id, str(exc))
            raise
        except Exception as exc:
            self.world.uncertain(target.object_id, str(exc))
            raise
        self.report.delivered[label] += 1
        self.report.cycle_seconds.append(self.clock()-began)
        return True

    def run(self):
        if self.report.status != "not_started":
            raise RuntimeError("mission instances are single-use")
        self.end_time = (self.original_deadline if self.original_deadline is not None else
                         (self.clock() if self.started_at is None else self.started_at) + self.config.deadline_seconds)
        self.report.status = "running"
        try:
            if type(self.end_time) not in (int, float) or not math.isfinite(self.end_time):
                raise ValueError('invalid original competition deadline')
            if (not all(math.isfinite(v) and v > 0 for v in
                        (self.config.deadline_seconds, self.config.action_timeout, self.config.observation_max_age))
                    or type(self.config.each_count) is not int or self.config.each_count < 0
                    or type(self.config.board_votes) is not int or self.config.board_votes < 1):
                raise ValueError("invalid mission timeout, count or board vote threshold")
            if any(label not in ("cake",) + OBJECTS or not 0 <= float(value) <= 1
                   for label, value in self.config.target_thresholds.items()):
                raise ValueError("invalid per-class target threshold")
            if (type(self.config.max_pick_retries) is not int or self.config.max_pick_retries < 0
                    or not math.isfinite(self.config.finish_margin_seconds) or self.config.finish_margin_seconds < 0
                    or not math.isfinite(self.config.initial_cycle_budget_seconds)
                    or self.config.initial_cycle_budget_seconds <= 0):
                raise ValueError('invalid retry or cycle-budget configuration')
            scope = select_scope(self.config.mission_scope, self.config.sorting_points)
            if hasattr(self.robot, 'set_deadline'):
                self.robot.set_deadline(self.end_time)
            self._check()
            self._read_board()
            cake_delivered = False
            for attempt in range(self.config.max_pick_retries+1):
                if self._deliver('cake', 'A'):
                    cake_delivered = True
                    break
            if not cake_delivered:
                if scope.name == 'cake-only' or not self.config.allow_cake_degradation or not self._empty_confirmed():
                    raise MissionError('cake failed; empty-hand continuation is not established')
                self.report.degraded = True
                self._event('degraded:cake_not_delivered')
            while scope.regular_labels:
                self._check()
                labels = {label for label in scope.regular_labels
                          if self.report.delivered[label] < self.config.each_count}
                if not labels:
                    break
                if not self._cycle_fits():
                    self._event('finish:no_new_cycle_within_budget')
                    break
                began = self.clock()
                target = self._find_any(labels)
                if target is None:
                    self._event('finish:no_remaining_feasible_target')
                    break
                self._deliver(target.label, self.report.board[target.label], target=target, cycle_started=began)
            self.report.status = ('completed' if cake_delivered and all(
                self.report.delivered[label] == self.config.each_count for label in scope.regular_labels) else 'partial')
        except (MissionError, ValueError) as exc:
            self.report.status = "aborted"
            self.report.error = str(exc)
            self._event("abort:" + str(exc))
        except Exception as exc:
            self.report.status = "aborted"
            self.report.error = "unexpected {}: {}".format(type(exc).__name__, exc)
            self._event("abort:" + self.report.error)
        finally:
            self.ledger.cancel(self.report.error or self.report.status)
            self.report.actions = list(self.ledger.records)
            self.report.objects = self.world.snapshot()
            self.robot.stop_motion()
        return self.report
