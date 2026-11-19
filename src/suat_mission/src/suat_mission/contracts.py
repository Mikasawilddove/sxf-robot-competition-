"""Transport-free action correlation. A truthy value is never completion evidence."""
import math
from dataclasses import asdict, dataclass
from enum import Enum


class Outcome(str, Enum):
    CONFIRMED = 'confirmed'
    FAILED = 'failed'
    UNKNOWN = 'unknown'
    CANCELLED = 'cancelled'
    TIMEOUT = 'timeout'
    NOT_ACQUIRED = 'not_acquired'
    UNREACHABLE = 'unreachable'


@dataclass(frozen=True)
class ActionContext:
    run_id: str
    goal_id: str
    action: str
    object_id: str
    scene_version: int
    deadline: float


@dataclass(frozen=True)
class ActionResult:
    context: ActionContext
    outcome: Outcome
    evidence: str = ''

    def record(self):
        result = asdict(self)
        result['outcome'] = self.outcome.value
        return result


class ActionLedger:
    """One in-flight action, consumed once, with a monotonic absolute deadline."""
    def __init__(self, run_id, clock):
        self.run_id = run_id
        self.clock = clock
        self.sequence = 0
        self.pending = None
        self.cancelled = False
        self.records = []

    def begin(self, action, deadline, object_id='', scene_version=0):
        if self.cancelled or self.pending is not None:
            raise ValueError('action ledger is cancelled or busy')
        if not math.isfinite(deadline) or deadline <= self.clock():
            raise ValueError('action deadline has expired')
        self.sequence += 1
        self.pending = ActionContext(self.run_id, '{}:{:04d}'.format(self.run_id, self.sequence),
                                     action, object_id, scene_version, deadline)
        return self.pending

    def finish(self, result):
        current = self.pending
        if current is None:
            raise ValueError('unsolicited, duplicate or cancelled action result')
        if not isinstance(result, ActionResult) or result.context != current:
            raise ValueError('action result does not match run/goal/object/scene/deadline')
        if not isinstance(result.outcome, Outcome):
            raise ValueError('invalid action outcome')
        if self.clock() >= current.deadline:
            result = ActionResult(current, Outcome.TIMEOUT, 'result arrived after deadline')
        elif result.outcome == Outcome.CONFIRMED and not result.evidence.strip():
            result = ActionResult(current, Outcome.UNKNOWN, 'missing completion evidence')
        self.pending = None
        self.records.append(result.record())
        return result

    def cancel(self, reason='mission ended'):
        self.cancelled = True
        if self.pending is not None:
            self.records.append(ActionResult(self.pending, Outcome.CANCELLED, reason).record())
            self.pending = None
