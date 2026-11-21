"""A process-local start/reset latch. This is not a physical emergency stop."""
import threading
import math
import time
import uuid


class RunGate:
    def __init__(self, clock=time.monotonic):
        self._lock = threading.Lock()
        self.active = None
        self.state = 'idle'
        self.clock = clock
        self.run_id = None
        self.started_at = None
        self.deadline = None
        self.deadline_seconds = 600.0
        self.startup_delay = 0.0
        self.stop_margin = 0.0
        self.start_attempts = 0
        self.budget_anchor = None
        self.clock_already_running = False

    def prepare(self, new_run=False, deadline_seconds=600.0, startup_delay=0.0, stop_margin=0.0,
                clock_already_running=False, entered_at=None):
        """Only an explicit new preparation clears the previous match's budget."""
        values = (deadline_seconds, startup_delay, stop_margin)
        if (any(type(value) not in (int, float) or not math.isfinite(value) for value in values)
                or deadline_seconds <= 0 or startup_delay < 0 or stop_margin < 0
                or deadline_seconds <= startup_delay + stop_margin or type(clock_already_running) is not bool):
            raise ValueError('invalid mission deadline, startup delay or stop margin')
        with self._lock:
            if self.active is not None or self.state != 'idle':
                raise ValueError('reset is required before preparation')
            if self.started_at is not None and not new_run:
                return self.run_id
            now = self.clock()
            if entered_at is not None:
                if (type(entered_at) not in (int, float) or not math.isfinite(entered_at)
                        or not 0 <= entered_at <= now):
                    raise ValueError('invalid terminal Enter monotonic timestamp')
                if entered_at + deadline_seconds - startup_delay - stop_margin <= now:
                    raise ValueError('original competition deadline reached during startup')
            self.run_id = uuid.uuid4().hex
            self.started_at = self.deadline = None
            self.deadline_seconds, self.startup_delay, self.stop_margin = values
            self.start_attempts = 0
            self.clock_already_running = clock_already_running
            self.budget_anchor = entered_at if entered_at is not None else (now if clock_already_running else None)
            return self.run_id

    def start(self):
        with self._lock:
            if self.state != 'idle' or self.active is not None:
                raise ValueError('reset is required, or a worker is still active')
            now = self.clock()
            proposed_deadline = ((now if self.budget_anchor is None else self.budget_anchor)
                                 + self.deadline_seconds - self.startup_delay - self.stop_margin)
            if now >= (self.deadline if self.deadline is not None else proposed_deadline):
                raise ValueError('original competition deadline reached')
            self.active = uuid.uuid4().hex
            if self.run_id is None:
                self.run_id = self.active
            if self.started_at is None:
                self.started_at = now
                self.deadline = proposed_deadline
            self.start_attempts += 1
            self.state = 'running'
            return self.active

    def stop(self):
        with self._lock:
            self.state = 'stopped'
            return self.active

    def reset(self):
        with self._lock:
            if self.active is not None:
                raise ValueError('cannot reset until the active worker has ended')
            self.state = 'idle'

    def permitted(self, token):
        with self._lock:
            return self._permitted(token)

    def _permitted(self, token):
        return (bool(token) and self.active == token and self.state == 'running'
                and self.deadline is not None and self.clock() < self.deadline)

    def write_if_running(self, token, write):
        """Serialize one bounded device write with revocation, not its reply wait."""
        with self._lock:
            if not self._permitted(token):
                raise RuntimeError('arm command requires an active run token')
            write()

    def finish(self, token):
        with self._lock:
            if not token or self.active != token:
                return False
            self.active = None
            if self.state == 'running':
                self.state = 'finished'
            return True
