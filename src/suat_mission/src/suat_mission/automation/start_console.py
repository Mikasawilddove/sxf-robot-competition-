"""Single fresh terminal Enter, bound to the prepared run and scope."""
import threading


class SingleEnterConsole:
    def __init__(self, scope):
        if scope not in ('cake-only', 'full'):
            raise ValueError('invalid mission scope')
        self.scope = scope
        self.state = {}
        self.used = False
        self.lock = threading.RLock()

    def observe(self, state):
        with self.lock:
            self.state = dict(state)

    def ready(self):
        with self.lock:
            return (self.state.get('prepared') is True and self.state.get('scope') == self.scope
                    and bool(self.state.get('run_id')) and self.state.get('state') == 'idle'
                    and not self.state.get('error'))

    def start_once(self, discard_pending_input, read_line, publish):
        with self.lock:
            if self.used or not self.ready():
                raise ValueError('console is not ready or its start was already consumed')
            run_id = self.state['run_id']
            self.used = True
        discard_pending_input()
        if read_line() != '':
            raise ValueError('only one fresh Enter is accepted')
        with self.lock:
            if not self.ready() or self.state['run_id'] != run_id:
                raise ValueError('prepared run changed while waiting for Enter')
            publish({'command': 'start', 'run_id': run_id})

    def start_after_enter(self, run_id, entered_at, publish):
        """Consume the launcher's earlier terminal Enter for this prepared run."""
        with self.lock:
            if self.used or not self.ready() or self.state['run_id'] != run_id:
                raise ValueError('prepared run changed or the terminal Enter was already consumed')
            if entered_at is None or self.state.get('budget_anchor_monotonic') != entered_at:
                raise ValueError('mission node did not acknowledge the original Enter time; deploy matching code')
            self.used = True
            publish({'command': 'start', 'run_id': run_id})
