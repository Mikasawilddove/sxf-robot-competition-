"""Monotonic station-leg budgets shared by competition and diagnostics."""
from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class NavigationBudget:
    cruise: float = 60.
    # Existing departure and executor stop caps. Heading uses its own window
    # so cruise cannot spend the time needed to turn and confirm a stop.
    departure: float = 25.
    align: float = 20.
    stop: float = 5.

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (self.cruise, self.departure, self.align, self.stop)):
            raise ValueError('navigation phase budgets must be finite and positive')

    def deadline(self, now, hard_deadline):
        return min(hard_deadline, now + self.departure + self.cruise + self.align + 3*self.stop)

    def stage(self, context, name, now):
        reserve = {'departure': self.align + 2*self.stop,
                   'cruise': self.align + self.stop, 'align': 0.}[name]
        # Cap the phase while preserving downstream windows when the parent
        # is shortened by the judge. Each phase owns its stopped handoff.
        deadline = min(now + getattr(self, name) + self.stop, context.deadline - reserve)
        return replace(context, deadline=deadline)
