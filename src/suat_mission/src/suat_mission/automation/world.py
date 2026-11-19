"""Single-writer physical-instance state; image proximity does not create identity."""
import math
from dataclasses import asdict, dataclass, field
from enum import Enum


class ObjectState(str, Enum):
    LOCALIZED = 'localized'
    RESERVED = 'reserved'
    HELD_CONFIRMED = 'held_confirmed'
    HELD_UNCERTAIN = 'held_uncertain'
    PLACED_CONFIRMED = 'placed_confirmed'
    BLOCKED = 'blocked'


@dataclass(frozen=True)
class SceneSnapshot:
    scene_id: str
    version: int
    observed_at: float  # receiver's monotonic clock, never wall-clock epoch
    frame_id: str
    detections: tuple
    manipulation: dict = field(default_factory=dict)
    source_stamp: float = 0.


@dataclass
class ObjectRecord:
    object_id: str
    label: str
    state: ObjectState = ObjectState.LOCALIZED
    scene_version: int = 0
    zone: str = ''
    evidence: str = ''


class WorldModel:
    def __init__(self, clock, max_age=2.0):
        self.clock = clock
        self.max_age = max_age
        self.scene_id = None
        self.version = 0
        self.objects = {}

    def observe(self, scene):
        if not isinstance(scene, SceneSnapshot):
            raise ValueError('instance observations must be a SceneSnapshot')
        age = self.clock() - scene.observed_at
        if (not scene.scene_id or not scene.frame_id or not math.isfinite(age)
                or not 0 <= age <= self.max_age or type(scene.version) is not int
                or scene.version <= self.version):
            raise ValueError('stale, repeated or invalid scene snapshot')
        if self.scene_id is not None and self.scene_id != scene.scene_id:
            raise ValueError('identity source restarted during this run')
        ids = [d.object_id for d in scene.detections]
        if any(not isinstance(i, str) or not i.strip() for i in ids) or len(set(ids)) != len(ids):
            raise ValueError('missing or duplicate physical object_id')
        # Validate the entire snapshot before mutating any record.
        for det in scene.detections:
            if not det.valid():
                raise ValueError('invalid instance detection')
            if det.object_id in self.objects and self.objects[det.object_id].label != det.label:
                raise ValueError('physical object_id changed class')
        self.scene_id, self.version = scene.scene_id, scene.version
        candidates = []
        for det in scene.detections:
            record = self.objects.setdefault(det.object_id, ObjectRecord(det.object_id, det.label))
            if record.state == ObjectState.LOCALIZED:
                record.scene_version = scene.version
                candidates.append(det)
        return candidates

    def transition(self, object_id, expected, state, evidence='', zone=''):
        record = self.objects[object_id]
        if record.state != expected:
            raise ValueError('illegal instance transition: {} -> {}'.format(record.state, state))
        record.state, record.evidence, record.zone = state, evidence, zone

    def reserve(self, object_id):
        self.transition(object_id, ObjectState.LOCALIZED, ObjectState.RESERVED)

    def uncertain(self, object_id, evidence):
        record = self.objects[object_id]
        # A request may have physically executed even if its result was lost.
        if record.state in (ObjectState.RESERVED, ObjectState.HELD_CONFIRMED):
            record.state, record.evidence = ObjectState.HELD_UNCERTAIN, evidence

    def snapshot(self):
        return [dict(asdict(r), state=r.state.value) for r in self.objects.values()]

    def release_empty(self, object_id, evidence, *, blocked=False):
        """Release a reservation only after positively establishing an empty hand."""
        self.transition(object_id, ObjectState.RESERVED,
                        ObjectState.BLOCKED if blocked else ObjectState.LOCALIZED, evidence)
