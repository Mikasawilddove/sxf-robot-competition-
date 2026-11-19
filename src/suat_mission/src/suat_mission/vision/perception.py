"""Turn object detections into a conservative board reading and pick target."""
from collections import Counter
from dataclasses import dataclass, field
from math import isfinite
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

OBJECTS = ("teddybear", "clock", "banana")
ALL_ITEMS = OBJECTS + ("cake",)
COCO_IDS = {88: "teddybear", 85: "clock", 52: "banana", 61: "cake"}


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    x: float
    y: float
    width: float = 0.0
    height: float = 0.0
    object_id: str = ''  # empty for board-only detections; never infer from class
    surface: dict = field(default_factory=dict, compare=False)
    occluded_by: tuple = ()
    graspable: object = None

    def valid(self) -> bool:
        return (self.label in ALL_ITEMS and
                all(isfinite(v) for v in (self.score, self.x, self.y, self.width, self.height)) and
                0 <= self.score <= 1 and self.width >= 0 and self.height >= 0)


def board_order(detections: Iterable[Detection], threshold: float = 0.6,
                min_separation: float = 25.0, axis: str = 'x',
                reverse: bool = False) -> Optional[Tuple[str, str, str]]:
    """Return exactly one detection per class ordered along the chosen image axis.

    Multiple detections of the same class are ambiguous and rejected. The camera
    view must be aligned with the board and cropped externally if necessary.
    """
    if axis not in ('x', 'y'):
        raise ValueError('board axis must be x or y')
    selected = [d for d in detections if d.valid() and d.label in OBJECTS and d.score >= threshold]
    if len(selected) != 3 or {d.label for d in selected} != set(OBJECTS):
        return None
    selected.sort(key=lambda d: getattr(d, axis), reverse=reverse)
    if any(abs(getattr(b, axis) - getattr(a, axis)) < min_separation for a, b in zip(selected, selected[1:])):
        return None
    return tuple(d.label for d in selected)


def vote_board(frames: Sequence[Sequence[Detection]], zones: Sequence[str],
               min_votes: int = 3, threshold: float = 0.6,
               min_separation: float = 25.0, axis: str = 'x', reverse: bool = False) -> Mapping[str, str]:
    """Require a unique, sufficiently frequent board order across distinct frames."""
    if len(zones) != 3 or set(zones) != {"B", "C", "D"}:
        raise ValueError("board slots must be a permutation of B, C, D")
    votes = Counter(order for order in (board_order(frame, threshold, min_separation, axis, reverse)
                                        for frame in frames) if order is not None)
    if not votes:
        raise ValueError("no valid classification board frames")
    ranked = votes.most_common()
    if ranked[0][1] < min_votes or ranked[0][1] * 2 <= sum(votes.values()):
        raise ValueError("classification board votes are insufficient or lack a majority")
    return dict(zip(ranked[0][0], zones))


def select_target(detections: Iterable[Detection], label: str, threshold: float = 0.6,
                  roi: Optional[Tuple[float, float, float, float]] = None) -> Optional[Detection]:
    """Pick the highest-confidence candidate within the configured image ROI."""
    candidates: List[Detection] = []
    for d in detections:
        if not d.valid() or d.label != label or d.score < threshold:
            continue
        if roi is not None and not (roi[0] <= d.x <= roi[2] and roi[1] <= d.y <= roi[3]):
            continue
        candidates.append(d)
    return max(candidates, key=lambda d: d.score, default=None)


def stable_detections(frames: Sequence[Sequence[Detection]], min_frames: int = 3,
                      threshold: float = 0.6, max_distance: float = 35.0) -> List[Detection]:
    """Associate detections by class and proximity, requiring distinct frames."""
    tracks = []
    for frame_index, frame in enumerate(frames):
        for det in sorted(frame, key=lambda d: d.score, reverse=True):
            if not det.valid() or det.score < threshold:
                continue
            choices = []
            for track in tracks:
                last = track[-1][1]
                if (track[-1][0] == frame_index or last.label != det.label
                        or last.object_id != det.object_id):
                    continue
                dist = ((last.x - det.x) ** 2 + (last.y - det.y) ** 2) ** 0.5
                if dist <= max_distance:
                    choices.append((dist, track))
            if choices:
                min(choices, key=lambda pair: pair[0])[1].append((frame_index, det))
            else:
                tracks.append([(frame_index, det)])
    result = []
    for track in tracks:
        if len(track) >= min_frames:
            ds = [entry[1] for entry in track]
            count = float(len(ds))
            result.append(Detection(ds[0].label, sum(d.score for d in ds) / count,
                                    sum(d.x for d in ds) / count, sum(d.y for d in ds) / count,
                                    sum(d.width for d in ds) / count, sum(d.height for d in ds) / count,
                                    ds[0].object_id))
    return result
