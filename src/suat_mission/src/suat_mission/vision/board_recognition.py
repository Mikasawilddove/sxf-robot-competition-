"""Board-only geometry and evidence decoding, independent of ROS and OpenCV.

Coordinates supplied to the reader default to ORIGINAL image pixels. Rectified
detectors explicitly select normalized_board coordinates, retaining exact quads.
Corners are in
normalized-board order (upper-left, upper-right, lower-right, lower-left), not
inferred from the class permutation.  No detector score is a probability.
"""
from collections import Counter
from copy import deepcopy
from dataclasses import asdict, dataclass, replace
from math import isfinite

import numpy as np

from suat_mission.vision.perception import OBJECTS


def _evidence_values(value):
    """Keep invalid raw numbers explicit, without emitting non-standard JSON NaN."""
    if isinstance(value, (float, np.floating)):
        return float(value) if isfinite(value) else repr(float(value))
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, dict):
        return {key: _evidence_values(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return type(value)(_evidence_values(item) for item in value)
    return deepcopy(value)


def _project(matrix, points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not np.isfinite(points).all():
        raise ValueError('non-finite board coordinate')
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    if np.any(np.abs(homogeneous[:, 2]) < 1.e-12):
        raise ValueError('board coordinate projects to infinity')
    return homogeneous[:, :2] / homogeneous[:, 2:3]


@dataclass(frozen=True)
class BoardGeometry:
    image_size: tuple
    corners: tuple
    normalized_size: tuple
    geometry_id: str

    def __post_init__(self):
        for size in (self.image_size, self.normalized_size):
            if len(size) != 2 or any(type(value) is not int or value <= 0 for value in size):
                raise ValueError('image and normalized sizes must be positive integer pixels')
        if not isinstance(self.geometry_id, str) or not self.geometry_id.strip():
            raise ValueError('board geometry needs a version or evidence identity')
        corners = np.asarray(self.corners, dtype=float)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            raise ValueError('board requires four finite original-image corners')
        if np.any(corners < 0) or np.any(corners > np.asarray(self.image_size)):
            raise ValueError('board corners fall outside original image')
        edges = np.roll(corners, -1, axis=0)-corners
        cross = edges[:, 0]*np.roll(edges[:, 1], -1)-edges[:, 1]*np.roll(edges[:, 0], -1)
        if not (np.all(cross > 1.e-9) or np.all(cross < -1.e-9)):
            raise ValueError('board corners must form a non-folded convex quadrilateral')
        object.__setattr__(self, 'image_size', tuple(self.image_size))
        object.__setattr__(self, 'normalized_size', tuple(self.normalized_size))
        object.__setattr__(self, 'corners', tuple(tuple(point) for point in corners.tolist()))
        width, height = self.normalized_size
        target = ((0., 0.), (width, 0.), (width, height), (0., height))
        rows, values = [], []
        for (x, y), (u, v) in zip(self.corners, target):
            rows.extend(((x, y, 1., 0., 0., 0., -u*x, -u*y),
                         (0., 0., 0., x, y, 1., -v*x, -v*y)))
            values.extend((u, v))
        matrix = np.append(np.linalg.solve(rows, values), 1.).reshape(3, 3)
        object.__setattr__(self, '_image_to_board', matrix)
        object.__setattr__(self, '_board_to_image', np.linalg.inv(matrix))

    def to_board(self, points):
        return _project(self._image_to_board, points)

    def to_image(self, points):
        return _project(self._board_to_image, points)

    def original_detection(self, detection):
        """Enclose a normalized match in an original-image, axis-aligned bbox.

        Perspective enclosure is conservative: the reader may reject a box that
        crosses a slot after this conversion.  The transform remains in evidence.
        """
        if not detection.valid():
            raise ValueError('invalid normalized detection')
        left, right = detection.x-detection.width/2, detection.x+detection.width/2
        top, bottom = detection.y-detection.height/2, detection.y+detection.height/2
        polygon = self.to_image([(left, top), (right, top), (right, bottom), (left, bottom)])
        low, high = polygon.min(axis=0), polygon.max(axis=0)
        return replace(detection, x=float((low[0]+high[0])/2), y=float((low[1]+high[1])/2),
                       width=float(high[0]-low[0]), height=float(high[1]-low[1]))

    def normalize(self, image):
        """Bilinear board crop; output dimensions are a processing choice in pixels."""
        if image.ndim not in (2, 3) or tuple(image.shape[1::-1]) != tuple(self.image_size):
            raise ValueError('image dimensions do not match board geometry')
        width, height = self.normalized_size
        ys, xs = np.mgrid[:height, :width]
        source = self.to_image(np.column_stack((xs.ravel(), ys.ravel())))
        x = np.clip(source[:, 0], 0., self.image_size[0]-1)
        y = np.clip(source[:, 1], 0., self.image_size[1]-1)
        x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
        x1 = np.minimum(x0+1, self.image_size[0]-1)
        y1 = np.minimum(y0+1, self.image_size[1]-1)
        wx, wy = x-x0, y-y0
        if image.ndim == 3:
            wx, wy = wx[:, None], wy[:, None]
        sampled = ((1-wx)*(1-wy)*image[y0, x0] + wx*(1-wy)*image[y0, x1]
                   + (1-wx)*wy*image[y1, x0] + wx*wy*image[y1, x1])
        shape = (height, width) + image.shape[2:]
        if np.issubdtype(image.dtype, np.integer):
            sampled = np.rint(sampled)
        return sampled.reshape(shape).astype(image.dtype)

    def evidence(self):
        return dict(asdict(self), image_to_board=self._image_to_board.tolist(),
                    board_to_image=self._board_to_image.tolist())


@dataclass(frozen=True)
class MarkerEvidence:
    """Declared read-only verification record; contains no guessed marker defaults."""
    family: str
    marker_id: int
    side_length_m: float
    camera_info_id: str
    camera_info_size: tuple
    camera_info_frame_id: str
    camera_info_stamp_ns: int
    marker_stamp_ns: int
    board_relation_id: str
    verified: bool = False

    def rejection(self, stamp_ns, frame_id, image_size):
        if self.verified is not True:
            return 'unverified_marker_parameters'
        if any(not isinstance(value, str) or not value.strip() for value in
               (self.family, self.camera_info_id, self.board_relation_id, self.camera_info_frame_id)):
            return 'missing_marker_camera_or_board_reference'
        if (type(self.marker_id) is not int or self.marker_id < 0
                or not isfinite(self.side_length_m) or self.side_length_m <= 0):
            return 'invalid_marker_id_or_size'
        if (self.camera_info_frame_id != frame_id or tuple(self.camera_info_size) != tuple(image_size)
                or type(self.camera_info_stamp_ns) is not int or self.camera_info_stamp_ns != stamp_ns
                or type(self.marker_stamp_ns) is not int or self.marker_stamp_ns != stamp_ns):
            return 'marker_camera_image_mismatch'
        return None


@dataclass(frozen=True)
class DirectionEvidence:
    run_id: str
    geometry_id: str
    source: str
    reference: str
    zones: tuple
    confirmed: bool = False
    stamp_ns: int = None  # required for a per-frame field label or verified marker
    marker: MarkerEvidence = None
    observation_frame: str = ''  # optical frame of a marker image from this stopped board visit

    def __post_init__(self):
        object.__setattr__(self, 'zones', tuple(self.zones))

    def rejection(self, run_id, geometry_id, stamp_ns, *, frame_id=None, image_size=None):
        if self.confirmed is not True:
            return 'unconfirmed_direction'
        if self.run_id != run_id or self.geometry_id != geometry_id:
            return 'direction_identity_mismatch'
        if not isinstance(self.reference, str) or not self.reference.strip():
            return 'missing_direction_reference'
        if len(self.zones) != 3 or set(self.zones) != {'B', 'C', 'D'}:
            return 'invalid_direction_slots'
        if self.source == 'fixed_reference':
            return None
        if self.source == 'station_zone_markers':
            if (type(self.stamp_ns) is not int or not 0 < self.stamp_ns <= stamp_ns
                    or not self.observation_frame or self.observation_frame != frame_id):
                return 'station_marker_image_mismatch'
            return None
        if self.source not in ('field_label', 'verified_marker'):
            return 'unverified_direction_source'
        if type(self.stamp_ns) is not int or self.stamp_ns != stamp_ns:
            return 'direction_timestamp_mismatch'
        if self.source == 'verified_marker':
            if not isinstance(self.marker, MarkerEvidence):
                return 'missing_marker_verification'
            return self.marker.rejection(stamp_ns, frame_id, image_size)
        return None


@dataclass(frozen=True)
class BoardDecision:
    accepted: bool
    reason: str
    mapping: dict
    evidence: dict


class BoardReader:
    """Decode explicitly framed pixel detections; one instance per match."""
    def __init__(self, run_id, geometry, direction=None, *, min_votes=3,
                 threshold=.6, max_age_ns=2000000000, started_ns=0,
                 slot_edges=(0., 1/3, 2/3, 1.), max_frames=100):
        if not isinstance(run_id, str) or not run_id.strip() or not isinstance(geometry, BoardGeometry):
            raise ValueError('board reader needs a current run and BoardGeometry')
        if (type(min_votes) is not int or min_votes < 1 or type(max_frames) is not int
                or max_frames < min_votes):
            raise ValueError('board vote and frame counts must be positive and compatible')
        if type(max_age_ns) is not int or max_age_ns <= 0 or type(started_ns) is not int or started_ns < 0:
            raise ValueError('board age/start must be integer source-clock nanoseconds')
        if not isfinite(threshold) or not 0 < threshold <= 1:
            raise ValueError('invalid board template-correlation threshold')
        if (len(slot_edges) != 4 or not all(isfinite(value) for value in slot_edges)
                or slot_edges[0] != 0 or slot_edges[-1] != 1
                or any(a >= b for a, b in zip(slot_edges, slot_edges[1:]))):
            raise ValueError('board slot edges must partition normalized x from 0 to 1')
        self.run_id, self.geometry, self.direction = run_id, geometry, direction
        self.min_votes, self.threshold = min_votes, threshold
        self.max_age_ns, self.started_ns = max_age_ns, started_ns
        self.slot_edges, self.max_frames = tuple(slot_edges), max_frames
        self._frames = []
        self._accepted_frames = []
        self._events_total = 0
        self._last_stamp_ns = started_ns
        self._image_ids = set()
        self._frame_id = None
        self._source = None

    @classmethod
    def from_config(cls, run_id, config, *, started_ns):
        """Use verified FIXED geometry/direction; dynamic evidence uses the constructor.

        Existing board_axis/reverse/slots retain their previous meaning.  A four-
        corner board_corners overrides rectangular board_roi; its point order
        already defines the normalized board x/y axes.
        """
        axis, reverse = config.get('board_axis', 'x'), config.get('board_reverse', False)
        if axis not in ('x', 'y') or type(reverse) is not bool:
            raise ValueError('board axis/reverse configuration is invalid')
        corners = config.get('board_corners', [])
        if not corners:
            roi = config.get('board_roi', [])
            if (len(roi) != 4 or not all(isfinite(value) for value in roi)
                    or roi[0] >= roi[2] or roi[1] >= roi[3]):
                raise ValueError('board needs an original-image ROI or four corners')
            x0, y0, x1, y1 = roi
            corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1)) if axis == 'x' else (
                (x0, y0), (x0, y1), (x1, y1), (x1, y0))
        geometry = BoardGeometry(config.get('board_image_size', []), corners,
                                 config.get('board_normalized_size', [300, 100]),
                                 config.get('board_geometry_id', ''))
        zones = tuple(config.get('board_slots', ()))
        if reverse:
            zones = tuple(reversed(zones))
        direction = DirectionEvidence(run_id, geometry.geometry_id,
                                      config.get('board_direction_source', ''),
                                      config.get('board_direction_reference', ''), zones,
                                      config.get('board_slots_confirmed', False))
        if direction.source != 'fixed_reference' or direction.rejection(run_id, geometry.geometry_id, 0):
            raise ValueError('fixed board direction requires confirmed current-layout reference evidence')
        max_age = config.get('observation_max_age', 2.)
        if not isfinite(max_age) or max_age <= 0:
            raise ValueError('invalid board observation age')
        return cls(run_id, geometry, direction, min_votes=config.get('board_votes', 3),
                   threshold=config.get('board_min_confidence', .6), max_age_ns=int(max_age*1.e9),
                   started_ns=started_ns, slot_edges=config.get('board_slot_edges', (0., 1/3, 2/3, 1.)),
                   max_frames=config.get('board_frames', 8))

    def _record(self, reason, mapping, evidence):
        decision = BoardDecision(reason == 'accepted', reason, deepcopy(mapping), _evidence_values(evidence))
        self._events_total += 1
        self._frames.append(decision)
        self._frames = self._frames[-self.max_frames:]
        if decision.accepted:
            self._accepted_frames.append(decision)
            self._accepted_frames = self._accepted_frames[-self.max_frames:]
        return deepcopy(decision)

    def evidence(self):
        """Return a detached audit snapshot; dropped bounded history is explicit."""
        return {'run_id': self.run_id, 'started_ns': self.started_ns,
                'geometry': self.geometry.evidence(), 'min_votes': self.min_votes,
                'template_threshold': self.threshold, 'score_semantics': 'TM_CCOEFF_NORMED',
                'max_age_ns': self.max_age_ns, 'slot_edges': self.slot_edges,
                'events_total': self._events_total,
                'discarded_frame_records': max(0, self._events_total-len(self._frames)),
                'frames': [asdict(frame) for frame in self._frames]}

    def observe(self, detections, *, run_id, stamp_ns, now_ns, frame_id, image_id,
                source='formal_template', score_semantics='TM_CCOEFF_NORMED', direction=None,
                coordinate_space='original_image'):
        detections = tuple(detections)
        direction = direction or self.direction
        evidence = {'run_id': run_id, 'stamp_ns': stamp_ns, 'received_ros_ns': now_ns,
                    'frame_id': frame_id, 'image_id': image_id, 'source': source,
                    'score_semantics': score_semantics, 'geometry_id': self.geometry.geometry_id,
                    'coordinate_space': coordinate_space,
                    'direction': asdict(direction) if direction is not None else None,
                    'detections': [asdict(detection) for detection in detections]}
        if run_id != self.run_id:
            return self._record('frame_run_mismatch', {}, evidence)
        if coordinate_space not in ('original_image', 'normalized_board'):
            return self._record('invalid_coordinate_space', {}, evidence)
        if type(stamp_ns) is not int or type(now_ns) is not int or stamp_ns <= 0 or now_ns <= 0:
            return self._record('invalid_frame_timestamp', {}, evidence)
        if stamp_ns <= self.started_ns:
            return self._record('frame_precedes_run', {}, evidence)
        if not 0 <= now_ns-stamp_ns <= self.max_age_ns:
            return self._record('stale_or_future_frame', {}, evidence)
        if stamp_ns <= self._last_stamp_ns:
            return self._record('repeated_or_out_of_order_frame', {}, evidence)
        self._last_stamp_ns = stamp_ns
        if source not in ('formal_template', 'formal_hybrid'):
            return self._record('unverified_detector_source', {}, evidence)
        if score_semantics != 'TM_CCOEFF_NORMED':
            return self._record('board_score_semantics_mismatch', {}, evidence)
        if any(not isinstance(value, str) or not value.strip() for value in (frame_id, image_id)):
            return self._record('missing_image_provenance', {}, evidence)
        if image_id in self._image_ids:
            return self._record('repeated_image_identity', {}, evidence)
        if self._frame_id is not None and frame_id != self._frame_id:
            return self._record('image_frame_mismatch', {}, evidence)
        if self._source is not None and source != self._source:
            return self._record('detector_source_changed', {}, evidence)
        self._image_ids.add(image_id)
        self._frame_id, self._source = frame_id, source
        if direction is None:
            return self._record('missing_direction', {}, evidence)
        rejection = direction.rejection(self.run_id, self.geometry.geometry_id, stamp_ns,
                                        frame_id=frame_id, image_size=self.geometry.image_size)
        if rejection is not None:
            return self._record(rejection, {}, evidence)
        selected = []
        for detection, item in zip(detections, evidence['detections']):
            if detection.label not in OBJECTS:
                item['excluded'] = 'not_an_ordinary_board_class'
                continue
            if not detection.valid():
                return self._record('invalid_detection', {}, evidence)
            if detection.score < self.threshold:
                item['excluded'] = 'below_template_threshold'
                continue
            center = (np.asarray([detection.x, detection.y]) if coordinate_space == 'normalized_board'
                      else self.geometry.to_board([(detection.x, detection.y)])[0])
            u, v = center / self.geometry.normalized_size
            item['board_center'] = center.tolist()
            if not (0 <= u < 1 and 0 <= v < 1):
                item['excluded'] = 'outside_roi'
                continue
            if detection.width <= 0 or detection.height <= 0:
                return self._record('missing_detection_extent', {}, evidence)
            slot = int(np.searchsorted(self.slot_edges, u, side='right'))-1
            item['slot'] = slot
            left, right = detection.x-detection.width/2, detection.x+detection.width/2
            top, bottom = detection.y-detection.height/2, detection.y+detection.height/2
            box = np.asarray([(left, top), (right, top), (right, bottom), (left, bottom)])
            polygon = box if coordinate_space == 'normalized_board' else self.geometry.to_board(box)
            item['board_polygon'] = polygon.tolist()
            item['original_image_polygon'] = self.geometry.to_image(polygon).tolist()
            unit_polygon = polygon / self.geometry.normalized_size
            if np.any(unit_polygon < -1.e-9) or np.any(unit_polygon > 1.+1.e-9):
                return self._record('detection_crosses_roi', {}, evidence)
            if (np.min(unit_polygon[:, 0]) < self.slot_edges[slot]-1.e-9 or
                    np.max(unit_polygon[:, 0]) > self.slot_edges[slot+1]+1.e-9):
                return self._record('detection_crosses_slot', {}, evidence)
            selected.append((slot, detection.label))
        if len(selected) != 3:
            return self._record('missing_or_extra_slot', {}, evidence)
        if len({label for _, label in selected}) != 3:
            return self._record('duplicate_class', {}, evidence)
        if {slot for slot, _ in selected} != {0, 1, 2}:
            return self._record('duplicate_slot', {}, evidence)
        order = tuple(label for _, label in sorted(selected))
        evidence['order'] = order
        mapping = dict(zip(order, direction.zones))
        return self._record('accepted', mapping, evidence)

    def result(self, *, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            raise ValueError('current source-clock timestamp must be positive integer nanoseconds')
        fresh = [frame for frame in self._accepted_frames if
                 0 <= now_ns-frame.evidence['stamp_ns'] <= self.max_age_ns]
        votes = Counter(tuple(sorted(frame.mapping.items())) for frame in fresh)
        evidence = dict(self.evidence(), queried_ros_ns=now_ns,
                        support_stamps_ns=[frame.evidence['stamp_ns'] for frame in fresh],
                        support_frames=[asdict(frame) for frame in fresh],
                        votes=[{'mapping': dict(mapping), 'count': count} for mapping, count in votes.items()])
        if len(votes) > 1:
            return BoardDecision(False, 'conflicting_votes', {}, evidence)
        if not votes or max(votes.values()) < self.min_votes:
            return BoardDecision(False, 'insufficient_votes', {}, evidence)
        mapping, _ = votes.most_common(1)[0]
        return BoardDecision(True, 'accepted', dict(mapping), evidence)
