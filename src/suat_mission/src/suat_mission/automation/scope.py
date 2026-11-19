"""One resource selection for diagnostics, construction and execution."""
from dataclasses import dataclass

from suat_mission.vision.perception import OBJECTS


@dataclass(frozen=True)
class MissionScope:
    name: str
    sorting_points: tuple
    labels: tuple
    delivery_zones: tuple

    @property
    def required_points(self):
        return ('Classification_area',) + self.sorting_points + tuple(
            'Collection_' + zone for zone in self.delivery_zones)

    @property
    def regular_labels(self):
        return tuple(label for label in self.labels if label != 'cake')


def select_scope(name='full', sorting_points=('Sorting_N', 'Sorting_W')):
    if name not in ('cake-only', 'full'):
        raise ValueError('mission_scope must be cake-only or full')
    if (not isinstance(sorting_points, (list, tuple)) or not sorting_points
            or any(not isinstance(point, str) or not point.strip() for point in sorting_points)
            or len(set(sorting_points)) != len(sorting_points)):
        raise ValueError('sorting_points must contain unique nonempty waypoint names')
    return MissionScope(name, tuple(sorting_points), ('cake',) if name == 'cake-only' else ('cake',) + OBJECTS,
                        ('A',) if name == 'cake-only' else tuple('ABCD'))


def scope_from_config(config):
    return select_scope(config.get('mission_scope', 'full'),
                        config.get('sorting_points', ('Sorting_N', 'Sorting_W')))
