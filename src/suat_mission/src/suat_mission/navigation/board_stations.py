"""Map-bound recorded and nearby classification-board observation poses."""
import math

from suat_mission.navigation.map_identity import validate_map_identity


def _pose(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 3
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in values)):
        raise ValueError('board station requires a finite [x, y, yaw] map pose')
    return tuple(map(float, values))


def board_station_candidates(route_pose, binding, records):
    """Return view candidates in evidence order. Navigation and live reading still decide."""
    route = _pose(route_pose)
    if not isinstance(records, dict) or records.get('schema') != 2 or records.get('frame_id') != 'map':
        raise ValueError('board station records require schema 2 in the map frame')
    validate_map_identity(records, binding)
    record = records.get('candidate_metadata', {}).get('Classification_area')
    if not isinstance(record, dict) or record.get('frame_id') != 'map':
        raise ValueError('missing map-frame Classification_area station record')
    validate_map_identity(record, binding)
    latest = _pose(record.get('pose'))
    if latest != _pose(records.get('waypoints', {}).get('Classification_area')):
        raise ValueError('Classification_area waypoint and source record disagree')
    metadata = record.get('metadata', {})

    options = []
    seen = set()

    def add(name, pose, evidence):
        if pose not in seen:
            seen.add(pose)
            options.append({'id': name, 'pose': pose, 'view_evidence': evidence})

    previous = metadata.get('previous_station_pose')
    if previous is not None:
        add('recorded_reference', _pose(previous), 'recorded reference board view')
    x, y, yaw = latest
    for index, adjustment in enumerate(metadata.get('waypoint_adjustments', ()), start=1):
        pose = _pose(adjustment.get('new_pose'))
        if pose != latest:
            add('recorded_adjustment_' + str(index), pose, 'recorded observation adjustment; live read required')
    backoff = (x - .05 * math.cos(yaw), y - .05 * math.sin(yaw), yaw)
    add('latest_backoff_5cm', backoff, 'derived from recorded pose; live read required')
    add('latest_recording', latest, 'board visible in field notes; live read required')
    add('active_route', route, 'configured route view; live read required')
    return tuple(options)
