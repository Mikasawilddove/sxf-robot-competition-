"""Admission of task stations against one complete selected-map identity."""
from collections.abc import Mapping
from suat_mission.navigation.map_identity import map_identity, validate_map_identity
from suat_mission.automation.scope import scope_from_config


def _require_confirmed(record):
    if (('field_validated' in record and record['field_validated'] is not True)
            or (record.get('schema') == 2 and record.get('field_validated') is not True)
            or record.get('confirmation_level') == 'candidate'
            or 'CANDIDATE' in str(record.get('status', '')).upper()):
        raise ValueError('unvalidated candidate waypoints cannot be used as confirmed task stations')


def validate_waypoint_binding(config, binding, recorded=None):
    """Reject legacy/incompatible identities; never add identity to an old record."""
    _require_confirmed(config)
    identity = map_identity(config.get('waypoint_map_identity'))
    if binding is not None and identity != map_identity(binding):
        raise ValueError('waypoint_map_identity differs from the selected map binding')
    if recorded:
        if not isinstance(recorded, Mapping):
            raise ValueError('recorded waypoints must be an object')
        _require_confirmed(recorded)
        validate_map_identity(recorded, identity)
        points = recorded.get('waypoints', {})
        metadata = recorded.get('candidate_metadata', {})
        if not isinstance(points, Mapping) or not isinstance(metadata, Mapping):
            raise ValueError('invalid recorded waypoint document')
        for name in scope_from_config(config).required_points:
            if points.get(name) != config.get('waypoints', {}).get(name):
                raise ValueError('recorded waypoint differs from mission configuration: ' + name)
            if name in metadata:
                if not isinstance(metadata[name], Mapping):
                    raise ValueError('invalid station metadata: ' + name)
                _require_confirmed(metadata[name])
                validate_map_identity(metadata[name], identity)
    return identity
