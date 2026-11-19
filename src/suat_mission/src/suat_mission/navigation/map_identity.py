"""Complete, path-independent map identity shared by saved resources."""
import math


def map_identity(binding):
    """Return a detached identity; an incomplete/legacy binding is not accepted."""
    if not isinstance(binding, dict):
        raise ValueError('map binding must be an object')
    keys = ('resolution', 'width', 'height',
            'origin', 'origin_yaw', 'negate', 'free_thresh', 'occupied_thresh',
            'mode', 'frame_id', 'layout_version', 'alignment_version')
    if any(key not in binding for key in keys):
        raise ValueError('incomplete map identity; geometry and versions are required')
    result = {key: binding[key] for key in keys}
    for key in ('mode', 'frame_id', 'layout_version', 'alignment_version'):
        if not isinstance(result[key], str) or not result[key].strip():
            raise ValueError('invalid map identity '+key)
    for key in ('width', 'height'):
        if type(result[key]) is not int or result[key] <= 0:
            raise ValueError('invalid map identity '+key)
    origin = result['origin']
    if not isinstance(origin, (list, tuple)) or len(origin) != 2:
        raise ValueError('map origin requires [x,y] and a separate origin_yaw')
    for value in (*origin, result['origin_yaw'], result['resolution'], result['free_thresh'], result['occupied_thresh']):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('non-finite map identity geometry')
    if result['resolution'] <= 0 or not 0 <= result['free_thresh'] < result['occupied_thresh'] <= 1:
        raise ValueError('invalid map resolution or thresholds')
    if type(result['negate']) is not int or result['negate'] not in (0, 1) or result['mode'] != 'trinary':
        raise ValueError('unsupported map occupancy interpretation')
    result['origin'] = [float(v) for v in origin]
    return result


def validate_map_identity(record, binding):
    """Fail closed for historical records that only stored image/grid hashes."""
    identity = map_identity(binding)
    saved = record.get('map_identity') if isinstance(record, dict) else None
    if not isinstance(saved, dict) or map_identity(saved) != identity:
        raise ValueError('resource has missing or different complete map identity; recheck the original record')
    return identity
