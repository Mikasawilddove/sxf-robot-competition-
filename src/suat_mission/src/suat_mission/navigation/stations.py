"""Map-bound mission stations and independent candidate recordings."""
from datetime import datetime, timezone
from hashlib import sha256
import json
import math
from pathlib import Path

from suat_mission.navigation.map_identity import map_identity, validate_map_identity
from suat_mission.navigation.navigation_gate import goal_check_radius, goal_is_free
from suat_mission.navigation.pickup_keepout import is_pickup_station
from suat_mission.navigation.saved_grid import flat_map_metadata
from suat_mission.navigation.start_pose import read_start_pose, validate_start_pose


SORTING = ('Sorting_W', 'Sorting_S', 'Sorting_E', 'Sorting_N')
COLLECTION = ('Collection_A', 'Collection_B', 'Collection_C', 'Collection_D')
BOARD_STATION = 'Classification_area'
STATIONS = ('Start_A', BOARD_STATION, *SORTING, *COLLECTION)


def board_pairs(station=BOARD_STATION):
    return [('Start_A',station), *((station,goal) for goal in SORTING)]


def route_groups():
    return {
        'routes': board_pairs() + [(start,goal) for start in SORTING for goal in COLLECTION]
                  + [(start,goal) for start in COLLECTION for goal in SORTING],
        'fallback_routes': [(start,goal) for start in SORTING for goal in SORTING if start != goal],
        'direct_start_routes': [('Start_A',goal) for goal in SORTING],
    }


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def station_pose(pose):
    if (not isinstance(pose, (list, tuple)) or len(pose) != 3
            or not all(type(v) in (int, float) and math.isfinite(v) for v in pose)):
        raise ValueError('station requires finite [x,y,yaw]')
    return list(map(float, pose))


def map_image_digest(grid):
    selected = Path(grid['map_yaml'])
    image = selected.parent / flat_map_metadata(selected)['image']
    return sha256(image.read_bytes()).hexdigest()


def destination_radius(name, body_radius, keepout):
    return goal_check_radius(body_radius,is_pickup_station(keepout,name))


def body_radius(config):
    footprint = config['global_costmap']
    if footprint.get('footprint') not in (None, '', '[]', []):
        raise ValueError('offline station check requires the configured circular footprint')
    radius = float(footprint['robot_radius']) + float(footprint['footprint_padding'])
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('invalid configured body radius')
    return radius


def save_candidate(directory, name, pose, grid, radius, tf_stamp_ns):
    if name not in STATIONS or type(tf_stamp_ns) is not int or tf_stamp_ns <= 0:
        raise ValueError('unknown station or invalid TF timestamp')
    pose = station_pose(pose)
    clear = goal_is_free(grid, *pose[:2], radius)
    if not clear and name != 'Start_A':
        raise ValueError('station footprint blocked or in unknown space')
    record = {
        'schema': 2, 'frame_id': 'map', 'child_frame_id': 'base_footprint',
        'name': name, 'pose': pose, 'map_identity': map_identity(grid),
        'map_image_sha256': map_image_digest(grid),
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'source': 'live_tf', 'source_stamp_ns': tf_stamp_ns,
        'checked_radius_m': radius,
        'validation_status': 'STATIC_FOOTPRINT_CLEAR' if clear else 'START_CONTACT_REVIEW_REQUIRED',
    }
    path = Path(directory) / (name + '.json')
    data = json.dumps(record, ensure_ascii=False, indent=2) + '\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(data)
    return path


def load_candidate(path, name, grid, image_digest):
    record = read_json(path)
    validate_map_identity(record, grid)
    if (record.get('schema') != 2 or record.get('name') != name
            or record.get('frame_id') != 'map' or record.get('child_frame_id') != 'base_footprint'
            or record.get('source') != 'live_tf'
            or type(record.get('source_stamp_ns')) is not int or record['source_stamp_ns'] <= 0
            or record.get('map_image_sha256') != image_digest
            or record.get('validation_status') not in ('STATIC_FOOTPRINT_CLEAR', 'START_CONTACT_REVIEW_REQUIRED')):
        raise ValueError('invalid or mismatched station candidate: ' + str(path))
    return dict(record, pose=station_pose(record['pose']), file=str(path))


def mission_stations(root, config, binding, *, inventory=False):
    """Interpret the existing formats in the same order as formal navigation."""
    root = Path(root)
    points = {}

    def add(name, pose, source, status, recorded_at=None, *, override=False):
        pose = station_pose(pose)
        if name in points and not override:
            if points[name]['pose'] != pose:
                raise ValueError('conflicting mission station: ' + name)
            return
        points[name] = {'pose': pose, 'file': str(source), 'source': status,
                        'recorded_at': recorded_at, 'validation_status': 'FIELD_VALIDATION_NOT_INFERRED'}

    def read_resource(path, source, *, override=False, optional=False):
        path = root / path
        if (optional or inventory) and not path.exists():
            return
        data = read_json(path)
        if data.get('schema') == 2 and 'child_frame_id' in data:
            raise ValueError('individual TF candidate is not a formal station resource: '+str(path))
        validate_map_identity(data, binding)
        poses = data.get('waypoints', {})
        if override:
            valid = (data.get('frame_id') == 'map' and isinstance(poses,dict)
                     and (set(poses) <= set(SORTING) if inventory else set(poses) == set(SORTING)))
            if not valid:
                reason = ('invalid map-frame pickup approaches in ' if inventory
                          else 'four map-frame pickup approaches required in ')
                raise ValueError(reason+str(path))
        if isinstance(poses,list):
            names = [p['name'] for p in poses]
            if len(names) != len(set(names)):
                raise ValueError('duplicate station in '+str(path))
            poses = {p['name']:p['pose'] for p in poses}
        for name, pose in poses.items():
            add(name, pose, path, source, data.get('recorded_at') or data.get('recorded_on_robot_date')
                or data.get('client_session_date'),
                override=override)

    read_resource(config.get('recorded_route_file','data/navigation/recorded_stations_20261115.json'),
                  'recorded_route')
    for name, pose in config.get('waypoints', {}).items():
        add(name, pose, 'mission_config', 'inline')
    if config.get('additional_waypoints_file'):
        read_resource(config['additional_waypoints_file'], 'supplemental_record', optional=True)
    if config.get('pickup_stations_file'):
        if inventory:
            # The formal consumer requires this complete override. Leaving a
            # legacy pose in the inventory would hide a missing configured point.
            for name in SORTING:
                points.pop(name,None)
        read_resource(config['pickup_stations_file'], 'configured_pickup_approach', override=True)
    return points


def candidate_inventory(directory, stations, grid, image_digest):
    """Overlay a selected recording session for offline review only."""
    directory = Path(directory)
    if not directory.is_dir():
        raise FileNotFoundError('candidate session not found: '+str(directory))
    reviewed = dict(stations)
    for name in STATIONS:
        path = directory/(name+'.json')
        if path.is_file():
            record = load_candidate(path,name,grid,image_digest)
            reviewed[name] = dict(record, reviewed_as='OFFLINE_CANDIDATE', formal_resource=False)
    return reviewed


def load_start(root, grid):
    path = Path(root)/'data/waypoints/start_pose_A.yaml'
    record = read_start_pose(path)
    return {'pose': list(validate_start_pose(record,grid)), 'file': str(path), 'source': 'confirmed_start_record',
            'recorded_at': record.get('client_session_date'), 'validation_status': 'VERIFIED_AT_REST'}
