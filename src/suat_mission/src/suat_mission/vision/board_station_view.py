"""Board views and floor zone labels for one stopped classification visit."""
import importlib.util
import json
from pathlib import Path

from suat_mission.vision.board_recognition import BoardGeometry


def station_views(config, station):
    """Keep the reference separate. A derived pose needs its own live votes."""
    path = config['board_geometry_file']
    if station == 'Classification_area__recorded_reference':
        path = config.get('board_reference_geometry_file', path)
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    views = []

    def add(view):
        geometry = BoardGeometry(tuple(data['image_size']), view['corners'],
                                 tuple(view.get('normalized_size', data['normalized_size'])), view['geometry_id'])
        if not any(existing.corners == geometry.corners for existing in views):
            views.append(geometry)

    records_file = config.get('board_station_records_file')
    if station == 'Classification_area__latest_recording' and records_file:
        records = Path(records_file)
        view_file = (records.parent.parent / 'calibration' / 'board_view_candidates'
                     / (records.stem + '_latest_recording_view.json'))
        if view_file.is_file():
            view = json.loads(view_file.read_text(encoding='utf-8'))
            pose = next((entry['pose'] for entry in config.get('board_station_candidates', ())
                         if entry['id'] == 'latest_recording'), None)
            if pose is not None and tuple(pose) == tuple(view['recorded_pose']):
                if tuple(view['image_size']) != tuple(data['image_size']):
                    raise ValueError('recorded camera dimensions differ from the runtime calibration')
                add(view)
                return view_file, data, views

    candidate = config.get('board_view_candidate')
    if candidate:
        # The unsuffixed name uses the configured route pose. The latest
        # recording and its 5 cm backoff also evaluate the recorded geometry.
        stations = candidate.get('stations', (
            'Classification_area', 'Classification_area__active_route',
            'Classification_area__latest_recording', 'Classification_area__latest_backoff_5cm'))
        if station in stations:
            if tuple(candidate['image_size']) != tuple(data['image_size']):
                raise ValueError('board view dimensions differ from the station calibration')
            add(candidate)
    add(data)
    return path, data, views


def observe_zone_markers(root, image):
    """Call the teammate's image-only decoder without launching its arm/camera CLI."""
    folder = Path(root) / 'src/suat_mission/zone'
    spec = importlib.util.spec_from_file_location('suat_field_zone_vision', folder / 'zone_vision.py')
    vision = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vision)
    library = vision.load_marker_library(str(folder / 'zone_markers'))
    # A one- or two-template library cannot establish a unique B/C/D match.
    _, markers = vision.read_zone_markers(image, library if set(library) == {'B', 'C', 'D'} else {})
    return [dict(zone=m['zone'], source=m['source'], text=m['text'],
                 cx=float(m['cx']), quad=m['quad'].tolist())
            for m in sorted(markers, key=lambda m: m['cx'])]
