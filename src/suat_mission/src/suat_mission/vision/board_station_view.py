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


def visible_board_view(image, reference):
    """Recover the bright printed board face near a recorded station view."""
    import cv2
    import numpy as np
    corners = np.asarray(reference.corners, dtype=np.float32)
    span = np.ptp(corners, axis=0)
    pad = int(span[1] * .6)
    width, height = reference.image_size
    x0, y0 = np.maximum(np.floor(corners.min(axis=0)).astype(int) - pad, 0)
    x1, y1 = np.minimum(np.ceil(corners.max(axis=0)).astype(int) + pad, (width, height))
    gray = cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    smooth = cv2.GaussianBlur(gray, (3, 3), 0)
    background, paper = np.percentile(smooth, (65, 85))
    if paper - background < 8.:
        return None
    reference_area = cv2.contourArea(corners)
    kernel = np.ones((5, 5), np.uint8)
    # A brighter cutoff can separate the paper from a light floor patch touching it.
    for fraction in (.5, .65, .8):
        mask = cv2.threshold(smooth, background + (paper - background) * fraction, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            hull = cv2.convexHull(contour)
            area = cv2.contourArea(hull)
            if not .55 * reference_area <= area <= 1.25 * reference_area:
                continue
            if cv2.contourArea(contour) < .90 * area:
                continue
            quad = cv2.approxPolyDP(hull, .018 * cv2.arcLength(hull, True), True).reshape(-1, 2)
            if len(quad) != 4:
                continue
            left, right = np.argsort(quad[:, 0]).reshape(2, 2)
            tl, bl = left[np.argsort(quad[left, 1])]
            tr, br = right[np.argsort(quad[right, 1])]
            quad = quad[[tl, tr, br, bl]].astype(float) + (x0, y0)
            if np.any(np.linalg.norm(quad - corners, axis=1) > pad):
                continue
            if np.any(quad <= (x0, y0)) or np.any(quad >= (x1 - 1, y1 - 1)):
                continue
            center = quad.mean(axis=0)
            quad = np.clip(center + (quad - center) * (1.035, 1.09), (0, 0), (width - 1, height - 1))
            return BoardGeometry(reference.image_size, quad, reference.normalized_size,
                                 reference.geometry_id + '_visible_outline')
    return None


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
