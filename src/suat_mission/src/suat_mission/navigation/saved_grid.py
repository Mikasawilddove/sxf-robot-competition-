"""Portable map_server PGM reader; no OpenCV, ROS or package installation needed."""
import ast
import json
import math
from pathlib import Path
import re
from suat_mission.navigation.map_identity import map_identity


def flat_map_metadata(path):
    """Read the flat scalar/inline-list subset emitted by map_server; reject nesting."""
    out = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        if line[0].isspace() or ':' not in line:
            raise ValueError('unsupported nested map metadata')
        key, value = line.split(':', 1)
        value = value.split(' #', 1)[0].strip()
        if key in out:
            raise ValueError('duplicate map metadata key')
        try:
            out[key] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            out[key] = value
    return out


def pgm_pixels(raw):
    pos = 0
    def token():
        nonlocal pos
        while pos < len(raw):
            if raw[pos] in b' \t\r\n':
                pos += 1
            elif raw[pos] == 35:
                end = raw.find(b'\n', pos)
                pos = len(raw) if end < 0 else end+1
            else:
                break
        match = re.match(rb'[^\s#]+', raw[pos:])
        if match is None:
            raise ValueError('truncated PGM header')
        pos += len(match[0])
        return match[0]
    magic, width, height, maximum = token(), int(token()), int(token()), int(token())
    if magic not in (b'P5', b'P2') or maximum != 255 or not (0 < width and 0 < height and width*height <= 4000000):
        raise ValueError('only 8-bit PGM maps up to 4M cells are supported')
    if magic == b'P5':
        if pos >= len(raw) or raw[pos] not in b' \t\r\n':
            raise ValueError('missing PGM raster separator')
        pos += 2 if raw[pos:pos+2] == b'\r\n' else 1
        pixels = list(raw[pos:])
        if len(pixels) != width*height:
            raise ValueError('PGM raster size mismatch')
    else:
        pixels = [int(token()) for _ in range(width*height)]
        if any(not 0 <= v <= 255 for v in pixels):
            raise ValueError('invalid PGM pixel')
    return width, height, pixels


def load_saved_grid(root):
    root = Path(root).resolve()
    selected = flat_map_metadata(root/'src/suat_mission/config/selected_map.yaml')
    path = (root/selected['map_yaml']).resolve()
    path.relative_to(root)
    config = flat_map_metadata(path)
    image = (path.parent/config['image']).resolve()
    image.relative_to(root)
    raw = image.read_bytes()
    # Historical labels are retained for file compatibility, not computed/compared.
    identity = selected.get('image_sha256')
    ox, oy, yaw = map(float, config['origin'])
    resolution = float(config['resolution'])
    if not all(math.isfinite(v) for v in (ox, oy, yaw, resolution)) or yaw != 0 or resolution <= 0:
        raise ValueError('invalid or rotated map geometry')
    if config.get('mode', 'trinary') not in ('trinary', 0, None):
        raise ValueError('only trinary maps are supported')
    free, occupied = float(config['free_thresh']), float(config['occupied_thresh'])
    if not 0 <= free < occupied <= 1 or config.get('negate', 0) not in (0, 1):
        raise ValueError('invalid map thresholds')
    width, height, pixels = pgm_pixels(raw)
    data = []
    for row in range(height-1, -1, -1):
        for pixel in pixels[row*width:(row+1)*width]:
            probability = pixel/255. if config.get('negate', 0) else 1-pixel/255.
            data.append(100 if probability > occupied else 0 if probability < free else -1)
    binding = {'map_yaml': str(path), 'image_sha256': identity, 'resolution': resolution,
            'width': width, 'height': height, 'origin': [ox, oy], 'data': data,
            'origin_yaw': yaw, 'negate': config.get('negate', 0), 'mode': 'trinary',
            'free_thresh': free, 'occupied_thresh': occupied,
            'frame_id': selected.get('frame_id'), 'layout_version': selected.get('layout_version'),
            'alignment_version': selected.get('map_record_alignment_version', selected.get('alignment_version')),
            'grid_sha256':selected.get('grid_sha256'), 'identity_method':'selected_path_and_geometry'}
    map_identity(binding)
    if binding['frame_id'] != 'map':
        raise ValueError('navigation currently supports only the map frame')
    field_file = selected.get('field_alignment_file')
    if field_file:
        from suat_mission.navigation.field_alignment import validate_alignment
        field_path = (root/field_file).resolve()
        field_path.relative_to(root)
        version = selected['alignment_version']
        if version == 'UNALIGNED':
            raise ValueError('selected field transform requires a named alignment version')
        record = json.loads(field_path.read_text(encoding='utf-8'))
        binding.update(field_alignment=validate_alignment(record, binding, version),
                       field_alignment_file=str(field_path))
    elif 'map_record_alignment_version' in selected:
        raise ValueError('selected field alignment file is missing')
    return binding
