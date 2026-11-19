"""A separately recorded competition start pose, tied to the exact map."""
import math
import json
from pathlib import Path
from suat_mission.navigation.map_identity import map_identity, validate_map_identity


def read_start_pose(path):
    """JSON works offline; existing YAML resources retain the robot's PyYAML dependency."""
    text=Path(path).read_text(encoding='utf-8-sig')
    try:
        return json.loads(text)
    except ValueError:
        try:
            import yaml
        except ImportError as exc:
            raise ValueError('PyYAML is required to read this existing YAML Start_A record; no dependency is installed automatically') from exc
        try:
            return yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise ValueError('invalid Start_A YAML: '+str(exc)) from exc


def validate_start_pose(data, binding):
    if not isinstance(data, dict) or data.get('schema') != 1 or data.get('name') != 'Start_A':
        raise ValueError('not a Start_A record')
    if data.get('frame_id') != 'map' or data.get('verified_at_rest') is not True:
        raise ValueError('A start has not been confirmed at rest')
    if data.get('site_confirmation') != 'operator_confirmed_A_facing_D_and_scan_alignment':
        raise ValueError('missing field identity and direction confirmation')
    validate_map_identity(data, binding)
    pose = data.get('pose')
    if (not isinstance(pose, (list, tuple)) or len(pose) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose)):
        raise ValueError('invalid A pose')
    cov = data.get('initial_covariance_diagonal')
    if (not isinstance(cov, (list, tuple)) or len(cov) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in cov)):
        raise ValueError('missing positive initialization covariance')
    return tuple(float(v) for v in pose)


def make_start_pose(pose, binding, agreement, session_date, site_confirmed=False):
    if not site_confirmed:
        raise ValueError('operator must confirm A placement, direction and scan alignment')
    if not isinstance(agreement, (int, float)) or not math.isfinite(agreement) or not .8 <= agreement <= 1.:
        raise ValueError('scan geometry is not ready for recording')
    data = {'schema': 1, 'name': 'Start_A', 'frame_id': 'map', 'pose': list(pose),
            'map_identity': map_identity(binding),
            'map_image_sha256': binding['image_sha256'], 'map_grid_sha256': binding['grid_sha256'],
            'heading_reference': 'A_to_D', 'verified_at_rest': True,
            'site_confirmation': 'operator_confirmed_A_facing_D_and_scan_alignment',
            'initial_covariance_diagonal': [.1**2, .1**2, math.radians(5)**2],
            'covariance_note': 'initialization prior; not independently measured localization accuracy',
            'scan_endpoint_fraction_within_0_10m': float(agreement), 'client_session_date': session_date,
            'moving_localization_validation': 'NOT_PASSED',
            'navigation_validation': 'NOT_RUN'}
    validate_start_pose(data, binding)
    return data
