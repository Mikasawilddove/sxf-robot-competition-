"""Read-only configuration diagnostics; passing this does not certify a robot."""
import math
import re
from pathlib import Path
from collections.abc import Mapping
from suat_mission.manipulation.calibration import PlanarCalibration
from suat_mission.vision.board_recognition import BoardReader
from suat_mission.automation.resource_binding import validate_waypoint_binding
from suat_mission.automation.scope import scope_from_config, select_scope
from suat_mission.navigation.navigation_profiles import speed_profile

REQUIRED_POINTS = select_scope().required_points  # compatibility for callers of the original diagnostic
GATES = ('hardware_ready', 'board_slots_confirmed', 'calibration_verified', 'identity_source_verified',
         'feedback_source_verified', 'placement_source_verified', 'control_chain_verified')


def validate_board_configuration(config):
    """Use the runtime geometry/direction constructor and explicit transport declarations."""
    BoardReader.from_config('configuration_check', config, started_ns=0)
    if config.get('board_detector_source') not in ('formal_template', 'formal_hybrid'):
        raise ValueError('board_detector_source must identify the verified formal detector')
    if config.get('board_score_semantics') != 'TM_CCOEFF_NORMED':
        raise ValueError('board_score_semantics must be TM_CCOEFF_NORMED for ordinary board classes')
    for name in ('board_detector_publisher', 'board_image_topic'):
        value = config.get(name)
        if not isinstance(value, str) or not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*', value):
            raise ValueError(name + ' requires the verified absolute ROS source name')


def finite_vector(value, size):
    try:
        return (isinstance(value, (list, tuple)) and len(value) == size
                and all(type(v) in (int, float) and math.isfinite(v) for v in value))
    except (ValueError, TypeError):
        return False


def inspect_config(config, recorded=None, *, binding=None):
    if isinstance(config,Mapping) and config.get('execution_backend')=='serial_rgbd':
        return inspect_serial_config(config,binding)
    issues = []
    def require(condition, code, detail):
        if not condition:
            issues.append({'code': code, 'detail': detail})
    def mapping(name):
        value = config.get(name, {})
        require(isinstance(value, Mapping), 'type.' + name, 'must be a mapping')
        return value if isinstance(value, Mapping) else {}
    if not isinstance(config, Mapping):
        return {'mode': 'read_only', 'configuration_complete': False, 'motion_authorized': False,
                'issues': [{'code': 'type.config', 'detail': 'must be a mapping'}]}
    try:
        scope = scope_from_config(config)
    except (TypeError, ValueError) as exc:
        return {'mode': 'read_only', 'configuration_complete': False, 'motion_authorized': False,
                'scope': config.get('mission_scope'), 'issues': [{'code': 'scope', 'detail': str(exc)}],
                'recorded_points': [], 'recorded_missing': []}
    for name in GATES:
        require(config.get(name) is True, 'gate.' + name, 'not independently verified')
    try:
        speed = config.get('navigation_speed', 'slow')
        speed_profile(speed)
        if 'speed_profile' in config and config['speed_profile'] != speed:
            raise ValueError('legacy speed_profile conflicts with navigation_speed')
    except (TypeError, ValueError) as exc:
        require(False, 'navigation.speed', str(exc))
    points = mapping('waypoints')
    calibrations, heights, drops = mapping('calibration_files'), mapping('grasp_z'), mapping('drop_poses')
    slots = config.get('board_slots')
    require(isinstance(slots, (list, tuple)) and len(slots) == 3
            and all(isinstance(slot, str) for slot in slots) and set(slots) == {'B', 'C', 'D'},
            'board.slots', 'three camera-order slots B/C/D are required even for cake-only')
    require(config.get('board_axis', 'x') in ('x', 'y'), 'board.axis', 'must be x or y')
    try:
        validate_board_configuration(config)
    except (TypeError, ValueError) as exc:
        require(False, 'board.configuration', str(exc))
    for name in scope.required_points:
        require(finite_vector(points.get(name), 3), 'waypoint.' + name, 'missing/invalid in mission configuration')
    try:
        validate_waypoint_binding(config, binding, recorded)
    except (TypeError, ValueError) as exc:
        require(False, 'waypoint.map_binding', str(exc))
    bounds, z_bounds = config.get('arm_bounds'), config.get('arm_z_bounds')
    valid_bounds = (finite_vector(bounds, 4) and bounds[0] < bounds[1] and bounds[2] < bounds[3])
    valid_z = finite_vector(z_bounds, 2) and z_bounds[0] < z_bounds[1]
    require(valid_bounds and valid_z, 'geometry.bounds', 'verified XY/Z limits required')
    for name in ('approach_offset', 'lift_offset', 'mission_timeout', 'action_timeout', 'navigation_timeout',
                 'feedback_timeout', 'observation_timeout', 'observation_max_age'):
        value = config.get(name)
        require(type(value) in (int, float) and math.isfinite(value) and value > 0,
                'number.' + name, 'must be finite and positive')
    budget = []
    require(type(config.get('judge_clock_already_running')) is bool, 'budget.judge_clock',
            'explicit boolean required: judge clock running at preparation, or starts at Enter')
    for name in ('startup_delay', 'stop_margin'):
        value = config.get(name)
        valid = type(value) in (int, float) and math.isfinite(value) and value >= 0
        require(valid, 'number.' + name, 'explicit finite nonnegative seconds required; unknown is not zero')
        if valid:
            budget.append(value)
    timeout = config.get('mission_timeout')
    if len(budget) == 2 and type(timeout) in (int, float) and math.isfinite(timeout):
        require(sum(budget) < timeout, 'number.usable_budget', 'startup delay and stop margin consume the full budget')
    for label in scope.labels:
        calibration = calibrations.get(label)
        require(bool(calibration), 'calibration.' + label,
                'verified file and height-specific validation required')
        if calibration:
            try:
                PlanarCalibration.from_values(Path(calibration).expanduser().read_text(encoding='utf-8').split())
            except (OSError, TypeError, ValueError) as exc:
                require(False, 'calibration_file.' + label, str(exc))
        height = heights.get(label)
        require(type(height) in (int, float) and math.isfinite(height), 'grasp_z.' + label, 'height required in mm')
    require(bool(config.get('calibration_id')), 'calibration.version', 'version required in observation contract')
    for zone in scope.delivery_zones:
        pose = drops.get(zone)
        require(finite_vector(pose, 3), 'drop_pose.' + zone, 'verified drop pose required in mm')
        if finite_vector(pose, 3) and valid_bounds and valid_z:
            require(bounds[0] <= pose[0] <= bounds[1] and bounds[2] <= pose[1] <= bounds[3]
                    and z_bounds[0] <= pose[2] <= z_bounds[1], 'drop_bounds.' + zone, 'outside configured bounds')
    known = recorded.get('waypoints', {}) if isinstance(recorded, Mapping) else {}
    known = known if isinstance(known, Mapping) else {}
    return {'mode': 'read_only', 'configuration_complete': not issues,
            'scope': scope.name,
            'selected_map_checked': binding is not None,
            'motion_authorized': False, 'issues': issues,
            'recorded_points': sorted(known),
            'recorded_missing': [p for p in scope.required_points if not finite_vector(known.get(p), 3)],
            'limits': 'No motion, TF, footprint, physical calibration validity or live feedback was tested.'}


def inspect_serial_config(config, binding):
    """Required production resources, without legacy manual verification switches."""
    from suat_mission.manipulation.pickup_geometry import PickupGeometry
    issues=[]
    try:
        scope=scope_from_config(config)
        geometry=PickupGeometry.load(config['pickup_geometry_file'])
        if geometry.values.get('grasp_xy_source')=='official-planar':
            models=geometry.values.get('official_planar',{}).get('planes',[])
            for label in scope.labels:
                if not any(label in p['labels'] and p.get('top_z_mm') is not None
                           and p.get('image_size') is not None for p in models):
                    issues.append({'code':'official_calibration.'+label,
                                   'detail':'factory model missing a known plane height or calibrated image size'})
        for point in scope.required_points:
            if not finite_vector(config.get('waypoints',{}).get(point),3):
                issues.append({'code':'waypoint.'+point,'detail':'recorded task station missing'})
        if int(config.get('max_base_alignments',2)) > 0:
            geometry.number('arm_yaw_in_base_rad')
        for key in ('observation_pose_mm','board_observation_pose_mm','transport_pose_mm','empty_travel_pose_mm'):
            geometry.check_pose(geometry.vector(key,3))
        for zone in scope.delivery_zones:
            if geometry.values.get('placement_mode')=='visual_red_zone':continue
            slots=geometry.values.get('drop_slots_arm_mm',{}).get(zone,[])
            needed=1 if zone=='A' else int(config.get('each_count',3))
            if len(slots)<needed:
                issues.append({'code':'placement.'+zone,'detail':'not enough recorded placement slots'})
            for index in range(len(slots)):
                geometry.release(zone,'cake' if zone=='A' else 'clock',index)
            region=geometry.values.get('drop_regions_arm_mm',{}).get(zone)
            if not finite_vector(region,4):
                issues.append({'code':'placement_region.'+zone,'detail':'measured landing region missing'})
        board=json_board_geometry(config['board_geometry_file'])
        if board.get('direction_confirmed') is not True:
            issues.append({'code':'board.direction','detail':'physical B/C/D order not recorded'})
        for name in ('startup_delay','stop_margin'):
            value=config.get(name)
            if type(value) not in (int,float) or not math.isfinite(value) or value<0:
                issues.append({'code':'budget.'+name,'detail':'explicit nonnegative seconds required'})
        if type(config.get('judge_clock_already_running')) is not bool:
            issues.append({'code':'budget.judge_clock','detail':'timer relation to Enter must be specified'})
        if binding is not None:
            from suat_mission.navigation.map_identity import map_identity
            if map_identity(config.get('waypoint_map_identity'))!=map_identity(binding):
                issues.append({'code':'waypoint.map','detail':'recorded stations refer to another map/layout'})
    except (OSError,KeyError,TypeError,ValueError) as exc:
        issues.append({'code':'serial_resources','detail':str(exc)})
    return {'mode':'production_resources','configuration_complete':not issues,
            'motion_authorized':False,'issues':issues,'scope':config.get('mission_scope'),
            'limits':'Live serial, camera and navigation are prepared by their actual adapters.'}


def json_board_geometry(path):
    import json
    return json.loads(Path(path).read_text(encoding='utf-8'))
