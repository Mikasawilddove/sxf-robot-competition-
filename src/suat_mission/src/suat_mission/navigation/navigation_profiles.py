"""One source for planner, smoother and output limits. Candidates, not hardware ratings."""
import math
from pathlib import Path

PROFILES = {
    'slow': {'linear': .08, 'angular': .30, 'accel_linear': .20, 'accel_angular': .60},
    'normal': {'linear': .12, 'angular': .45, 'accel_linear': .25, 'accel_angular': .80},
    'fast': {'linear': .20, 'angular': .60, 'accel_linear': .30, 'accel_angular': 1.0},
}

PLANNER_MODES = ('rollout', 'dwa')
CONTROLLER_HZ = 10.0
PLANNER_NAMESPACE = '/move_base/DWAPlannerROS'
LOCAL_PLANNER_PLUGIN = 'dwa_local_planner/DWAPlannerROS'


def configured_global_planner(root, mode):
    """Read move_base's global plugin in navigation_field.launch load order."""
    import yaml
    directory = Path(root) / 'src/suat_mission/config'
    names = ['navigation_test.yaml']
    if mode in ('rollout','path'):
        names.append('navigation_rollout_candidate.yaml')
    plugin = None
    for name in names:
        with (directory / name).open(encoding='utf-8') as stream:
            plugin = yaml.safe_load(stream).get('base_global_planner', plugin)
    if not isinstance(plugin, str) or '/' not in plugin:
        raise ValueError('invalid base_global_planner in ' + ', '.join(names))
    return plugin

# Both modes use the same critics and footprint. Sampling differs because
# the dynamic window covers one control period, while rollout covers a horizon.
PLANNER_COMMON = {
    'path_distance_bias': 32.0, 'goal_distance_bias': 24.0, 'occdist_scale': .01,
    'forward_point_distance': .15, 'min_vel_theta': .03, 'min_vel_trans': .005,
    'sim_granularity': .01, 'angular_sim_granularity': .025,
    'vy_samples': 1, 'vth_samples': 25,
}


def local_planner_settings(mode):
    if mode == 'path':
        return {}
    if mode not in PLANNER_MODES:
        raise ValueError('local planner must be rollout or dwa')
    # Rollout caps the longitudinal sample at distance divided by sim_time.
    # This horizon and minimum leave room to converge on a 1 cm station.
    return dict(PLANNER_COMMON, use_dwa=mode == 'dwa', sim_time=1.5, vx_samples=11)


def settings_match(actual, expected):
    for name, value in expected.items():
        received = actual.get(name)
        if isinstance(value, bool):
            if received is not value:
                return False
        elif type(received) not in (int, float) or not math.isclose(received,value,rel_tol=1e-8,abs_tol=1e-9):
            return False
    return True


def planner_profile_matches(actual, mode):
    return settings_match(actual, local_planner_settings(mode))


def speed_profile(name):
    if name not in PROFILES:
        raise ValueError('unknown navigation speed profile: '+str(name))
    return dict(PROFILES[name])


def planner_speed_limits(name):
    profile = speed_profile(name)
    return {'max_vel_x': profile['linear'], 'max_vel_trans': profile['linear'],
            'max_vel_theta': profile['angular']}


def leg_timeout(path_length, speed):
    if not math.isfinite(path_length) or not 0 <= path_length <= 8:
        raise ValueError('leg path must be finite and at most 8 m')
    p = speed_profile(speed)
    return min(180., max(35., path_length/p['linear']*2 + math.pi/p['angular'] + 15))


def path_travel_fault(previous, current, distance, limit=8.):
    if not all(math.isfinite(v) for v in (*previous, *current, distance, limit)) or distance < 0:
        return 'invalid_field_odometry', distance
    step = math.hypot(current[0]-previous[0], current[1]-previous[1])
    distance += step
    if step > .25:
        return 'odometry_position_jump', distance
    return ('field_leg_travel_bound' if distance > limit else None), distance
