"""One source for planner, smoother and output limits. Candidates, not hardware ratings."""
import math
from pathlib import Path

PROFILES = {
    'pickup': {'linear': .025, 'angular': .20, 'accel_linear': .20, 'accel_angular': .60},
    'slow': {'linear': .08, 'angular': .30, 'accel_linear': .20, 'accel_angular': .60},
    'normal': {'linear': .12, 'angular': .45, 'accel_linear': .25, 'accel_angular': .80},
    'fast': {'linear': .20, 'angular': .60, 'accel_linear': .30, 'accel_angular': 1.0},
}

PLANNER_MODES = ('rollout', 'dwa')
CONTROLLER_HZ = 10.0
PLANNER_NAMESPACE = '/move_base/DWAPlannerROS'
LOCAL_PLANNER_PLUGIN = 'dwa_local_planner/DWAPlannerROS'
BOARD_POSITION = 'board_position'
BOARD_HEADING = 'board_heading'


def configured_global_planner(root, mode):
    """Read move_base's global plugin in navigation_field.launch load order."""
    import yaml
    directory = Path(root) / 'src/suat_mission/config'
    names = ['navigation_test.yaml']
    if mode == 'rollout':
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
    'forward_point_distance': .15, 'min_vel_theta': .03, 'min_vel_trans': .02,
    'sim_granularity': .01, 'angular_sim_granularity': .025,
    'vy_samples': 1, 'vth_samples': 25,
}


def local_planner_settings(mode, approach=False, xy_tolerance=.04, *, comparison=False,
                           track_front=False):
    if mode not in PLANNER_MODES:
        raise ValueError('local planner must be rollout or dwa')
    if mode == 'rollout' or comparison:
        # Noetic bounds forward rollout samples by goal distance / sim_time.
        # With a 4 cm goal tolerance, 3 s excludes the 0.02 m/s minimum
        # translation before the goal is accepted. The 1.5 s approach avoids it.
        settings = dict(PLANNER_COMMON, use_dwa=mode == 'dwa', sim_time=3., vx_samples=11)
        if approach:
            settings['sim_time'] = max(1/CONTROLLER_HZ,
                                       min(1.5, .8*xy_tolerance/PLANNER_COMMON['min_vel_trans']))
            # Score the base center while approaching a precise station pose.
            if not track_front:
                settings['forward_point_distance'] = 0.
        return settings
    return dict(PLANNER_COMMON, use_dwa=True, sim_time=1.2 if approach else 1.7, vx_samples=8)


def settings_match(actual, expected):
    for name, value in expected.items():
        received = actual.get(name)
        if isinstance(value, bool):
            if received is not value:
                return False
        elif type(received) not in (int, float) or not math.isclose(received,value,rel_tol=1e-8,abs_tol=1e-9):
            return False
    return True


def planner_profile_matches(actual, mode, *, comparison=False):
    """Recognize a cruise setup or the last stopped approach setup."""
    tolerance=actual.get('xy_goal_tolerance',.04)
    if type(tolerance) not in (int,float) or not math.isfinite(tolerance) or tolerance<=0:
        return False
    return (settings_match(actual,local_planner_settings(mode,comparison=comparison))
            or settings_match(actual,local_planner_settings(mode,True,tolerance,comparison=comparison))
            or settings_match(actual,local_planner_settings(mode,True,tolerance,comparison=comparison,
                                                            track_front=True)))


def speed_profile(name):
    if name not in PROFILES:
        raise ValueError('unknown navigation speed profile: '+str(name))
    return dict(PROFILES[name])


def planner_speed_limits(name, *, turn_only=False):
    profile = speed_profile(name)
    if turn_only:
        return {'max_vel_x': 0., 'max_vel_trans': 0.,
                'max_vel_theta': min(profile['angular'], .20)}
    return {'max_vel_x': profile['linear'], 'max_vel_trans': profile['linear'],
            'max_vel_theta': profile['angular']}


def station_approach_pose(path, distance=.35):
    """Pose along the approved path, facing the remaining segment to the station."""
    for a, b in reversed(list(zip(path, path[1:]))):
        length = math.dist(a, b)
        if length == 0:
            continue
        if length >= distance:
            fraction = distance/length
            return [b[0]+fraction*(a[0]-b[0]), b[1]+fraction*(a[1]-b[1]),
                    math.atan2(b[1]-a[1],b[0]-a[0])]
        distance -= length
    return None


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
