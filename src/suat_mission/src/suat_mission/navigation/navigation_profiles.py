"""One source for planner, smoother and output limits. Candidates, not hardware ratings."""
import math

PROFILES = {
    'slow': {'linear': .08, 'angular': .30, 'accel_linear': .20, 'accel_angular': .60},
    'normal': {'linear': .12, 'angular': .45, 'accel_linear': .25, 'accel_angular': .80},
    'fast': {'linear': .20, 'angular': .60, 'accel_linear': .30, 'accel_angular': 1.0},
}


def speed_profile(name):
    if name not in PROFILES:
        raise ValueError('speed must be slow, normal or fast')
    return dict(PROFILES[name])


def planner_speed_limits(name):
    profile = speed_profile(name)
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
