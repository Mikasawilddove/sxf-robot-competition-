"""Final base-output lease for the isolated low-speed field test (not a safety PLC)."""
import math
import uuid


def navigation_clients_fault(publishers, client='/suat_point_navigation_test'):
    # move_base itself publishes action goals to bridge its simple-goal input.
    # Source: ros-planning/navigation noetic-devel move_base/src/move_base.cpp.
    allowed = {'/move_base', client}
    if set(publishers.get('/move_base/goal', [])) - allowed:
        return 'other_navigation_clients'
    if publishers.get('/move_base_simple/goal', []):
        return 'uncontrolled_simple_goal_publisher'
    return None


def motion_bounds_fault(start, current):
    if len(start) != 3 or len(current) != 3 or not all(math.isfinite(v) for v in tuple(start)+tuple(current)):
        return 'invalid_test_odometry_pose'
    if math.hypot(current[0]-start[0], current[1]-start[1]) > .60:
        return 'first_test_travel_bound'
    delta = math.atan2(math.sin(current[2]-start[2]), math.cos(current[2]-start[2]))
    if abs(delta) > .35:
        return 'first_test_yaw_bound'
    return None


class NavigationGate:
    MAX_V = .08
    MAX_W = .30
    HEARTBEAT_TIMEOUT = .6
    RAW_TIMEOUT = .4
    SMOOTH_TIMEOUT = .25
    STARTUP_GRACE = 2.0
    MAX_RUN = 60.0

    def __init__(self, max_v=.08, max_w=.30, max_run=60.):
        if not all(math.isfinite(v) for v in (max_v,max_w,max_run)) or not (0 < max_v <= .20 and 0 < max_w <= .60 and 0 < max_run <= 180):
            raise ValueError('invalid candidate navigation limits')
        self.MAX_V,self.MAX_W,self.MAX_RUN=max_v,max_w,max_run
        self.lease_v,self.lease_w=max_v,max_w
        self.state = 'DISARMED'
        self.reason = 'explicit_arm_required'
        self.token = None
        self.sequence = -1
        self.started = self.heartbeat_at = None
        self.raw = self.smooth = None

    def arm(self, now, fault=None, caps=None):
        if self.state != 'DISARMED' or fault:
            raise ValueError(fault or 'stop/release required before a new run')
        v,w=caps if caps is not None else (self.MAX_V,self.MAX_W)
        if not (0<v<=self.MAX_V and 0<w<=self.MAX_W):
            raise ValueError('navigation lease limits exceed configured speed')
        self.lease_v,self.lease_w=v,w
        self.state, self.reason = 'ARMED', 'waiting_for_fresh_navigation_input'
        self.token, self.sequence = uuid.uuid4().hex, -1
        self.started = self.heartbeat_at = now
        self.raw = self.smooth = None
        return self.token

    def heartbeat(self, token, sequence, now, latched=False):
        if (latched or self.state != 'ARMED' or token != self.token
                or type(sequence) is not int or sequence <= self.sequence):
            return False
        if now - self.heartbeat_at >= self.HEARTBEAT_TIMEOUT or now - self.started >= self.MAX_RUN:
            self.stop('expired_lease')
            return False
        self.sequence, self.heartbeat_at = sequence, now
        return True

    def command(self, source, values, now, latched=False):
        if self.state != 'ARMED':
            return
        if (latched or len(values) != 6 or not all(math.isfinite(v) for v in values)
                or any(abs(values[i]) > 1e-8 for i in (1, 2, 3, 4))):
            self.stop('invalid_or_latched_twist')
            return
        v, w = values[0], values[5]
        if source == 'raw':
            if v < -1e-6 or v > self.lease_v + 1e-3 or abs(w) > self.lease_w + 1e-3:
                self.stop('planner_limit_mismatch')
                return
            self.raw = (now, v, w)
        elif source == 'smooth':
            self.smooth = (now, v, w)
        else:
            self.stop('unexpected_input_source')

    def stop(self, reason='explicit_stop'):
        if self.state in ('DISARMED', 'ARMED'):
            self.state, self.reason, self.token = 'LATCHED', reason, None
            self.raw = self.smooth = None

    def output(self, now, fault=None):
        if self.state != 'ARMED':
            return 0.0, 0.0
        if fault or now - self.heartbeat_at >= self.HEARTBEAT_TIMEOUT or now - self.started >= self.MAX_RUN:
            self.stop(fault or 'expired_lease')
        elif self.raw is None or self.smooth is None:
            if now - self.started >= self.STARTUP_GRACE:
                self.stop('navigation_input_never_arrived')
        elif now - self.raw[0] > self.RAW_TIMEOUT or now - self.smooth[0] > self.SMOOTH_TIMEOUT:
            self.stop('stale_navigation_input')
        else:
            # Keep the planner/smoother configuration consistent. This independent
            # output bound also covers the factory driver's asymmetric clamp bug.
            self.reason = 'low_speed_navigation'
            return max(0.0, min(self.lease_v, self.smooth[1])), max(-self.lease_w, min(self.lease_w, self.smooth[2]))
        return 0.0, 0.0

    def departure_output(self, now, linear, fault=None, angular=0.):
        """Explicit internal departure source; do not impersonate planner inputs."""
        if self.state != 'ARMED':
            return 0.0, 0.0
        if fault or now-self.heartbeat_at >= self.HEARTBEAT_TIMEOUT or now-self.started >= self.MAX_RUN:
            self.stop(fault or 'expired_lease')
        elif (not math.isfinite(linear) or not 0 <= linear <= self.MAX_V
              or not math.isfinite(angular) or abs(angular)>self.MAX_W):
            self.stop('invalid_departure_velocity')
        else:
            self.reason = 'a_corner_departure'
            return linear, angular
        return 0.0, 0.0

    def release(self, stationary, no_active_goal):
        if self.state == 'ARMED' or not stationary or not no_active_goal:
            raise ValueError('stop, wait for stationary odometry, and finish/cancel the goal before release')
        self.__init__(self.MAX_V,self.MAX_W,self.MAX_RUN)


def goal_is_free(grid, x, y, radius=.25):
    """Check the full conservative circular footprint in a non-rotated map."""
    resolution, width, height = grid['resolution'], grid['width'], grid['height']
    ox, oy = grid['origin']
    if not all(math.isfinite(v) for v in (x, y, radius, resolution)) or resolution <= 0 or radius <= 0:
        raise ValueError('invalid goal geometry')
    cx, cy = int(math.floor((x-ox)/resolution)), int(math.floor((y-oy)/resolution))
    extent = int(math.ceil(radius/resolution)) + 1
    for row in range(cy-extent, cy+extent+1):
        for col in range(cx-extent, cx+extent+1):
            # Distance to the cell rectangle, not only its center.
            dx = max(ox+col*resolution-x, 0.0, x-(ox+(col+1)*resolution))
            dy = max(oy+row*resolution-y, 0.0, y-(oy+(row+1)*resolution))
            if dx*dx+dy*dy > radius*radius:
                continue
            if not (0 <= col < width and 0 <= row < height):
                return False
            if grid['data'][row*width+col] != 0:
                return False
    return True
