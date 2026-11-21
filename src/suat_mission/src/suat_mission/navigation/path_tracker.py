"""Track one verified map path with a fixed fast profile and continuous feedback."""
import math


BRAKING_LATENCY_S = .20


def angle_error(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


class PathTracker:
    LOOKAHEAD = .16

    def __init__(self, points, yaw, xy_tolerance, yaw_tolerance, speed, contact_end=None):
        self.points = [tuple(points[0])]
        self.lengths = [0.]
        self.contact_end = contact_end
        for point in map(tuple, points[1:]):
            length = math.dist(point, self.points[-1])
            if length > 1e-6:
                self.points.append(point)
                self.lengths.append(self.lengths[-1] + length)
        self.goal_yaw = yaw
        self.xy_tolerance = xy_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.speed = speed
        self.progress = 0.
        self.position_reached = False
        self.v = self.w = 0.

    def command(self, pose, measured_w, dt, in_contact, *, measured_v=0.):
        x, y, yaw = pose
        distance = math.dist((x, y), self.points[-1])
        end = self.lengths[-1]
        if not self.position_reached:
            # Progress can advance only on the nearby part of the path. A
            # parallel return leg cannot pull the lookahead across a corner.
            best = (float('inf'), self.progress)
            for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
                first, last = self.lengths[i:i+2]
                if last < self.progress or first > self.progress + .25:
                    continue
                dx, dy = b[0]-a[0], b[1]-a[1]
                fraction = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/(last-first)**2))
                progress = first + fraction*(last-first)
                if self.progress <= progress <= self.progress + .25:
                    best = min(best, (math.hypot(x-a[0]-fraction*dx, y-a[1]-fraction*dy), progress))
            if best[0] < float('inf'):
                self.progress = best[1]
            if not in_contact and end-self.progress <= self.LOOKAHEAD and distance <= .65*self.xy_tolerance:
                self.position_reached = True
        if self.position_reached:
            error = angle_error(self.goal_yaw, yaw)
            target_v = 0.
            if self.v > 0. or abs(measured_v) >= .015:
                return self.brake(dt)
        else:
            if in_contact:
                tx, ty = self.contact_end
            else:
                look = min(self.progress + self.LOOKAHEAD, end)
                tx, ty = self.points[-1]
                for i, last in enumerate(self.lengths[1:], 1):
                    if last > look:
                        a, b = self.points[i-1:i+1]
                        fraction = (look-self.lengths[i-1])/(last-self.lengths[i-1])
                        tx, ty = (a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1]))
                        break
            error = angle_error(math.atan2(ty-y,tx-x), yaw)
            target_v = min(self.speed['linear'], 1.4*max(distance, end-self.progress))
            if not in_contact and self.near_goal:
                target_v = min(target_v, max(0., distance-.35*self.xy_tolerance))
            target_v *= max(0., math.cos(error)-.15)/.85
        if self.position_reached and abs(error) <= .6*self.yaw_tolerance:
            target_w = 0.
        else:
            cap = self.speed['angular']
            target_w = max(-cap, min(cap, 1.8*error-.30*measured_w))
        self.v = self.slew(self.v, target_v, self.speed['accel_linear']*dt)
        self.w = self.slew(self.w, target_w, self.speed['accel_angular']*dt)
        return self.v, self.w

    @staticmethod
    def slew(previous, target, limit):
        return previous + max(-limit, min(limit, target-previous))

    def brake(self, dt):
        self.v = self.slew(self.v, 0., self.speed['accel_linear']*dt)
        self.w = 0.
        return self.v, self.w

    @property
    def near_goal(self):
        return self.lengths[-1]-self.progress <= self.LOOKAHEAD

    def arrived(self, pose, measured_v, measured_w):
        return (self.position_reached and self.v == self.w == 0.
                and abs(measured_v) < .015 and abs(measured_w) < .03
                and math.dist(pose[:2], self.points[-1]) <= self.xy_tolerance
                and abs(angle_error(self.goal_yaw, pose[2])) <= self.yaw_tolerance)


def braking_speed(distance, deceleration):
    return deceleration * (math.sqrt(BRAKING_LATENCY_S**2 + 2*distance/deceleration)-BRAKING_LATENCY_S)


def braking_arc(pose, v, w, deceleration, latency=BRAKING_LATENCY_S):
    """Sample a conservative continuation of the current differential-drive arc."""
    x, y, yaw = pose
    horizon = latency + abs(v)/(2*deceleration)
    steps = max(1, math.ceil(abs(v)*horizon/.02))
    points = []
    for i in range(1, steps+1):
        t = horizon*i/steps
        heading = yaw+w*t
        if abs(w) < 1e-5:
            points.append((x+v*t*math.cos(yaw), y+v*t*math.sin(yaw)))
        else:
            points.append((x+v/w*(math.sin(heading)-math.sin(yaw)),
                           y+v/w*(math.cos(yaw)-math.cos(heading))))
    return points
