"""机械臂笛卡尔分段路径：抬升 -> 平移 -> 竖直接近 -> 下降 -> 抬升。

swiftpro 只接收末端位置（position_write_topic，单位毫米），没有 MoveIt，
因此这里不是关节空间规划，而是可校验、可限速的末端折线：

1. 先把末端抬到统一安全高度，避免拖动时扫到相邻物块；
2. 在安全高度上平移，再竖直下降，保证最后一段是纯竖直；
3. 每一点都做工作空间校验，越界立即报错而不是发给硬件。

纯逻辑模块，不 import rospy。
"""
from dataclasses import dataclass
from math import ceil, isfinite, sqrt
from typing import List, Sequence, Tuple


# Measured SwiftPro speeds from the teammate's fast pick/place program.
_FEED_SPEEDS = ((1000.0, 20.0), (10000.0, 44.3), (40000.0, 78.7), (80000.0, 95.4))
FEED_FAST = 80000.0
FEED_APPROACH = 40000.0
FEED_LOADED = 10000.0


def speed_of(feed: float) -> float:
    """Interpolate measured tool speed in mm/s at a SwiftPro feed setting."""
    if feed <= _FEED_SPEEDS[0][0]:
        return _FEED_SPEEDS[0][1]
    for (low_feed, low_speed), (high_feed, high_speed) in zip(_FEED_SPEEDS, _FEED_SPEEDS[1:]):
        if feed <= high_feed:
            ratio = (feed - low_feed) / (high_feed - low_feed)
            return low_speed + ratio * (high_speed - low_speed)
    return _FEED_SPEEDS[-1][1]


def travel_dwell(distance_mm: float, feed: float) -> float:
    """Settling interval used by the team's single-command Arm.goto()."""
    return max(0.45, distance_mm / max(speed_of(feed), 1.0) + 0.55)


def vertical_lift_points(start: Sequence[float], end_z: float, step: float = 0.0):
    """Keep contact XY fixed. With no segmentation, send one lift target."""
    x, y, z = map(float, start)
    if not all(isfinite(v) for v in (x, y, z, end_z, step)) or end_z < z:
        raise ValueError('lift needs a finite endpoint at or above contact height')
    while z < end_z:
        z = end_z if step <= 0 else min(end_z, z + step)
        yield x, y, z


@dataclass(frozen=True)
class ArmLimits:
    """机械臂末端可达的笛卡尔工作空间，单位毫米。现场需按实测收紧。"""

    x_min: float = 40.0
    x_max: float = 280.0
    y_min: float = -180.0
    y_max: float = 180.0
    z_min: float = -80.0
    z_max: float = 200.0

    def __post_init__(self):
        values = (self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max)
        if not all(isfinite(v) for v in values):
            raise ValueError('non-finite arm limits')
        if not (self.x_min < self.x_max and self.y_min < self.y_max and self.z_min < self.z_max):
            raise ValueError('arm limits must be ordered min < max')

    def outside(self, xyz: Sequence[float]) -> str:
        x, y, z = (float(v) for v in xyz)
        if not all(isfinite(v) for v in (x, y, z)):
            return 'non-finite pose'
        if not self.x_min <= x <= self.x_max:
            return 'x=%.1f outside [%.1f, %.1f]' % (x, self.x_min, self.x_max)
        if not self.y_min <= y <= self.y_max:
            return 'y=%.1f outside [%.1f, %.1f]' % (y, self.y_min, self.y_max)
        if not self.z_min <= z <= self.z_max:
            return 'z=%.1f outside [%.1f, %.1f]' % (z, self.z_min, self.z_max)
        return ''

    def contains(self, xyz: Sequence[float]) -> bool:
        return not self.outside(xyz)

    def check(self, xyz: Sequence[float]) -> None:
        reason = self.outside(xyz)
        if reason:
            raise ValueError('arm pose rejected: ' + reason)


@dataclass(frozen=True)
class ArmPathConfig:
    """路径节奏参数。speed 是末端线速度，用于估算每步停留时间。"""

    safe_z_mm: float = 150.0
    approach_mm: float = 60.0
    lift_mm: float = 80.0
    release_mm: float = 40.0
    seal_hold_s: float = 0.8
    release_hold_s: float = 0.6
    lateral_step_mm: float = 15.0
    vertical_step_mm: float = 6.0
    lateral_speed_mm_s: float = 60.0
    vertical_speed_mm_s: float = 25.0

    def __post_init__(self):
        values = (self.safe_z_mm, self.approach_mm, self.lift_mm, self.release_mm,
                  self.seal_hold_s, self.release_hold_s, self.lateral_step_mm,
                  self.vertical_step_mm, self.lateral_speed_mm_s, self.vertical_speed_mm_s)
        if not all(isfinite(v) for v in values):
            raise ValueError('non-finite path config')
        if self.approach_mm <= 0 or self.lift_mm <= 0 or self.release_mm <= 0:
            raise ValueError('approach, lift and release must be positive')
        if self.lateral_step_mm <= 0 or self.vertical_step_mm <= 0:
            raise ValueError('step sizes must be positive')
        if self.lateral_speed_mm_s <= 0 or self.vertical_speed_mm_s <= 0:
            raise ValueError('speeds must be positive')


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    stage: str
    dwell_s: float = 0.0

    def xyz(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


def _step_time(distance_mm: float, speed_mm_s: float) -> float:
    return round(distance_mm / speed_mm_s, 3)


def _line(start: Sequence[float], end: Sequence[float], step_mm: float,
          stage: str, speed_mm_s: float) -> List[Waypoint]:
    """从 start 到 end 的等距插值，返回不含 start 的点列。"""
    dx = float(end[0]) - float(start[0])
    dy = float(end[1]) - float(start[1])
    dz = float(end[2]) - float(start[2])
    distance = sqrt(dx * dx + dy * dy + dz * dz)
    if distance < 1e-6:
        return []
    count = max(1, int(ceil(distance / step_mm)))
    dwell = _step_time(distance, speed_mm_s) / count
    points = []
    for index in range(1, count + 1):
        ratio = index / float(count)
        points.append(Waypoint(float(start[0]) + dx * ratio,
                               float(start[1]) + dy * ratio,
                               float(start[2]) + dz * ratio,
                               stage, dwell))
    return points


def _validate(points: Sequence[Waypoint], limits: ArmLimits) -> None:
    for point in points:
        limits.check(point.xyz())


def plan_pick(start: Sequence[float], grasp: Sequence[float],
              limits: ArmLimits, cfg: ArmPathConfig) -> List[Waypoint]:
    """生成抓取路径：安全高度 -> 目标正上方 -> 竖直下降到抓取高度 -> 抬升。"""
    limits.check(start)
    limits.check(grasp)
    gx, gy, gz = (float(v) for v in grasp)
    sx, sy, sz = (float(v) for v in start)
    safe_z = max(cfg.safe_z_mm, gz + cfg.approach_mm)
    if safe_z > limits.z_max:
        raise ValueError('safe height exceeds arm z limit')
    if gz + cfg.lift_mm > limits.z_max:
        raise ValueError('lift height exceeds arm z limit')

    points: List[Waypoint] = []
    current = (sx, sy, sz)

    if sz < safe_z:
        segment = _line(current, (sx, sy, safe_z), cfg.vertical_step_mm,
                        'retract', cfg.vertical_speed_mm_s)
        points.extend(segment)
        current = (sx, sy, safe_z)

    points.extend(_line(current, (gx, gy, safe_z), cfg.lateral_step_mm,
                        'travel', cfg.lateral_speed_mm_s))
    current = (gx, gy, safe_z)

    approach_z = gz + cfg.approach_mm
    points.extend(_line(current, (gx, gy, approach_z), cfg.vertical_step_mm,
                        'approach', cfg.vertical_speed_mm_s))

    descend = _line((gx, gy, approach_z), (gx, gy, gz), cfg.vertical_step_mm,
                    'descend', cfg.vertical_speed_mm_s)
    if descend:
        descend[-1] = Waypoint(gx, gy, gz, 'descend', cfg.seal_hold_s)
    points.extend(descend)

    points.extend(_line((gx, gy, gz), (gx, gy, gz + cfg.lift_mm), cfg.vertical_step_mm,
                        'lift', cfg.vertical_speed_mm_s))

    _validate(points, limits)
    return points


def plan_place(start: Sequence[float], place: Sequence[float],
               limits: ArmLimits, cfg: ArmPathConfig) -> List[Waypoint]:
    """生成放置路径：安全高度 -> 放置点正上方 -> 下降到放置高度 -> 抬升脱离。"""
    limits.check(start)
    limits.check(place)
    px, py, pz = (float(v) for v in place)
    sx, sy, sz = (float(v) for v in start)
    safe_z = max(cfg.safe_z_mm, pz + cfg.release_mm, sz)
    if safe_z > limits.z_max:
        raise ValueError('safe height exceeds arm z limit')

    points: List[Waypoint] = []
    current = (sx, sy, sz)
    if sz < safe_z:
        points.extend(_line(current, (sx, sy, safe_z), cfg.vertical_step_mm,
                            'retract', cfg.vertical_speed_mm_s))
        current = (sx, sy, safe_z)

    points.extend(_line(current, (px, py, safe_z), cfg.lateral_step_mm,
                        'travel', cfg.lateral_speed_mm_s))

    descend = _line((px, py, safe_z), (px, py, pz), cfg.vertical_step_mm,
                    'descend', cfg.vertical_speed_mm_s)
    if descend:
        descend[-1] = Waypoint(px, py, pz, 'descend', cfg.release_hold_s)
    points.extend(descend)

    points.extend(_line((px, py, pz), (px, py, pz + cfg.release_mm), cfg.vertical_step_mm,
                        'retract', cfg.vertical_speed_mm_s))

    _validate(points, limits)
    return points


def path_duration(points: Sequence[Waypoint]) -> float:
    return round(sum(max(0.0, p.dwell_s) for p in points), 3)


def path_summary(points: Sequence[Waypoint]) -> str:
    stages = {}
    for point in points:
        stages[point.stage] = stages.get(point.stage, 0) + 1
    detail = ', '.join('%s=%d' % (name, count) for name, count in stages.items())
    return '%d waypoints (%s), estimated %.1fs' % (len(points), detail, path_duration(points))
