"""Offline, bounded planar approach to a known marker; no ROS or motor I/O.

Inputs must already be transformed into base_footprint: +x forward, +y left.
This centers the marker's bearing; it does not align to the marker plane normal.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class AlignmentResult:
    linear: float
    angular: float
    status: str


@dataclass(frozen=True)
class AlignmentConfig:
    target_distance: float = 0.5
    distance_tolerance: float = 0.04
    angle_tolerance: float = 0.04
    rotate_first_angle: float = 0.25
    max_range: float = 2.0
    max_age: float = 0.5
    linear_gain: float = 0.4
    angular_gain: float = 0.8
    max_linear: float = 0.10
    max_angular: float = 0.35


def alignment_command(x, y, stamp, now, cfg=AlignmentConfig()):
    values = (x, y, stamp, now, cfg.target_distance, cfg.distance_tolerance,
              cfg.angle_tolerance, cfg.rotate_first_angle, cfg.max_range, cfg.max_age,
              cfg.linear_gain, cfg.angular_gain, cfg.max_linear, cfg.max_angular)
    if not all(math.isfinite(v) for v in values):
        return AlignmentResult(0.0, 0.0, 'invalid')
    if (min(cfg.target_distance, cfg.distance_tolerance, cfg.angle_tolerance, cfg.max_range,
            cfg.max_age, cfg.linear_gain, cfg.angular_gain, cfg.max_linear, cfg.max_angular) <= 0
            or cfg.rotate_first_angle < cfg.angle_tolerance or cfg.max_range <= cfg.target_distance):
        return AlignmentResult(0.0, 0.0, 'invalid_config')
    if now - stamp > cfg.max_age or stamp - now > 0.05:
        return AlignmentResult(0.0, 0.0, 'stale')
    distance = math.hypot(x, y)
    if x <= 0 or distance > cfg.max_range:
        return AlignmentResult(0.0, 0.0, 'out_of_view')
    bearing = math.atan2(y, x)
    distance_error = distance - cfg.target_distance
    if abs(distance_error) <= cfg.distance_tolerance and abs(bearing) <= cfg.angle_tolerance:
        return AlignmentResult(0.0, 0.0, 'aligned')
    angular = max(-cfg.max_angular, min(cfg.max_angular, cfg.angular_gain * bearing))
    # Never automatically reverse. It requires a separately verified rear clearance.
    if distance_error < -cfg.distance_tolerance:
        return AlignmentResult(0.0, 0.0, 'too_close')
    if abs(bearing) > cfg.rotate_first_angle:
        return AlignmentResult(0.0, angular, 'turning')
    linear = max(0.0, min(cfg.max_linear, cfg.linear_gain * max(0.0, distance_error)))
    return AlignmentResult(linear, angular, 'approaching')
