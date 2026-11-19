"""Read-only local depth-plane measurement in the camera optical frame."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SurfaceMeasurement:
    depth_m: float
    angle_to_camera_deg: float
    normal: tuple
    samples: int
    residual_m: float


@dataclass(frozen=True)
class ReferencePlane:
    normal: tuple
    offset_m: float
    inliers: int
    sampled_points: int


def measure_surface(depth, x, y, intrinsics, depth_scale=0.001, radius=10,
                    min_depth=0.1, max_depth=3.0, max_depth_deviation=0.035,
                    min_samples=25):
    """Fit a plane around an image pixel; returns None for unreliable input.

    The angle is to the camera optical axis, not to gravity. A facing plane is
    not necessarily a horizontal top face unless camera pose has been validated.
    """
    fx, fy, cx, cy = map(float, intrinsics)
    if (not all(math.isfinite(v) for v in (x, y, fx, fy, cx, cy, depth_scale))
            or fx <= 0 or fy <= 0 or depth_scale <= 0 or radius < 1
            or depth is None or depth.ndim != 2):
        return None
    px, py = int(round(x)), int(round(y))
    if not (0 <= px < depth.shape[1] and 0 <= py < depth.shape[0]):
        return None
    yy, xx = np.mgrid[max(0,py-radius):min(depth.shape[0],py+radius+1),
                      max(0,px-radius):min(depth.shape[1],px+radius+1)]
    z = depth[yy, xx].astype(np.float64) * depth_scale
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    if int(valid.sum()) < min_samples:
        return None
    median_depth = float(np.median(z[valid]))
    valid &= np.abs(z-median_depth) <= max_depth_deviation
    if int(valid.sum()) < min_samples:
        return None
    points = np.column_stack(((xx[valid]-cx)*z[valid]/fx,
                              (yy[valid]-cy)*z[valid]/fy, z[valid]))
    centered = points - points.mean(axis=0)
    _, singular_values, vh = np.linalg.svd(centered, full_matrices=False)
    if len(singular_values) < 3 or singular_values[1] <= 1e-8:
        return None
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(normal[2])))))
    residual = float(np.percentile(np.abs(centered.dot(normal)), 80))
    return SurfaceMeasurement(median_depth, angle, tuple(float(v) for v in normal), len(points), residual)


def estimate_reference_plane(depth, intrinsics, depth_scale=0.001, stride=16,
                             iterations=80, tolerance_m=0.008, min_points=100,
                             min_fraction=0.35, min_normal_z=0.75):
    """Estimate a dominant camera-facing plane, not guaranteed to be the floor.

    Experiment only: a tabletop, marker board, or other large plane may dominate.
    The caller must validate scene geometry before using the plane for any action.
    """
    if depth is None or depth.ndim != 2 or stride < 1 or iterations < 1 or depth_scale <= 0:
        return None
    fx, fy, cx, cy = map(float, intrinsics)
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy, depth_scale)) or fx <= 0 or fy <= 0:
        return None
    yy, xx = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[yy, xx].astype(np.float64) * depth_scale
    valid = np.isfinite(z) & (z > .15) & (z < 2.0)
    points = np.column_stack(((xx[valid]-cx)*z[valid]/fx,
                              (yy[valid]-cy)*z[valid]/fy, z[valid]))
    if len(points) < max(3, min_points):
        return None
    rng = np.random.RandomState(3)
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = float(np.linalg.norm(normal))
        if length < 1e-7:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < min_normal_z:
            continue
        mask = np.abs(points.dot(normal) - normal.dot(a)) < tolerance_m
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
    if best_mask is None or best_count < min_points or best_count / float(len(points)) < min_fraction:
        return None
    selected = points[best_mask]
    center = selected.mean(axis=0)
    _, singular_values, vh = np.linalg.svd(selected-center, full_matrices=False)
    if len(singular_values) < 3 or singular_values[1] <= 1e-8:
        return None
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    if normal[2] < min_normal_z:
        return None
    return ReferencePlane(tuple(float(v) for v in normal), float(normal.dot(center)),
                          best_count, len(points))


def height_above_reference(surface, x, y, intrinsics, plane):
    if surface is None or plane is None:
        return None
    fx, fy, cx, cy = map(float, intrinsics)
    if fx <= 0 or fy <= 0:
        return None
    z = surface.depth_m
    point = np.array([(x-cx)*z/fx, (y-cy)*z/fy, z])
    height = plane.offset_m - np.dot(np.asarray(plane.normal), point)
    return float(height) if math.isfinite(float(height)) else None
