"""RGB-D 目标定位：框内取深度、求深度质心、反投影成三维点。

设计约束：
- 纯逻辑，不 import rospy，可直接离线单测；
- 长度单位统一为毫米，和 swiftpro position 消息一致；
- 相机系沿用 ROS optical frame：x 向右、y 向下、z 向前。

厂商原方案只用像素做二维线性映射（x = kx*py + bx），物体高度一变就偏。
这里补两件事：用深度求稳健的物体表面高度，用深度簇质心代替框中心。
"""
from dataclasses import dataclass
from math import acos, degrees, exp, isfinite, sqrt
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔模型内参，来自 /camera/color/camera_info 的 K[0],K[4],K[2],K[5]。"""

    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if not all(isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy)):
            raise ValueError('non-finite camera intrinsics')
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError('focal length must be positive')
        if self.cx <= 0 or self.cy <= 0:
            raise ValueError('principal point must be positive')

    @classmethod
    def from_k(cls, k: Sequence[float]):
        if len(k) < 9:
            raise ValueError('camera matrix needs at least 9 values')
        return cls(float(k[0]), float(k[4]), float(k[2]), float(k[5]))


@dataclass(frozen=True)
class DepthConfig:
    """深度采样阈值。默认值按 0.2~0.7 m 的台面抓取场景保守给定。"""

    min_depth_mm: float = 120.0
    max_depth_mm: float = 700.0
    shrink: float = 0.15
    cluster_tolerance_mm: float = 12.0
    reference_percentile: float = 20.0
    min_valid_pixels: int = 60
    min_cluster_ratio: float = 0.25
    max_spread_mm: float = 40.0
    max_residual_mm: float = 6.0
    max_gradient_mm_px: float = 0.8

    def __post_init__(self):
        values = (self.min_depth_mm, self.max_depth_mm, self.cluster_tolerance_mm,
                  self.reference_percentile, self.max_spread_mm,
                  self.max_residual_mm, self.max_gradient_mm_px)
        if not all(isfinite(v) for v in values):
            raise ValueError('non-finite depth config')
        if not 0 < self.min_depth_mm < self.max_depth_mm:
            raise ValueError('depth range must be positive and ordered')
        if not 0.0 <= self.shrink < 0.5:
            raise ValueError('shrink must be within [0, 0.5)')
        if self.cluster_tolerance_mm <= 0 or self.max_spread_mm <= 0:
            raise ValueError('cluster tolerance and spread limit must be positive')
        if self.max_residual_mm <= 0 or self.max_gradient_mm_px <= 0:
            raise ValueError('residual and gradient limits must be positive')
        if not 0 < self.reference_percentile < 100:
            raise ValueError('reference percentile must be within (0, 100)')
        if self.min_valid_pixels < 4:
            raise ValueError('min_valid_pixels is too small to be meaningful')
        if not 0 < self.min_cluster_ratio <= 1:
            raise ValueError('min_cluster_ratio must be within (0, 1]')


@dataclass(frozen=True)
class DepthSample:
    """一次框内深度采样结果。u、v 是深度簇质心，不是框中心。

    spread_mm 是簇内原始深度跨度，斜视的平面也会很大，只作诊断；
    判断"平不平"要看 residual_mm —— 拟合平面后的残差，斜平面照样接近 0。
    """

    z_mm: float
    u: float
    v: float
    valid_pixels: int
    cluster_pixels: int
    spread_mm: float
    coverage: float
    residual_mm: float = 0.0
    gradient_mm_px: float = 0.0
    flatness: float = 1.0
    normal: Tuple[float, float, float] = (0.0, 0.0, 1.0)

    @property
    def tilt_deg(self) -> float:
        """表面法线与光轴的夹角。吸盘垂直下压时要按这个角度补。"""
        return degrees(acos(max(-1.0, min(1.0, self.normal[2]))))

    def quality(self) -> float:
        """0~1 的可用性评分，用于上层决定是否值得伸手。"""
        coverage_term = min(1.0, self.coverage)
        pixel_term = min(1.0, self.cluster_pixels / 400.0)
        return round(coverage_term * self.flatness * pixel_term, 4)


@dataclass(frozen=True)
class ObjectPoint:
    """一个目标的定位结果：像素、相机系坐标、可选机械臂系坐标。"""

    u: float
    v: float
    z_mm: float
    camera_xyz_mm: Tuple[float, float, float]
    arm_xyz_mm: Optional[Tuple[float, float, float]]
    spread_mm: float
    coverage: float
    quality: float
    residual_mm: float = 0.0
    tilt_deg: float = 0.0

    def accepted(self, min_quality: float = 0.35) -> bool:
        return self.quality >= min_quality and self.arm_xyz_mm is not None


@dataclass(frozen=True)
class Extrinsics:
    """相机系到机械臂系的刚体变换，4x4 行主序，平移单位为毫米。"""

    values: Tuple[float, ...]

    def __post_init__(self):
        if len(self.values) != 16:
            raise ValueError('extrinsics require exactly 16 values')
        if not all(isfinite(v) for v in self.values):
            raise ValueError('non-finite extrinsics')
        bottom = self.values[12:16]
        if any(abs(a - b) > 1e-6 for a, b in zip(bottom, (0.0, 0.0, 0.0, 1.0))):
            raise ValueError('last extrinsics row must be 0 0 0 1')
        rotation = self.rotation
        for row in rotation:
            norm = sqrt(sum(v * v for v in row))
            if abs(norm - 1.0) > 1e-3:
                raise ValueError('rotation rows must be unit length')

    @classmethod
    def from_values(cls, values: Sequence[float]):
        return cls(tuple(float(v) for v in values))

    @classmethod
    def from_params(cls, rx: float, ry: float, rz: float,
                    tx: float, ty: float, tz: float):
        """按 ZYX 欧拉角（弧度）与毫米平移构造。现场标定前先用手工测量的近似值。"""
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)
        rot = np.array([
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ], dtype=float)
        return cls.from_matrix(rot, (tx, ty, tz))

    @classmethod
    def from_matrix(cls, rotation, translation):
        rot = np.asarray(rotation, dtype=float)
        if rot.shape != (3, 3):
            raise ValueError('rotation must be 3x3')
        tra = np.asarray(translation, dtype=float).reshape(3)
        matrix = np.eye(4)
        matrix[:3, :3] = rot
        matrix[:3, 3] = tra
        return cls.from_values(matrix.reshape(-1))

    @property
    def matrix(self) -> np.ndarray:
        return np.array(self.values, dtype=float).reshape(4, 4)

    @property
    def rotation(self) -> np.ndarray:
        return self.matrix[:3, :3]

    @property
    def translation(self) -> np.ndarray:
        return self.matrix[:3, 3]

    def apply(self, point_mm: Sequence[float]) -> Tuple[float, float, float]:
        p = np.array([float(point_mm[0]), float(point_mm[1]), float(point_mm[2]), 1.0])
        out = self.matrix.dot(p)
        if abs(out[3] - 1.0) > 1e-6:
            raise ValueError('extrinsics produced a projective result')
        return (float(out[0]), float(out[1]), float(out[2]))


def sample_depth_centroid(depth_mm, bbox: Sequence[float],
                          cfg: DepthConfig = DepthConfig()) -> DepthSample:
    """在框内取最靠近相机的一簇深度，返回其高度与质心像素。

    物体顶面比桌面近，所以用低分位数找参考面，再按容差取簇。
    框内混入背景时，质心自然偏向真实物体，而不是框中心。
    """
    array = np.asarray(depth_mm)
    if array.ndim != 2:
        raise ValueError('depth image must be a 2-D array')
    cx, cy, width, height = (float(v) for v in bbox)
    if not all(isfinite(v) for v in (cx, cy, width, height)):
        raise ValueError('non-finite bbox')
    if width <= 2 or height <= 2:
        raise ValueError('bbox is too small for depth sampling')

    rows, cols = array.shape[:2]
    half_w = 0.5 * width * (1.0 - cfg.shrink)
    half_h = 0.5 * height * (1.0 - cfg.shrink)
    x0 = max(0, int(round(cx - half_w)))
    x1 = min(cols, int(round(cx + half_w)))
    y0 = max(0, int(round(cy - half_h)))
    y1 = min(rows, int(round(cy + half_h)))
    if x1 - x0 < 2 or y1 - y0 < 2:
        raise ValueError('bbox lies outside the depth image')

    roi = array[y0:y1, x0:x1].astype(np.float32)
    valid = np.isfinite(roi) & (roi >= cfg.min_depth_mm) & (roi <= cfg.max_depth_mm)
    valid_count = int(valid.sum())
    if valid_count < cfg.min_valid_pixels:
        raise ValueError('too few valid depth pixels: %d' % valid_count)

    values = roi[valid]
    reference = float(np.percentile(values, cfg.reference_percentile))
    cluster = valid & (np.abs(roi - reference) <= cfg.cluster_tolerance_mm)
    cluster_count = int(cluster.sum())
    if cluster_count < cfg.min_valid_pixels:
        raise ValueError('depth cluster too small: %d' % cluster_count)
    if cluster_count < cfg.min_cluster_ratio * valid_count:
        raise ValueError('depth cluster is not dominant: %d/%d' % (cluster_count, valid_count))

    flat = roi[cluster]
    z = float(np.median(flat))
    spread = float(np.percentile(flat, 90) - np.percentile(flat, 10))
    if spread > cfg.max_spread_mm:
        raise ValueError('depth cluster spans too much: %.1f mm' % spread)

    cluster_rows, cluster_cols = np.nonzero(cluster)
    u_mean = x0 + float(cluster_cols.mean())
    v_mean = y0 + float(cluster_rows.mean())
    residual, gradient, normal = _fit_plane(cluster_cols + x0, cluster_rows + y0,
                                            flat)
    if gradient > cfg.max_gradient_mm_px:
        raise ValueError('surface is too oblique: %.2f mm/px' % gradient)
    if residual > cfg.max_residual_mm:
        raise ValueError('surface is not planar: residual %.1f mm' % residual)

    return DepthSample(z_mm=z,
                       u=u_mean,
                       v=v_mean,
                       valid_pixels=valid_count,
                       cluster_pixels=cluster_count,
                       spread_mm=spread,
                       coverage=cluster_count / float(valid_count),
                       residual_mm=residual,
                       gradient_mm_px=gradient,
                       flatness=exp(-(residual / cfg.max_residual_mm) ** 2),
                       normal=normal)


def _fit_plane(cols, rows, values):
    """最小二乘拟合 z = a*u + b*v + c，返回 (残差 rms, 梯度模长, 法线)。

    坐标先减去质心再拟合，避免 u、v 有几百像素时法方程病态。
    判定平整必须看残差：斜平面原始深度跨度很大，但残差依然接近 0。
    """
    us = np.asarray(cols, dtype=np.float64)
    vs = np.asarray(rows, dtype=np.float64)
    zs = np.asarray(values, dtype=np.float64)
    u0 = us.mean()
    v0 = vs.mean()
    design = np.column_stack((us - u0, vs - v0, np.ones(zs.size)))
    coef, *_ = np.linalg.lstsq(design, zs, rcond=None)
    residual = float(np.sqrt(np.mean((zs - design.dot(coef)) ** 2)))
    gradient = float(np.hypot(coef[0], coef[1]))
    normal = np.array([-coef[0], -coef[1], 1.0], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    return residual, gradient, (float(normal[0]), float(normal[1]), float(normal[2]))


def back_project(u: float, v: float, z_mm: float,
                 intrinsics: CameraIntrinsics) -> Tuple[float, float, float]:
    """像素 + 深度反投影为相机系坐标，单位毫米。"""
    if not all(isfinite(val) for val in (u, v, z_mm)):
        raise ValueError('non-finite pixel or depth')
    if z_mm <= 0:
        raise ValueError('depth must be positive')
    z_m = z_mm / 1000.0
    x_m = (u - intrinsics.cx) * z_m / intrinsics.fx
    y_m = (v - intrinsics.cy) * z_m / intrinsics.fy
    return (x_m * 1000.0, y_m * 1000.0, z_m * 1000.0)


def localize(depth_mm, bbox: Sequence[float], intrinsics: CameraIntrinsics,
             extrinsics: Optional[Extrinsics] = None,
             cfg: DepthConfig = DepthConfig()) -> ObjectPoint:
    """框内定位一个目标：深度质心 -> 反投影 -> 可选转到机械臂系。"""
    sample = sample_depth_centroid(depth_mm, bbox, cfg)
    camera_xyz = back_project(sample.u, sample.v, sample.z_mm, intrinsics)
    arm_xyz = extrinsics.apply(camera_xyz) if extrinsics is not None else None
    return ObjectPoint(u=sample.u, v=sample.v, z_mm=sample.z_mm,
                       camera_xyz_mm=camera_xyz, arm_xyz_mm=arm_xyz,
                       spread_mm=sample.spread_mm, coverage=sample.coverage,
                       quality=sample.quality(), residual_mm=sample.residual_mm,
                       tilt_deg=sample.tilt_deg)


def surface_height_mm(point: ObjectPoint) -> float:
    """目标上表面相对相机的深度，用于和标定平面比较得到抓取高度。"""
    return point.z_mm
