"""手眼标定：把相机系坐标和机械臂系坐标对齐成一个刚体变换。

做法是最小二乘的刚体配准（Kabsch/Umeyama）：给定 N 组对应点
    p_cam_i  <->  p_arm_i
求 R, t 使 sum |R p_cam_i + t - p_arm_i|^2 最小。

只用 SVD，不需要初值；但要求点集不退化：
- 至少 4 个点（3 个共面点会留下镜像歧义）；
- 三点不能共线；
- 分布要张开，都挤在一小块时旋转量的解会很飘。

这里返回的 Extrinsics 沿用 depth_geometry 的定义：相机系 -> 臂系，毫米。
"""
from dataclasses import dataclass
from math import isfinite
from typing import Sequence, Tuple

import numpy as np

from suat_mission.manipulation.depth_geometry import Extrinsics


@dataclass(frozen=True)
class FitResult:
    """标定结果连同可信度指标，现场必须看残差再看能不能用。"""

    extrinsics: Extrinsics
    residual_rms_mm: float
    residual_max_mm: float
    point_count: int
    spread_mm: float

    def usable(self, max_rms_mm: float = 8.0, max_point_mm: float = 15.0) -> bool:
        return (self.point_count >= 4 and self.residual_rms_mm <= max_rms_mm
                and self.residual_max_mm <= max_point_mm)

    def describe(self) -> str:
        return ('%d points, rms %.2f mm, max %.2f mm, spread %.1f mm'
                % (self.point_count, self.residual_rms_mm, self.residual_max_mm,
                   self.spread_mm))


def _as_points(points: Sequence[Sequence[float]], name: str) -> np.ndarray:
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError('%s must be an Nx3 array' % name)
    if array.shape[0] < 3:
        raise ValueError('%s needs at least 3 points' % name)
    if not np.all(np.isfinite(array)):
        raise ValueError('%s contains non-finite values' % name)
    return array


def fit_rigid(camera_points: Sequence[Sequence[float]],
              arm_points: Sequence[Sequence[float]]) -> FitResult:
    """由对应点求 相机系 -> 臂系 的刚体变换。"""
    cam = _as_points(camera_points, 'camera_points')
    arm = _as_points(arm_points, 'arm_points')
    if cam.shape != arm.shape:
        raise ValueError('camera and arm point sets must have the same size')

    cam_center = cam.mean(axis=0)
    arm_center = arm.mean(axis=0)
    cam_zero = cam - cam_center
    arm_zero = arm - arm_center

    # 退化检查：点都挤在一起时旋转解不可信
    spread = float(np.sqrt(np.mean(np.sum(cam_zero ** 2, axis=1))))
    if spread < 5.0:
        raise ValueError('camera points are too clustered (spread %.1f mm)' % spread)

    covariance = cam_zero.T.dot(arm_zero)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T.dot(u.T)
    if np.linalg.det(rotation) < 0:
        # 出现镜像解，翻转最小奇异值对应的轴
        vt[-1, :] *= -1.0
        rotation = vt.T.dot(u.T)
    translation = arm_center - rotation.dot(cam_center)

    predicted = cam.dot(rotation.T) + translation
    errors = np.linalg.norm(predicted - arm, axis=1)

    return FitResult(extrinsics=Extrinsics.from_matrix(rotation, translation),
                     residual_rms_mm=float(np.sqrt(np.mean(errors ** 2))),
                     residual_max_mm=float(errors.max()),
                     point_count=int(cam.shape[0]),
                     spread_mm=spread)


def apply(extrinsics: Extrinsics, camera_points: Sequence[Sequence[float]]) -> np.ndarray:
    """把一个相机系点集整体变换到臂系，用于标定后复核。"""
    points = np.asarray(camera_points, dtype=np.float64)
    matrix = extrinsics.matrix
    homogeneous = np.column_stack((points, np.ones(points.shape[0])))
    return homogeneous.dot(matrix.T)[:, :3]


def check_pairs(camera_points: Sequence[Sequence[float]],
                arm_points: Sequence[Sequence[float]],
                max_pair_error_mm: float = 400.0) -> Tuple[bool, str]:
    """粗筛：同一组对应点在两个坐标系里的距离差太多，说明配对本身错了。

    标定时最怕"点配错"，残差会很大但求解器照样给出一个 R、t。
    这里先用两两间距离做一次不依赖坐标系的检查。
    """
    cam = _as_points(camera_points, 'camera_points')
    arm = _as_points(arm_points, 'arm_points')
    if cam.shape != arm.shape:
        raise ValueError('camera and arm point sets must have the same size')
    count = cam.shape[0]
    worst = 0.0
    for i in range(count):
        for j in range(i + 1, count):
            d_cam = float(np.linalg.norm(cam[i] - cam[j]))
            d_arm = float(np.linalg.norm(arm[i] - arm[j]))
            worst = max(worst, abs(d_cam - d_arm))
    if worst > max_pair_error_mm:
        return False, 'pair distance mismatch up to %.1f mm' % worst
    return True, 'pair distances agree within %.1f mm' % worst


def sanity(extrinsics: Extrinsics) -> str:
    """把外参翻译成人能读的一句话，避免符号错了还照跑。"""
    rotation = extrinsics.rotation
    translation = extrinsics.translation
    parts = []
    for axis, index in (('x', 0), ('y', 1), ('z', 2)):
        vector = rotation[:, index]
        dominant = int(np.argmax(np.abs(vector)))
        sign = '+' if vector[dominant] >= 0 else '-'
        parts.append('cam_%s -> arm_%s%s' % (axis, 'xyz'[dominant], sign))
    return ('; '.join(parts) +
            ' | offset (%.1f, %.1f, %.1f) mm' % tuple(translation))
