"""机械臂可达范围（按高度分层）与抓取点校验。

SwiftPro 是"基座偏航 + 大臂/小臂两个俯仰 + 腕部保持竖直"的平行四边形臂，
所以在任意给定高度 z 上，吸盘能到达的水平位置是一个以肩关节为心的圆环：

    r_min(z) <= r <= r_max(z)

r 越大越远；抬高时 r_max 收缩、r_min 张开，中间那圈才够得着。
这些数字全部来自 swiftpro 的 URDF（pro_model.xacro），不是猜的：

    肩关节   x = 13.2 mm,  z = 105.6 mm   （Joint1 高度 72.3 + Joint2 偏置 33.3）
    大臂 L1  = 142.07 mm                  （Joint3 原点 z 偏置）
    小臂 L2  = 158.52 mm                  （Joint8 原点 x 偏置）
    腕到吸盘 TOOL = 27.03 mm              （Joint9 原点 z 偏置，腕竖直时）

注意：这是几何可达域，不含固件自带的限位、自碰撞和台面干涉。
实机验收要用实测点集复核，`envelope_table()` 打印的表可以拿去和实测对照。
"""
from dataclasses import dataclass
from math import hypot, isfinite, sqrt
from typing import List, Optional, Sequence, Tuple

SHOULDER_X_MM = 13.2
SHOULDER_Z_MM = 105.6
UPPER_ARM_MM = 142.07
FOREARM_MM = 158.52
TOOL_DROP_MM = 27.03


@dataclass(frozen=True)
class Reach:
    """某一高度上的水平可达环。r 以肩关节竖直轴为圆心。"""

    z_mm: float
    r_min_mm: float
    r_max_mm: float

    @property
    def empty(self) -> bool:
        return self.r_max_mm <= self.r_min_mm


@dataclass(frozen=True)
class ReachCheck:
    """一次抓取点的可达判定，reason 直接可以打日志给现场看。"""

    ok: bool
    reason: str
    radius_mm: float
    reach: Reach

    def describe(self) -> str:
        if self.ok:
            return 'reachable r=%.1f in [%.1f, %.1f] at z=%.1f' % (
                self.radius_mm, self.reach.r_min_mm, self.reach.r_max_mm, self.reach.z_mm)
        return self.reason


def reach_at(z_mm: float, upper: float = UPPER_ARM_MM,
             fore: float = FOREARM_MM, tool: float = TOOL_DROP_MM,
             shoulder_z: float = SHOULDER_Z_MM) -> Reach:
    """给定吸盘末端高度，返回可水平到达的半径区间。

    腕关节高度 = 末端高度 + tool（吸盘挂在腕下），
    与大臂/小臂构成的三角关系只看腕到肩的竖直距离 dz。
    """
    if not isfinite(z_mm):
        raise ValueError('z must be finite')
    if upper <= 0 or fore <= 0:
        raise ValueError('link lengths must be positive')
    dz = (z_mm + tool) - shoulder_z
    outer = upper + fore
    inner = abs(upper - fore)

    if abs(dz) >= outer:
        return Reach(z_mm, 0.0, 0.0)          # 太高或太低，够不着
    r_max = sqrt(outer * outer - dz * dz)
    if abs(dz) >= inner:
        r_min = 0.0                            # 抬得够高，圆环中心那块也够得着
    else:
        r_min = sqrt(inner * inner - dz * dz)
    return Reach(z_mm, r_min, r_max)


def check_point(x_mm: float, y_mm: float, z_mm: float,
                margin_mm: float = 10.0,
                z_min_mm: Optional[float] = None,
                z_max_mm: Optional[float] = None,
                front_only: bool = True) -> ReachCheck:
    """判断一个抓取点是否在可达域内，留 margin 的余量。

    margin 的作用是不要贴着边界发指令：边界上的点固件可能直接拒收，
    而且真到那儿吸盘姿态也快压不下去了。
    """
    if not all(isfinite(v) for v in (x_mm, y_mm, z_mm)):
        raise ValueError('target must be finite')
    reach = reach_at(z_mm)
    radius = hypot(x_mm - SHOULDER_X_MM, y_mm)

    if z_min_mm is not None and z_mm < z_min_mm:
        return ReachCheck(False, 'below z_min (%.1f < %.1f)' % (z_mm, z_min_mm), radius, reach)
    if z_max_mm is not None and z_mm > z_max_mm:
        return ReachCheck(False, 'above z_max (%.1f > %.1f)' % (z_mm, z_max_mm), radius, reach)
    if reach.empty:
        return ReachCheck(False, 'height %.1f mm is out of arm stroke' % z_mm, radius, reach)
    if front_only and x_mm < 0:
        return ReachCheck(False, 'behind the arm base (x=%.1f)' % x_mm, radius, reach)
    if radius > reach.r_max_mm - margin_mm:
        return ReachCheck(False, 'too far: r=%.1f > %.1f (limit %.1f)'
                          % (radius, reach.r_max_mm - margin_mm, reach.r_max_mm), radius, reach)
    if radius < reach.r_min_mm + margin_mm:
        return ReachCheck(False, 'too close: r=%.1f < %.1f (limit %.1f)'
                          % (radius, reach.r_min_mm + margin_mm, reach.r_min_mm), radius, reach)
    return ReachCheck(True, 'ok', radius, reach)


def envelope_table(heights_mm: Sequence[float], points: int = 5) -> List[str]:
    """把若干高度的可达半径打成表，用于和实测点集对照。"""
    lines = ['    z(mm)   r_min    r_max   (腕高 = z + %.1f)' % TOOL_DROP_MM]
    for z in heights_mm:
        reach = reach_at(z)
        if reach.empty:
            lines.append('%9.1f       -        -   够不着' % z)
        else:
            lines.append('%9.1f %8.1f %8.1f' % (z, reach.r_min_mm, reach.r_max_mm))
    return lines


def grasp_height(support_z_mm: float, block_height_mm: float,
                 clearance_mm: float = 0.0) -> float:
    """方块顶面高度 = 支撑面高度 + 方块高度，留给吸盘的接触高度。

    clearance 为正表示压在顶面上方一点点（吸盘唇口压缩量），
    负值表示再往下压。10cm 方块就是 support + 100。
    """
    if not isfinite(support_z_mm) or not isfinite(block_height_mm):
        raise ValueError('support height and block height must be finite')
    if block_height_mm <= 0:
        raise ValueError('block height must be positive')
    return support_z_mm + block_height_mm + clearance_mm
