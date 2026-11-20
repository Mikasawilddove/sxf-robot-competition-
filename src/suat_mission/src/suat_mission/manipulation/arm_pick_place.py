#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""完整抓取流水线：开机初始位 -> 观察 -> 识别 -> 垂直抓取 -> 垂直抬升
-> 带着方块摆到最左 -> 扫桌面避让放置。

动作顺序：

1) 抓起并抬升到位后，默认执行「自动避让放置」：带着方块把关节1 转到最左，
   相机（固定在高处）拍一帧把桌面所有 100mm 方块都框出来，挑一块不压到
   它们的空地放下；方块数量达到 --crowd 个（默认 6）就改叠第二层。
   想恢复旧行为（抓起后原样带回初始位）用 --no-place；
   想放到固定坐标用 --place-at；想叠到指定名称的方块上用 --place-on。

2) 吸附等待结束后，以带载速度发送一条固定 XY 的垂直起提指令。
   随后的收臂和转运也受 --feed-loaded 限速。此入口使用估算等待时间。

3) 流程首尾都在开机初始位，跑完随时可以再跑一遍。

4) 给相机让视野时在指定高度改变方位。指令指定笛卡尔终点，
   实际关节轨迹由固件插补决定。

几何模型（cam_point = A @ arm_point + B）来自现场 8 点 DLT 拟合，
参数从场地标定文件读取，源码里不留数值：
    data/calibration/field_20261117/pickup_geometry_active.yaml
    data/calibration/field_20261117/pickup_geometry_C2_candidate.yaml
  （或 --calib 指定、环境变量 SUAT_PICKUP_GEOMETRY 指定）
相机被碰歪需要重标时，只改标定文件，不需要改这个脚本。
出厂标定文件 ~/thefile.txt 与本机相机安装不匹配，这里不使用。

用法：
    rosrun suat_mission arm_pick_place.py                  # 只检测，不动臂
    rosrun suat_mission arm_pick_place.py --execute        # 走完整流程
    rosrun suat_mission arm_pick_place.py --execute --place-at 240,-60,-132
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from swiftpro.msg import position, status
from suat_mission.manipulation.official_calibration import load_profile, project_top
from suat_mission.manipulation.arm_path import (
    FEED_APPROACH, FEED_FAST, FEED_LOADED, speed_of, travel_dwell, vertical_lift_points,
)
from suat_mission.vision.experimental_vision import ExperimentalVision, TEMPLATE_FILES, measure_layer

# --- 标定参数：全部从场地标定文件读，源码里不留 DLT 数值 --------------------
#
# 几何模型（cam_point = A @ arm_point + B）来自现场 8 点 DLT 拟合，
# 结果是 pickup_geometry_*.yaml 里的 legacy_camera_A / legacy_camera_B_mm。
# 相机被碰歪、重新标定之后只需要改标定文件，不用再动这个脚本。
#
# 文件按顺序逐个读、字段级合并（先出现的优先）：
#   1) --calib 指定的文件（可给多个）
#   2) 环境变量 SUAT_PICKUP_GEOMETRY
#   3) <工作空间>/data/calibration/field_20261117/pickup_geometry_active.yaml
#   4) <工作空间>/data/calibration/field_20261117/pickup_geometry_C2_candidate.yaml
CALIB_FILES_DEFAULT = (
    'data/calibration/field_20261117/pickup_geometry_active.yaml',
    'data/calibration/field_20261117/pickup_geometry_C2_candidate.yaml',
)
CALIB_ENV = 'SUAT_PICKUP_GEOMETRY'
PLANAR_PROFILE = 'data/calibration/official/camera_planar_20261119.yaml'

# D435 出厂内参，只在既拿不到 /camera/color/camera_info、标定文件里也没有内参时兜底
FX, FY, CX, CY = 604.3, 603.0, 330.1, 251.2

A = None   # 3x3，相机->臂 的仿射（DLT 拟合），由 load_calibration 填充
B = None   # 3，平移 mm，同上

CALIB = {
    'loaded': False,
    'source': [],          # 实际读到的文件（按优先级）
    'fields': {},          # 字段 -> 来源文件
    'intrinsics': 'builtin_default',
    'block_z': None,       # 方块顶面高度
    'grasp_offset': None,  # 吸盘唇比末端低多少
    'stack_height': None,  # 方块高度
    'arm_yaw_in_base_rad': None,
    'calibration_id': None,
    'source_status': None,
}

BRIDGE = CvBridge()


def _workspace_roots():
    """脚本可能在任意 CWD 下被调用，用脚本位置向上找几级当作工作空间根。"""
    roots = []
    cur = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        cur = os.path.dirname(cur)
        if not cur or cur in roots:
            break
        roots.append(cur)
    cwd = os.getcwd()
    if cwd not in roots:
        roots.append(cwd)
    return roots


def _read_calib_file(path):
    """.yaml 里可能是 YAML 也可能是 JSON（仓库里两种混着），先 YAML 再 JSON。"""
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    if path.endswith(('.yaml', '.yml')):
        try:
            import yaml
            data = yaml.safe_load(text)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
    return json.loads(text)


def _extract_calib(data):
    """从一份标定文件里抠出本脚本关心的字段，缺的就不放进结果里。"""
    out = {}
    a = data.get('legacy_camera_A')
    b = data.get('legacy_camera_B_mm')
    if a is not None and b is not None:
        out['A'] = np.asarray(a, dtype=float).reshape(3, 3)
        out['B'] = np.asarray(b, dtype=float).reshape(3)
    else:
        # 只存了 3x4/4x4 的情况：camera_to_arm = [inv(A) | -inv(A)@B]，反推回去
        m = data.get('camera_to_arm_mm')
        if m is not None:
            m = np.asarray(m, dtype=float)
            if m.shape in ((3, 4), (4, 4)):
                inv_a = np.linalg.inv(m[:3, :3])
                out['A'] = inv_a
                out['B'] = -inv_a @ m[:3, 3]

    for key in ('cup_offset_mm', 'support_z_mm', 'object_height_mm',
                'arm_yaw_in_base_rad', 'calibration_id', 'source_status'):
        if data.get(key) is not None:
            out[key] = data[key]

    # 内参：文件里可能写成 camera_matrix（3x3）或 fx/fy/cx/cy 四个单值
    cm = data.get('camera_matrix')
    if cm is not None:
        k = np.asarray(cm, dtype=float).reshape(3, 3)
        out['intrinsics'] = (float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]))
    elif all(data.get(k) is not None for k in ('fx', 'fy', 'cx', 'cy')):
        out['intrinsics'] = (float(data['fx']), float(data['fy']),
                             float(data['cx']), float(data['cy']))
    return out


def _intrinsics_from_topic(timeout=2.0):
    """/camera/color/camera_info 里是相机实际在用的内参，比写死的更可信。"""
    try:
        # 离线自测没有 rospy 节点时直接跳过，不要白等 timeout
        if not rospy.core.is_initialized():
            return None
        from sensor_msgs.msg import CameraInfo
        msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo,
                                    timeout=timeout)
        k = msg.K
        if k[0] > 0 and k[4] > 0:
            return (float(k[0]), float(k[4]), float(k[2]), float(k[5]))
    except Exception:
        pass
    return None


def load_calibration(files=None, use_camera_info=True, verbose=True):
    """按优先级读标定文件并写入模块级 A/B/内参/放置参数。

    files 为 None 时用 CALIB_FILES_DEFAULT + 环境变量；给空列表表示不读文件。
    只读文件和话题，不动机械臂，可以安全地在 main 里、发指令之前调用。
    """
    global A, B, FX, FY, CX, CY

    candidates = []
    for f in (files or []):
        candidates.append(f)
    env = os.environ.get(CALIB_ENV)
    if env:
        candidates.append(env)
    if files is None:
        candidates.extend(CALIB_FILES_DEFAULT)

    roots = _workspace_roots()
    merged = {}
    used_files = []
    field_src = {}
    matrix_file = None   # 提供 A/B 的那一份，calibration_id 要跟着它走
    for item in candidates:
        paths = [item] if os.path.isabs(item) else [os.path.join(r, item) for r in roots]
        path = next((p for p in paths if os.path.isfile(p)), None)
        if path is None:
            continue
        try:
            data = _read_calib_file(path)
        except Exception as exc:
            print('    标定文件解析失败，跳过 %s: %s' % (path, exc), flush=True)
            continue
        got = _extract_calib(data)
        if not got:
            continue
        used_files.append(path)
        for key, value in got.items():
            # A/B 只认第一次成功解析出来的那一份，其余字段也一样先到先得
            if key not in merged:
                merged[key] = value
                field_src[key] = os.path.basename(path)
        if matrix_file is None and 'A' in got:
            matrix_file = path
            # calibration_id / source_status 描述的是手眼矩阵本身，
            # 必须取自同一个文件，否则日志会显示成另一份标定的 id
            for key in ('calibration_id', 'source_status'):
                if got.get(key) is not None:
                    merged[key] = got[key]
                    field_src[key] = os.path.basename(path)

    if 'A' not in merged or 'B' not in merged:
        raise SystemExit(
            '找不到可用的手眼标定参数（legacy_camera_A / legacy_camera_B_mm）。\n'
            '请用 --calib 指定标定文件，或设置 %s 环境变量。\n'
            '已尝试：\n  %s' % (CALIB_ENV, '\n  '.join(candidates) or '(空)'))

    A = merged['A']
    B = merged['B']
    if not np.isfinite(A).all() or not np.isfinite(B).all():
        raise SystemExit('标定文件里的 legacy_camera_A / legacy_camera_B_mm 含非有限值')
    if abs(float(np.linalg.det(A))) < 1e-9:
        raise SystemExit('标定矩阵 A 不可逆（det≈0），文件有问题')

    # 内参优先级：camera_info 话题 > 标定文件 > 出厂兜底
    CALIB['intrinsics'] = 'builtin_default'
    fx = fy = cx = cy = None
    if use_camera_info:
        got = _intrinsics_from_topic()
        if got:
            fx, fy, cx, cy = got
            CALIB['intrinsics'] = 'camera_info'
    if fx is None and 'intrinsics' in merged:
        fx, fy, cx, cy = merged['intrinsics']
        CALIB['intrinsics'] = 'calib_file'
    if fx is not None:
        FX, FY, CX, CY = fx, fy, cx, cy

    # 方块顶面高度 = 支撑面 + 方块高；两个值都在标定文件里
    heights = merged.get('object_height_mm') or {}
    block_h = None
    if isinstance(heights, dict) and heights:
        block_h = max(float(v) for v in heights.values())
    if block_h is None:
        block_h = 100.0
    support_z = merged.get('support_z_mm')
    CALIB['block_z'] = (float(support_z) + block_h) if support_z is not None else None
    CALIB['grasp_offset'] = (float(merged['cup_offset_mm'])
                             if merged.get('cup_offset_mm') is not None else None)
    CALIB['stack_height'] = block_h
    CALIB['arm_yaw_in_base_rad'] = merged.get('arm_yaw_in_base_rad')
    CALIB['calibration_id'] = merged.get('calibration_id')
    CALIB['source_status'] = merged.get('source_status')
    CALIB['loaded'] = True
    CALIB['source'] = used_files
    CALIB['fields'] = field_src

    if verbose:
        print('=== 标定参数（来自文件，非硬编码）===', flush=True)
        for path in used_files:
            print('    来源: %s' % path, flush=True)
        if CALIB['calibration_id']:
            print('    calibration_id: %s' % CALIB['calibration_id'], flush=True)
        print('    A = %s' % np.array2string(A, precision=5, separator=', '), flush=True)
        print('    B = %s   (行列式 %.5f)'
              % (np.array2string(B, precision=3, separator=', '), float(np.linalg.det(A))),
              flush=True)
        print('    手眼矩阵来源: %s   id=%s'
              % (field_src.get('A', '?'), CALIB['calibration_id']), flush=True)
        if CALIB.get('source_status'):
            print('    该标定状态: %s' % CALIB['source_status'], flush=True)
        print('    内参 fx=%.2f fy=%.2f cx=%.2f cy=%.2f  来源=%s'
              % (FX, FY, CX, CY, CALIB['intrinsics']), flush=True)
        if CALIB['block_z'] is not None:
            print('    方块顶面 z = %.1f mm (支撑面 %.1f + 方块高 %.1f)'
                  % (CALIB['block_z'], support_z, block_h), flush=True)
        if CALIB['grasp_offset'] is not None:
            print('    吸盘偏移 = %.1f mm' % CALIB['grasp_offset'], flush=True)
        if CALIB['arm_yaw_in_base_rad'] is not None:
            print('    臂在车体上的偏航 = %.2f°'
                  % math.degrees(CALIB['arm_yaw_in_base_rad']), flush=True)
        # 仿射不是刚体变换，这一点在接入自主链路时必须保留标记
        if CALIB['calibration_id'] and 'dlt' in str(CALIB['calibration_id']).lower():
            print('    注意: 这是 DLT 仿射拟合（det=%.4f，非刚体），'
                  '不是刚性手眼变换' % float(np.linalg.det(A)), flush=True)
    return CALIB

# --- 速度 ---
# 固件不认名义进给率：F10000 标称 167mm/s，实机只有 44mm/s。
# 实测把 F 提到 80000 能跑到 95mm/s，再往上（150000）反而回落到 80，
# 说明 80000 就是固件的饱和点。
FEED_SLOW = FEED_APPROACH

# 转关节时末端走的是弧线，实测比直线快：r=200 转 90°（弧长 314mm）
# 正向 2.46s、反向 2.76s，等效 114~128mm/s。用直线那套 95mm/s 估会多等一秒多。
SPEED_SWING = 115.0

# 等待时间的算法：行程 / 速度 + 余量。
# 旧写法是写死 3.2~3.6s，实测 90mm 只要 0.94s，每个动作白等两秒多。
DWELL_PAD = 0.55      # 起跑延迟 + 停稳 + 速度切换的余量
DWELL_MIN = 0.45      # 再短的动作也要给这一点
DWELL_STEP = 1.2      # 分段走时每段的等待（兼容旧调用）
DWELL_LONG = 1.6


# 在观察位和方块正上方之间水平移动时保持的高度，必须高于方块顶面
SAFE_Z = 70.0


def parse_xyz(text):
    parts = [float(v) for v in str(text).replace(' ', '').split(',')]
    if len(parts) != 3:
        raise ValueError('需要 x,y,z 三个数，收到 %r' % text)
    return tuple(parts)


def parse_xy(text):
    """只给两个数（x,y）。给探测脚本挑点时用。"""
    parts = [float(v) for v in str(text).replace(' ', '').split(',')]
    if len(parts) != 2:
        raise ValueError('需要 x,y 两个数，收到 %r' % text)
    return tuple(parts)


def parse_place(text, default_z):
    """放置点允许只给 x,y。

    只给两个数时，z 自动取「方块顶面高度 + 吸盘偏移」，也就是吸盘唇刚好
    落在放置面上的高度。手填 z 很容易填成方块顶面高度，那样会多压 45mm。
    """
    parts = [float(v) for v in str(text).replace(' ', '').split(',')]
    if len(parts) == 2:
        return parts[0], parts[1], default_z
    if len(parts) == 3:
        return parts[0], parts[1], parts[2]
    raise ValueError('放置点需要 x,y 或 x,y,z，收到 %r' % text)


def pixel_to_arm(u, v, depth_mm, block_z):
    """已知方块顶面高度时，单个像素可以唯一反投影到臂坐标。"""
    if A is None:
        # 离线自测/被别的脚本 import 时也要能用，按默认优先级惰性加载一次
        load_calibration(verbose=False)
    cam = np.array([(u - CX) * depth_mm / FX,
                    (v - CY) * depth_mm / FY,
                    depth_mm])
    rhs = (cam - B - A[:, 2] * block_z)[:2]
    xy = np.linalg.solve(A[:2, :2], rhs)
    return float(xy[0]), float(xy[1])


def ground_baseline(depth):
    """逐行估计「地面在该行应有的深度」，作为判断凸起物的基线。

    地面是平的，但相机有俯仰角，所以同一块地面在图像上越靠上越远——
    这是透视，不是斜面。把每行的深度中位数连起来就得到这条基线：
    方块只占每行少数像素，动不了中位数。

    关键点：这条基线是**每帧现算**的，不是标定值。机器人开到任何位置，
    下一帧的深度就给出那个位置的地面，不需要人工重新标任何东西。
    """
    rows = depth.shape[0]
    base = np.full(rows, np.nan)
    for r in range(rows):
        line = depth[r]
        good = line[line > 0]
        if good.size < 60:
            continue
        base[r] = np.median(good)
    ok = ~np.isnan(base)
    if ok.sum() < 40:
        return None
    idx = np.arange(rows)
    return np.polyfit(idx[ok], base[ok], 2)


def rise_map(depth):
    """Estimate candidate support plane in 3D and signed heights in mm.

    A dominant plane is a candidate, not a guarantee that it is the physical
    support surface. The action lock remains until geometry is verified.
    """
    if depth is None or not isinstance(depth, np.ndarray) or depth.ndim != 2:
        return None
    if not all(np.isfinite(v) for v in (FX, FY, CX, CY)) or FX <= 0 or FY <= 0:
        return None
    height, width = depth.shape
    if height < 40 or width < 40:
        return None
    stride = max(2, min(height, width) // 120)
    vv, uu = np.mgrid[0:height:stride, 0:width:stride]
    zz = depth[::stride, ::stride].astype(np.float64)
    valid = np.isfinite(zz) & (zz > 300.0) & (zz < 1500.0)
    u, v, z = uu[valid], vv[valid], zz[valid]
    if z.size < 500:
        print('[plane] insufficient valid depth', flush=True)
        return None
    px, py, pz = (u-CX)*z/FX, (v-CY)*z/FY, z
    points = np.column_stack((px, py, pz))
    rng = np.random.RandomState(17)
    best_count, best_normal, best_d = 0, None, None
    for _ in range(220):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        mag = np.linalg.norm(normal)
        if mag < 1e-8:
            continue
        normal /= mag
        offset = -float(np.dot(normal, a))
        if offset < 0:
            normal, offset = -normal, -offset
        residual = np.abs(normal[0]*px + normal[1]*py + normal[2]*pz + offset)
        count = int(np.count_nonzero(residual <= 6.0))
        if count > best_count:
            best_count, best_normal, best_d = count, normal, offset
    if best_normal is None or best_count < max(400, int(0.30 * len(points))):
        print('[plane] no sufficiently supported plane', flush=True)
        return None
    residual = np.abs(best_normal[0]*px + best_normal[1]*py + best_normal[2]*pz + best_d)
    keep = residual <= 6.0
    # Insist that the plane appears over a broad part of the image.
    if np.ptp(u[keep]) < width*0.4 or np.ptp(v[keep]) < height*0.3:
        print('[plane] reference surface lacks spatial coverage', flush=True)
        return None
    support = points[keep]
    center = support.mean(axis=0)
    covariance = (support-center).T @ (support-center) / len(support)
    _, eigvec = np.linalg.eigh(covariance)
    normal = eigvec[:,0]
    offset = -float(np.dot(normal,center))
    if offset < 0:
        normal, offset = -normal, -offset
    yy, xx = np.indices(depth.shape, dtype=np.float32)
    dd = depth.astype(np.float32)
    result = dd * (normal[0]*(xx-CX)/FX + normal[1]*(yy-CY)/FY + normal[2]) + offset
    result[~np.isfinite(dd) | (dd <= 0)] = np.nan
    print('[plane] candidate support %.1f%%, normal %s' %
          (100.0 * best_count / len(points), np.array2string(normal, precision=3)), flush=True)
    return result.astype(np.float32)


def find_block(colour, depth, min_area=2500):
    """按「比地面凸出多少」分割方块，再用立方体尺寸做几何校验。

    判据只有方块自身的物理属性，不含任何与场地、相机位置有关的常数：

      * 地面基线：逐行取深度中位数（自动）
      * 凸起量：该行地面深度 - 该像素深度。地面≈0，10cm 方块≈100mm
      * 形状：面积够大、填充率高、长宽比接近 1（立方体正视/俯视都是方块）

    机械臂自己离相机近得多，凸起量在 200mm 以上，会自然被上限排除。
    """
    rise = rise_map(depth)
    if rise is None:
        return None, np.zeros_like(depth, np.uint8)
    mask = ((rise > 35) & (rise < 150) & (depth > 0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for blob in contours:
        area = cv2.contourArea(blob)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(blob)
        if w <= 0 or h <= 0:
            continue
        fill = area / float(w * h)
        # 俯视斜看时，方块只有顶面凸起满 100mm，竖直侧面是渐变，
        # 掩膜天然缺角，填充率会掉到 0.5 上下。0.55 这种硬阈值会把
        # 真方块筛掉（实测 0.54~0.56 抖动），放到 0.45 才稳。
        if fill < 0.45 or not 0.5 < w / float(h) < 2.0:
            continue
        moments = cv2.moments(blob)
        candidate = {'u': moments['m10'] / moments['m00'],
                     'v': moments['m01'] / moments['m00'],
                     'area': area, 'box': (x, y, w, h),
                     'fill': fill, 'rise': float(np.median(rise[int(moments['m01'] / moments['m00']) - 3:int(moments['m01'] / moments['m00']) + 3,
                                                                 int(moments['m10'] / moments['m00']) - 3:int(moments['m10'] / moments['m00']) + 3]))}
        if best is None or area > best['area']:
            best = candidate

    if best is None:
        return None, mask

    # 二义性：方块的顶面在这个视角下只是最上面一条窄带，而质心会落在
    # 竖直正面上，比真实顶面中心低约半块。取框顶一小条求质心才是顶面。
    x, y, w, h = best['box']
    strip_h = max(4, int(round(0.18 * h)))
    strip = mask[y:y + strip_h, x:x + w]
    ys, xs = np.where(strip > 0)
    if xs.size >= 40:
        best['u'] = x + xs.mean()
        best['v'] = y + ys.mean()
        best['top_face'] = True
    patch = depth[y:y + h, x:x + w]
    sel = patch[(patch > 0)]
    if sel.size:
        best['depth'] = int(np.median(sel))
    else:
        best['depth'] = int(depth[int(best['v']), int(best['u'])])
    return best, mask


_SMALL_TEMPLATES = None


def small_templates(size=96):
    """把四类模板缩到统一尺寸，用于「按框分类」。

    BRISK 那套是全图提特征再找单应，尺度差 2 倍、透视一大就匹配不上，
    还会把两个白方块互相认错。这里改成：位置由深度几何定（可靠），
    只让模板回答“这个框里是哪个图案”，判据是归一化相关。
    """
    global _SMALL_TEMPLATES
    if _SMALL_TEMPLATES is not None:
        return _SMALL_TEMPLATES
    import rospkg
    base = os.path.join(rospkg.RosPack().get_path('suat_mission'),
                        'assets', 'templates')
    out = {}
    for label, files in TEMPLATE_FILES.items():
        ims = []
        for name in files:
            image = cv2.imread(os.path.join(base, name))
            if image is None or not image.size:
                continue
            ims.append(cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA))
        out[label] = ims
    _SMALL_TEMPLATES = (size, out)
    return _SMALL_TEMPLATES


def classify_roi(colour, box, top_ratio=0.70):
    """判断一个框里是哪个图案，返回 [(label, score), ...]，按分数降序。

    斜俯视时框的下半部分是竖直侧面，图案是拉伸变形的，比对上反而拉低分数，
    所以只取框上部 top_ratio 那一段（顶面所在区域）。
    """
    size, tpls = small_templates()
    x, y, w, h = box
    x0, y0 = max(0, int(x)), max(0, int(y))
    x1 = min(colour.shape[1], int(x + w))
    y1 = min(colour.shape[0], int(y + h * top_ratio))
    if x1 - x0 < 12 or y1 - y0 < 12:
        return []
    roi = cv2.resize(colour[y0:y1, x0:x1], (size, size),
                     interpolation=cv2.INTER_AREA)
    roi_g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    out = []
    for label, ims in tpls.items():
        best = -1.0
        for im in ims:
            g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            r = cv2.matchTemplate(roi_g, g, cv2.TM_CCOEFF_NORMED)
            best = max(best, float(r.max()))
        out.append((label, best))
    out.sort(key=lambda t: -t[1])
    return out


def find_all_blocks(colour, depth, min_area=2000):
    """用深度凸起把画面里所有方块都找出来（不只挑一个），供分类用。

    判据和 find_block 完全一致：逐行地面基线 + 凸起量 35~150mm + 形状过滤。
    """
    rise = rise_map(depth)
    if rise is None:
        return [], np.zeros_like(depth, np.uint8)
    mask = ((rise > 35) & (rise < 150) & (depth > 0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for blob in contours:
        area = cv2.contourArea(blob)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(blob)
        if w <= 0 or h <= 0:
            continue
        fill = area / float(w * h)
        if fill < 0.45 or not 0.5 < w / float(h) < 2.0:
            continue
        out.append({'box': (x, y, w, h), 'area': area, 'fill': fill})
    out.sort(key=lambda b: -b['area'])
    return out, mask

_VISION = None


def named_vision():
    """加载 assets/templates 下的模板，构建包里那套 BRISK 检测器（只建一次）。

    与 launch/field_vision.launch 用的是同一套模板和同一组阈值：
    teddybear / clock / cake 走 BRISK 特征点 + 单应性找四边形，
    banana 走黄色轮廓 + 形状匹配。
    """
    global _VISION
    if _VISION is not None:
        return _VISION
    import rospkg

    base = os.path.join(rospkg.RosPack().get_path('suat_mission'),
                        'assets', 'templates')
    _VISION = ExperimentalVision.from_template_directory(base)
    return _VISION


def _top_face_uv(box, depth):
    """在检测框顶部取一条窄带，返回顶面中心的像素坐标。

    斜俯视时框的下半部分是方块侧面，直接用框中心会落到正面上，
    实测偏小半块，所以只取框顶 18% 那一条，并用深度中位数剔除背景。
    """
    x, y, w, h = box
    x0 = max(0, int(x))
    y0 = max(0, int(y))
    x1 = min(depth.shape[1], x0 + int(w))
    y1 = min(depth.shape[0], y0 + int(h))
    if x1 - x0 < 3 or y1 - y0 < 3:
        return None
    strip_h = max(4, int(round(0.18 * (y1 - y0))))
    patch = depth[y0:y0 + strip_h, x0:x1].astype(np.float32)
    good = patch[patch > 0]
    if good.size < 20:
        return None
    med = float(np.median(good))
    ys, xs = np.where((patch > 0) & (np.abs(patch - med) < 35))
    if xs.size < 20:
        return None
    return x0 + float(xs.mean()), y0 + float(ys.mean())


def find_named_block(colour, depth, target, min_score=0.45, min_hits=1):
    """识别画面里所有方块，挑出名字等于 target 的那一个。

    返回 (block, mask, found)：
      block 为 None 表示画面里没有叫这个名字的方块
      found 是本次识别到的全部方块，用来打印清单和画框图

    min_hits 是「目标至少要找到几个」。要叠同名方块时必须给 2 ——
    只找到一个的话，抓起来桌上就没得放了。数量不够会自动走增强重识别。
    """
    found = named_vision().detect_for_target(colour, target, min_score, min_hits)

    mask = np.zeros(depth.shape, np.uint8)
    for d in found:
        bx = int(round(d.x - d.width / 2))
        by = int(round(d.y - d.height / 2))
        cv2.rectangle(mask, (bx, by),
                      (int(round(bx + d.width)), int(round(by + d.height))), 255, -1)

    hits = [d for d in found if d.label == target]
    if not hits:
        return None, mask, found
    best = max(hits, key=lambda d: d.score)

    box = (int(round(best.x - best.width / 2)), int(round(best.y - best.height / 2)),
           int(round(best.width)), int(round(best.height)))
    uv = _top_face_uv(box, depth)
    if uv is None:
        # 深度换算不出来时退化到框上部 32% 处，至少方向不会偏到侧面
        uv = (best.x, best.y - best.height * 0.32)
    u, v = uv

    x, y, w, h = box
    patch = depth[max(0, y):max(0, y) + max(1, h), max(0, x):max(0, x) + max(1, w)]
    sel = patch[patch > 0]
    depth_mm = int(np.median(sel)) if sel.size else int(depth[int(v), int(u)])

    block = {'u': float(u), 'v': float(v),
             'area': float(best.width * best.height), 'box': box,
             'fill': 1.0, 'rise': 0.0, 'depth': depth_mm,
             'top_face': True, 'label': best.label, 'score': float(best.score)}
    return block, mask, found


class Arm(object):
    """所有运动都走这里，保证「垂直」就是真的垂直。"""

    def __init__(self, settle=1.5, loaded_feed=FEED_LOADED):
        self.pos = rospy.Publisher('/position_write_topic', position, queue_size=1)
        self.st = rospy.Publisher('/swiftpro_status_topic', status, queue_size=1)
        self.pump = rospy.Publisher('/pump_topic', status, queue_size=1)
        time.sleep(settle)
        self.cur = None
        self.loaded_feed = loaded_feed
        # 控制状态，仅用于限速。开泵不代表已抓住，关泵不代表已脱离。
        self.vacuum_requested = False

    def attach(self):
        # M17 上力。只在流程开始时发一次：每步都重发会让运动执行错乱
        # （同一点连发四次会得到四个不同位置）。
        for _ in range(3):
            self.st.publish(status(1))
            time.sleep(0.2)
        time.sleep(0.8)

    def set_feed(self, feed):
        """切换骨架速度。驱动每次回调都实时读这个参数，下一条指令立即生效。

        写参数是同步 RPC，成功后才发布位置指令。
        """
        feed = float(feed)
        # 驱动在收到位置消息时读这个参数。写失败就停止，避免沿用上一档。
        rospy.set_param('/swiftpro/feedrate', feed)

    def _dwell_for(self, dist, feed):
        """按实测速度模型算等待时间，代替写死的 3.2s。

        宁可稍长也不要短：等短了会在机械臂还在动的时候就发下一条，
        抓取时会把方块刮歪。
        """
        return travel_dwell(dist, feed)

    def goto(self, x, y, z, dwell=None, tag='', feed=None):
        if feed is None:
            feed = FEED_FAST
        if self.vacuum_requested:
            feed = min(feed, self.loaded_feed)
        x, y, z = float(x), float(y), float(z)
        if self.cur is not None:
            dist = math.sqrt((x - self.cur[0]) ** 2 + (y - self.cur[1]) ** 2
                             + (z - self.cur[2]) ** 2)
        else:
            dist = 120.0          # 不知道当前位置时按中等行程估
        estimated = self._dwell_for(dist, feed)
        if dwell is None:
            dwell = estimated
        elif self.vacuum_requested:
            dwell = max(dwell, estimated)
        print('[机械臂] 当前步骤：%s' % (tag or '移动'), flush=True)
        print('[机械臂] 目标位置：(%.1f, %.1f, %.1f) mm  F%.0f' % (x, y, z, feed), flush=True)
        print('[机械臂] 目标姿态：当前接口仅控制 XYZ；参考坐标系：arm_base；坐标单位：毫米', flush=True)
        self.set_feed(feed)
        self.pos.publish(position(x, y, z))
        time.sleep(dwell)
        self.cur = (x, y, z)
        print('[机械臂] 执行结果：指令已发送并等待 %.2f s；此旧入口无独立到位反馈，无法确认真实到达' % dwell,
              flush=True)
        print('    arm -> (%7.1f, %7.1f, %7.1f)  %5.0fmm/%.2fs%s'
              % (x, y, z, dist, dwell, ('  ' + tag) if tag else ''), flush=True)

    def descend(self, x, y, z_from, z_to, step=0.0, dwell=None, feed=None):
        """原地下降：x、y 全程不变。

        step <= 0 时一条 G0 直达目标（默认）；给了正数才分段走。
        G0 本身就是笛卡尔指令，一次到位比拆成几段更准，
        分段反而每段都要重新起停、累积误差。
        """
        if not (math.isfinite(z_to) and math.isfinite(z_from) and z_to <= z_from):
            raise ValueError('下降终点必须低于或等于起点')
        if z_from == z_to:
            return
        if feed is None:
            feed = FEED_SLOW
        if step <= 0:
            self.goto(x, y, z_to, dwell=dwell, feed=feed)
            return
        z = z_from
        while z > z_to:
            z = max(z_to, z - step)
            self.goto(x, y, z, feed=feed)

    def ascend(self, x, y, z_from, z_to, step=0.0, dwell=None, feed=None):
        """原地抬升：x、y 全程不变，只有 z 在动。step <= 0 时一条指令直达。"""
        if feed is None:
            feed = FEED_SLOW
        for pose in vertical_lift_points((x, y, z_from), z_to, step):
            self.goto(*pose, dwell=dwell if step <= 0 else None, feed=feed)

    def swing(self, radius, z, deg_from, deg_to, step_deg=10.0, dwell=None,
              feed=None):
        """在较高的指令高度改变方位，再在目标 XY 降低。固件决定实际插补轨迹。"""
        if feed is None:
            feed = FEED_FAST
        end_x = radius * math.cos(math.radians(deg_to))
        end_y = radius * math.sin(math.radians(deg_to))
        if dwell is None:
            arc = radius * abs(math.radians(deg_to - deg_from))
            swing_speed = SPEED_SWING
            if self.vacuum_requested:
                swing_speed = min(swing_speed, speed_of(min(feed, self.loaded_feed)))
            dwell = max(DWELL_MIN, arc / swing_speed + DWELL_PAD)
        if self.cur is None:
            raise ValueError('改变方位前需要已知起始指令位置')
        cx, cy, cz = self.cur
        transit_z = max(cz, z)
        if cz < transit_z:
            self.ascend(cx, cy, cz, transit_z)
        self.goto(end_x, end_y, transit_z, dwell=dwell, feed=feed,
                  tag='改变方位 %+.0f° -> %+.0f°' % (deg_from, deg_to))
        if z < transit_z:
            self.descend(end_x, end_y, transit_z, z)

    def pump_on(self, hold=2.5):
        print('[吸泵] 发送指令：开启', flush=True)
        self.pump.publish(status(1))
        self.vacuum_requested = True
        print('[吸泵] 接口返回：ROS 发布完成', flush=True)
        print('[吸泵] 吸附反馈：无压力/持物反馈，无法据此确认已吸附', flush=True)
        time.sleep(hold)

    def pump_off(self, hold):
        print('[吸泵] 发送指令：关闭', flush=True)
        self.pump.publish(status(0))
        print('[吸泵] 接口返回：ROS 发布完成；等待泄真空 %.1f s' % hold, flush=True)
        print('[吸泵] 释放反馈：无独立反馈，无法据此确认物块已释放', flush=True)
        time.sleep(hold)
        self.vacuum_requested = False


def snapshot(tag):
    msg=rospy.wait_for_message('/camera/color/image_raw', Image, timeout=5)
    image = BRIDGE.imgmsg_to_cv2(msg, 'bgr8')
    print('[相机] 图像话题：/camera/color/image_raw；分辨率：%d × %d；图像状态：正常；最后一帧时间：%.6f s（ROS 时间）' %
          (msg.width,msg.height,msg.header.stamp.to_sec()),flush=True)
    path = '/tmp/pp_%s.jpg' % tag
    cv2.imwrite(path, image)
    return image, path


def wait_still(max_wait=3.0, thr=0.25, need=2):
    """等画面真正静止再识别。

    相机装在末端，机械臂刚停下来的那一两百毫秒还在余振，这时候拍下来的帧是糊的。
    提速之后余振更明显 —— 固定 sleep 0.5s 会拍到模糊帧，BRISK 提不出特征，
    四个方块只认出一个。改成看帧间差：稳了就走，没稳就继续等，最长 3 秒。
    """
    t0 = time.time()
    prev = None
    stable = 0
    while time.time() - t0 < max_wait:
        try:
            frame = BRIDGE.imgmsg_to_cv2(
                rospy.wait_for_message('/camera/color/image_raw', Image, timeout=1.5),
                'mono8')
        except Exception:
            break
        small = cv2.resize(frame, (320, 240)).astype(np.int16)
        if prev is not None:
            pct = float((np.abs(small - prev) > 12).mean() * 100.0)
            if pct < thr:
                stable += 1
                if stable >= need:
                    print('    画面已静止（%.2fs）' % (time.time() - t0), flush=True)
                    return True
            else:
                stable = 0
        prev = small
    print('    等待画面静止超时，仍用当前帧', flush=True)
    return False


def do_place(arm, px, py, pz, lift_z, step):
    """把已经吸在吸盘上的方块放到 (px, py, pz)。

    顺序不能改：先降到位、贴稳，再关泵，等真空泄完，最后才允许动。
    反过来做（先关泵再下降）方块会在半空脱落，落地位置偏一大截。
    """
    if not (math.isfinite(pz) and math.isfinite(lift_z) and pz < lift_z):
        raise ValueError('放置高度必须低于转运高度；未发送放置动作')
    arm.goto(px, py, lift_z, tag='放置点上方（保持抬升高度）')
    arm.descend(px, py, lift_z, pz, step=step)
    time.sleep(1.0)
    arm.pump_off(hold=3.0)
    # 这一步保留：关泵后方块虽已落地，但吸盘唇还压在顶面上，
    # 直接抬 180mm 会用唇边刮着顶面滑过去，把方块带歪。
    # 先原地抬 25mm 确认脱离，再抬到巡航高度。
    clear_z = min(pz + 25.0, lift_z)
    arm.ascend(px, py, pz, clear_z)
    arm.ascend(px, py, clear_z, lift_z, feed=FEED_FAST)


# ============================ 自动放置选点 ============================
# 放置不再写死坐标，改成「带着方块摆到最左边看一眼桌面，绕开已有的方块放」。
# 方块都是 100mm 立方体。避让用的是「每个方块在图像里框出来的轮廓」
# 反投影到臂平面后的多边形 —— 不用外接圆，也不假设方块朝向：
# 桌上方块是随手摆的，朝向各不相同，按圆形算必须留到 141mm 才安全，
# 那样 100mm 间距的方块阵里根本插不进空位，实际没必要这么保守。
BLOCK_SIDE = 100.0
BLOCK_HALF_DIAG = BLOCK_SIDE * math.sqrt(2.0) / 2.0      # ≈ 70.7mm，仅作兜底


def find_raised_blocks(depth, layer_rise=100.0, min_area=2000):
    """把画面里所有「比地面高出来的东西」都框出来，叠到第二层的也算。

    与 find_all_blocks 的唯一区别是凸起量上限：那边卡在 150mm（只认桌面层），
    这里放到 3.2 层。放置选点必须看得见已经叠起来的那摞，
    否则会把新方块放到别人头顶上。
    """
    rise = rise_map(depth)
    if rise is None:
        raise ValueError('Cannot plan placement: support plane is unknown')
    hi = max(160.0, 3.2 * layer_rise)
    mask = ((rise > 35) & (rise < hi) & (depth > 0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for blob in contours:
        area = cv2.contourArea(blob)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(blob)
        if w <= 0 or h <= 0:
            continue
        fill = area / float(w * h)
        if fill < 0.40 or not 0.45 < w / float(h) < 2.2:
            continue
        out.append({'box': (x, y, w, h), 'area': area, 'fill': fill,
                    'contour': blob})
    out.sort(key=lambda b: -b['area'])
    return out, rise


def footprint_poly(contour, depth, top_z, depth_mm):
    """把图像里的方块轮廓反投影到臂坐标平面，得到落在地上的占位多边形。

    轮廓是「顶面 + 正面」的可见剪影，比真实底面略大一圈（正面被拍进来了），
    拿它当占位是偏保守的方向，正合适。
    用最小外接矩形取四个角，避免十个点的锯齿边把多边形撑得不规则。
    """
    rect = cv2.minAreaRect(contour)
    pts = []
    for u, v in cv2.boxPoints(rect):
        ax, ay = pixel_to_arm(float(u), float(v), depth_mm, top_z)
        pts.append((ax, ay))
    return np.array(pts, np.float32)


def scan_table(depth, block_z, stack_h, layer_rise,
               avoid_xy=None, avoid_r=160.0, r_lo=80.0, r_hi=355.0,
               min_area=2000, depth_lo=380.0):
    """扫一遍桌面：每个方块都框出来，换算成臂坐标、占位多边形和层数。

    avoid_xy 是「吸盘上挂着的那块自己」的臂坐标：它跟着手走，不是桌面障碍，
    必须剔掉，否则会把自己当成已占位置，落点被推到八丈远。
    depth_lo 用来剔掉贴到镜头上的东西（机械臂本体、大臂），桌上的方块
    实测都在 500mm 以外，比这近的一定不是要避让的方块。
    """
    blobs, rise = find_raised_blocks(depth, layer_rise, min_area)
    out = []
    for b in blobs:
        x, y, w, h = b['box']
        uv = _top_face_uv(b['box'], depth) or (x + w / 2.0, y + h * 0.10)
        patch = depth[max(0, y):y + h, max(0, x):x + w]
        sel = patch[patch > 0]
        if not sel.size:
            continue
        dmm = int(np.median(sel))
        if dmm < depth_lo:
            continue
        layer, _rise = measure_layer(rise, b['box'], layer_rise, stack_h)
        if layer is None:
            raise ValueError('Cannot plan placement: obstacle layer is unknown')
        top_z = block_z + layer * stack_h
        ax, ay = pixel_to_arm(uv[0], uv[1], dmm, top_z)
        r = math.hypot(ax, ay)
        if not r_lo < r < r_hi:
            continue
        if avoid_xy is not None and math.hypot(ax - avoid_xy[0],
                                               ay - avoid_xy[1]) < avoid_r:
            continue
        out.append({'box': b['box'], 'area': b['area'], 'uv': uv, 'depth': dmm,
                    'layer': layer, 'top_z': top_z, 'x': ax, 'y': ay, 'r': r,
                    'poly': footprint_poly(b['contour'], depth, top_z, dmm)})
    out.sort(key=lambda d: -d['area'])
    return out


def free_spot(blocks, gap=20.0, r_min=140.0, r_max=330.0,
              x_min=100.0, x_max=332.0):
    """在已有方块之间找一块能落脚的空地，返回 (x, y, 到最近方块的距离) 或 None。

    判据是「待放方块那块 100mm 正方形（四周各留 gap/2 间隙）」和
    每个已有方块的占位多边形不相交 —— 直接用多边形求交，朝向自然就对上了。
    候选区只取「已有方块的外接框再往外扩 1.5 个方块」的范围：
    桌子边界没有标定过，靠方块圈出来的这一片一定在桌面上，
    再往外找就可能挑到桌子外面去。
    可行候选里挑离方块堆中心最近的，方块会紧凑地挨着长，不会越摆越靠边。
    """
    if not blocks:
        return None
    grow = BLOCK_SIDE / 2.0 + max(0.0, gap) / 2.0
    polys = [b['poly'] for b in blocks if b.get('poly') is not None]
    if not polys:
        # 没有多边形（老数据/退化情况）就退回外接圆
        polys = [np.array([(b['x'] - BLOCK_HALF_DIAG, b['y'] - BLOCK_HALF_DIAG),
                           (b['x'] + BLOCK_HALF_DIAG, b['y'] - BLOCK_HALF_DIAG),
                           (b['x'] + BLOCK_HALF_DIAG, b['y'] + BLOCK_HALF_DIAG),
                           (b['x'] - BLOCK_HALF_DIAG, b['y'] + BLOCK_HALF_DIAG)],
                          np.float32) for b in blocks]
    cx = sum(b['x'] for b in blocks) / float(len(blocks))
    cy = sum(b['y'] for b in blocks) / float(len(blocks))
    xs = [b['x'] for b in blocks]
    ys = [b['y'] for b in blocks]
    pad = BLOCK_SIDE * 1.5
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad

    def evaluate(px, py):
        quad = np.array([[px - grow, py - grow], [px + grow, py - grow],
                         [px + grow, py + grow], [px - grow, py + grow]],
                        np.float32)
        clear = 1e9
        for poly in polys:
            area, _ = cv2.intersectConvexConvex(poly, quad)
            if area > 1.0:
                return None                     # 和某个方块叠上了
            d = cv2.pointPolygonTest(poly, (float(px), float(py)), True)
            clear = min(clear, abs(float(d)))
        return clear

    best = None
    for step in (20.0, 5.0):
        local = None
        px = x0
        while px <= x1 + 1e-6:
            py = y0
            while py <= y1 + 1e-6:
                if x_min < px < x_max and r_min < math.hypot(px, py) < r_max:
                    clear = evaluate(px, py)
                    if clear is not None:
                        score = math.hypot(px - cx, py - cy)
                        if local is None or score < local[0]:
                            local = (score, px, py, clear)
                py += step
            px += step
        if local is None:
            break
        best = local
        # 第二轮只在最优点 ±step 内细扫，把 20mm 网格的量化误差磨掉
        x0, x1 = best[1] - step, best[1] + step
        y0, y1 = best[2] - step, best[2] + step
    return None if best is None else (best[1], best[2], best[3])


def stack_target(blocks, sep=80.0):
    """挑一个可以往上叠的方块：自己还在桌面层、顶面没被别的方块压着。"""
    if not blocks:
        return None
    flat = [b for b in blocks if b['layer'] == 0] or blocks
    cands = []
    for b in flat:
        covered = False
        for o in blocks:
            if o is b:
                continue
            if (o['top_z'] > b['top_z'] + 30.0
                    and math.hypot(o['x'] - b['x'], o['y'] - b['y']) < sep):
                covered = True
                break
        if not covered:
            cands.append(b)
    if not cands:
        return None
    # 框得大的（面积大、识别更靠得住）、半径小的（够得着）优先
    return max(cands, key=lambda b: (b['area'], -b['r']))


def arm_to_pixel(x, y, z):
    """臂坐标反投回像素，用来把选好的落点画到扫描图上肉眼复核。

    必须和 pixel_to_arm 严格互逆：那边只用 A 的前两行解 x、y，深度是用
    实测深度直接给的，第 3 行不参与。所以这里反过来先由第 3 行算出该点的
    光学深度 cam_z，再用前两行得到 u、v —— 直接拿整条 A·p+B 当相机坐标去
    除第三分量会和 pixel_to_arm 差几十像素（标定矩阵不是正交的）。
    """
    p = np.array([float(x), float(y), float(z)])
    if A is None:
        load_calibration(verbose=False)
    cam_z = float(A[2].dot(p) + B[2])
    if cam_z <= 1.0:
        return None
    cam_xy = A[:2].dot(p) + B[:2]
    return (FX * cam_xy[0] / cam_z + CX, FY * cam_xy[1] / cam_z + CY)


def draw_scan(colour, blocks, spot=None, surface_z=None, stack=None,
              path='/tmp/pp_place_scan.jpg'):
    """把框出来的所有方块、被叠的那块、以及选中的落点画到图上。"""
    img = colour.copy()
    for i, b in enumerate(blocks):
        x, y, w, h = b['box']
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)),
                      (0, 255, 0), 2)
        cv2.putText(img, '#%d L%d' % (i, b['layer']),
                    (int(x), max(13, int(y) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    if stack is not None:
        x, y, w, h = stack['box']
        cv2.rectangle(img, (int(x) - 4, int(y) - 4),
                      (int(x + w) + 4, int(y + h) + 4), (0, 0, 255), 2)
    if spot is not None and surface_z is not None:
        half = BLOCK_SIDE / 2.0
        pts = [arm_to_pixel(spot[0] + dx, spot[1] + dy, surface_z)
               for dx, dy in ((-half, -half), (half, -half),
                              (half, half), (-half, half))]
        if all(p is not None for p in pts):
            poly = np.array([[int(round(p[0])), int(round(p[1]))] for p in pts],
                            np.int32)
            cv2.polylines(img, [poly], True, (0, 200, 255), 2)
            mx = int(round(sum(p[0] for p in pts) / 4.0))
            my = int(round(sum(p[1] for p in pts) / 4.0))
            cv2.drawMarker(img, (mx, my), (0, 200, 255), cv2.MARKER_CROSS, 18, 2)
    cv2.imwrite(path, img)
    return path


def auto_place(arm, args, home, obs, block_z, stack_h, layer_rise):
    """拿起方块后的自动放置：摆到最左边 -> 扫桌面 -> 避让选点 -> 放下。

    规则（现场定的）：
      1) 先带着方块把关节1 转到最左（观察位角度），把臂从视野里挪开；
      2) 把画面里所有 100mm 方块都框出来，换算成臂坐标；
      3) 方块少于 --crowd 个（默认 6）：找一块不压到它们的空地放下去；
      4) 方块多于等于 --crowd 个：改叠第二层，挑一个顶层还空着的方块挪上去。

    相机是固定在高处的，标定 A/B 跟臂姿态无关，所以拍照就在抬升高度上拍，
    不用再降回观察高度（降下去反而会让吸盘上那块离桌面方块只剩 10mm）。
    摆到最左边只是为了别让自己的臂和方块挡住镜头。
    """
    obs_r, obs_deg = 200.0, 90.0
    if obs is not None:
        obs_r = math.hypot(obs[0], obs[1]) or 200.0
        obs_deg = math.degrees(math.atan2(obs[1], obs[0]))
    ground_z = block_z + args.grasp_offset
    lift_z = args.lift_z

    print('=== 8. 带着方块摆到最左边（关节1 %+.0f°），看桌面还剩哪能放 ==='
          % obs_deg, flush=True)
    cur = arm.cur if arm.cur is not None else (home[0], home[1], lift_z)
    cur_r = math.hypot(cur[0], cur[1])
    cur_deg = math.degrees(math.atan2(cur[1], cur[0]))
    if abs(cur_r - obs_r) > 20.0:
        # 先水平挪到观察位半径（此时方块底在 z-52，离桌面方块顶面还有 80mm），
        # 再只转关节1，臂不做大姿态变化，避免刮到旁边的方块。
        arm.goto(obs_r * math.cos(math.radians(cur_deg)),
                 obs_r * math.sin(math.radians(cur_deg)),
                 lift_z, tag='半径对齐到观察位')
        swing_r = obs_r
    else:
        swing_r = cur_r
    delta = (cur_deg - obs_deg + 180.0) % 360.0 - 180.0
    if abs(delta) > 3.0:
        arm.swing(swing_r, lift_z, cur_deg, obs_deg)
    # 吸盘上挂着的那块就在这个臂坐标上，扫桌面时按它剔掉
    grip = (swing_r * math.cos(math.radians(obs_deg)),
            swing_r * math.sin(math.radians(obs_deg)))

    wait_still()
    colour, colour_path = snapshot('04_place_scan')
    depth = BRIDGE.imgmsg_to_cv2(
        rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw',
                               Image, timeout=5), 'passthrough')
    blocks = scan_table(depth, block_z, stack_h, layer_rise, avoid_xy=grip)

    print('    框出 %d 个方块（吸盘上这块已剔除）：' % len(blocks), flush=True)
    for i, b in enumerate(blocks):
        bx, by, bw, bh = b['box']
        print('      #%d 框=(%d,%d) %dx%d 面积=%.0f 层=%d 顶面z=%.0f -> 臂(%.0f,%.0f) r=%.0f'
              % (i, bx, by, bw, bh, b['area'], b['layer'], b['top_z'],
                 b['x'], b['y'], b['r']), flush=True)

    crowded = len(blocks) >= args.crowd
    spot = None
    pz = ground_z
    tgt = None
    how = ''
    if crowded:
        print('    桌面已摆 %d 个（>= %d），按规则叠第二层'
              % (len(blocks), args.crowd), flush=True)
        tgt = stack_target(blocks)
        if tgt is not None:
            spot = (tgt['x'], tgt['y'])
            pz = ground_z + stack_h
            how = '叠到 (%.0f,%.0f) 那块上面（第二层）' % (tgt['x'], tgt['y'])
    if spot is None:
        fs = free_spot(blocks, args.place_gap)
        if fs is not None:
            spot = (fs[0], fs[1])
            pz = ground_z
            how = '桌面空地，离最近方块净间隙 %.0fmm' % fs[2]
    if spot is None and not crowded:
        tgt = stack_target(blocks)
        if tgt is not None:
            spot = (tgt['x'], tgt['y'])
            pz = ground_z + stack_h
            how = '桌面找不到空地，改叠第二层 (%.0f,%.0f)' % (tgt['x'], tgt['y'])
    if spot is None:
        spot = args.place_default
        pz = ground_z
        how = '一个可用落点都找不到，退回默认点'
        print('    注意：%s' % how, flush=True)

    # 落点对应的「承放面」高度：方块底就贴在这个平面上
    surface_z = pz - args.grasp_offset - stack_h
    path = draw_scan(colour, blocks, spot, surface_z, tgt)
    print('    === 落点 (%.1f, %.1f, z=%.1f)：%s ==='
          % (spot[0], spot[1], pz, how), flush=True)
    cv2.imwrite('/tmp/pp_place_mask.jpg', colour)
    print('    扫描图（绿框=检出的方块，红框=被叠的那块，黄框=落点）: %s'
          % path, flush=True)
    do_place(arm, spot[0], spot[1], pz, lift_z, args.step)
    return True


def main():
    # 命令行给的速度要覆盖模块级常量，声明必须在任何引用之前
    global FEED_FAST, FEED_SLOW
    ap = argparse.ArgumentParser()
    ap.add_argument('--home', default='200,0,150',
                    help='开机初始位，流程从这里出发、最后回到这里')
    ap.add_argument('--observe', default='0,200,60',
                    help='观察位：关节1 转到最左、半径不变的那个点（实测 90°/r=200/z=60）。'
                         '填 same 表示不动臂，直接在当前位置识别')
    ap.add_argument('--travel-z', type=float, default=100.0,
                    help='水平移动时保持的高度')
    ap.add_argument('--lift-z', type=float, default=130.0,
                    help='抓起后垂直抬到的最终高度')
    ap.add_argument('--calib', action='append', default=None, metavar='FILE',
                    help='标定文件路径，可给多个（先到先得，字段级合并）。'
                         '不给则依次尝试环境变量 %s 和 %s'
                         % (CALIB_ENV, ' / '.join(CALIB_FILES_DEFAULT)))
    ap.add_argument('--planar-profile', metavar='FILE',
                    help='厂家平面标定配置；默认加载本机最新四系数做同帧对照')
    ap.add_argument('--grasp-source', choices=('dlt', 'official-planar'), default='dlt',
                    help='抓取 XY 来源。厂家平面实际执行要求已核实顶面高度及原始图像尺寸')
    ap.add_argument('--block-z', type=float, default=None,
                    help='方块顶面在臂坐标系中的高度。默认从标定文件的 '
                         'support_z_mm + object_height_mm 算，不再写死')
    ap.add_argument('--grasp-offset', type=float, default=None,
                    help='吸盘唇比末端橙色板低多少 mm。默认取标定文件的 cup_offset_mm')
    ap.add_argument('--y-offset', type=float, default=0.0,
                    help='反投影 y 向补偿（mm）。实测 clock 在 (168,272) 不加补偿正好抓准，'
                         '加了反而把目标推远到半径 348；这个经验值只在正前方一带成立，'
                         '方位角大时反而有害，所以默认 0')
    ap.add_argument('--target', default='',
                    help='要抓的方块名称：teddybear / clock / banana / cake。'
                         '给了就先用模板识别画面里所有方块、挑出同名的那个；'
                         '留空则沿用原来的深度凸起分割（不区分名称）')
    ap.add_argument('--place-at', default='',
                    help='放置点 x,y 或 x,y,z；留空则抓起后直接回初始位')
    ap.add_argument('--place-on', default='',
                    help='放到指定名称的方块正上方（叠上去）。填 same 表示与 --target '
                         '同名的另一个方块。方块高 100mm，所以落点 z 自动抬高 100mm')
    ap.add_argument('--stack-height', type=float, default=None,
                    help='被叠的那个方块的高度(mm)，决定放置时抬高多少。'
                         '默认取标定文件 object_height_mm')
    ap.add_argument('--layer', type=int, default=-1,
                    help='强制指定目标在第几层：0=桌面，1=叠在别的方块上。'
                         '默认 -1 表示由深度自动判断（靠顶面凸起量）')
    ap.add_argument('--layer-rise', type=float, default=None, help=(
        '一层方块顶面在深度图上实测的凸起量(mm)。相机是斜着看的，'
        '竖直 100mm 的方块在 rise 上量出来是 110mm 左右；'
        '两层就约 220。换相机角度要重新量'))
    ap.add_argument('--crowd', type=int, default=6,
                    help='桌面方块数达到多少个就改叠第二层（默认 6）。'
                         '六个、九个这种摆满的情况直接摞起来')
    ap.add_argument('--place-gap', type=float, default=20.0,
                    help='落点与已有方块之间要求的最小净间隙(mm)，默认 20。'
                         '方块按外接圆算占位，间隙给足就不会压角')
    ap.add_argument('--place-default', default='230,-70',
                    help='一个方块都框不到时的兜底落点 x,y（默认 230,-70）')
    ap.add_argument('--no-place', action='store_true',
                    help='不做避让放置，抓起后原样带回开机初始位（旧行为）')
    ap.add_argument('--place-only', action='store_true',
                    help='方块已在吸盘上，只执行放下，不重新识别和抓取')
    ap.add_argument('--step', type=float, default=0.0,
                    help='下降分段步长；0 为一步直达。带载起提始终使用单段垂直指令')
    ap.add_argument('--feed-fast', type=float, default=FEED_FAST,
                    help='快速档进给率（平移/转关节）。实测 80000 是固件饱和点，'
                         '再高反而变慢，别超过它')
    ap.add_argument('--feed-slow', type=float, default=FEED_SLOW,
                    help='慢速档进给率（空载接近）')
    ap.add_argument('--feed-loaded', type=float, default=FEED_LOADED,
                    help='开泵至泄压等待结束的进给率上限，默认 F10000，可调低')
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--detect-only', action='store_true',
                    help='走到观察位完成识别、打印结果后停下，不抓取。'
                         '用来单独验证识别与反投影')
    ap.add_argument('--classify-test', action='store_true',
                    help='走到观察位后用「深度找框 + 模板分类」逐个打印分数，'
                         '只做实验对比，不动臂抓取')
    args = ap.parse_args()
    if not all(math.isfinite(f) and f >= 1 for f in (args.feed_loaded, args.feed_slow, args.feed_fast)):
        ap.error('进给率必须是至少为 1 的有限值')
    if args.feed_loaded > FEED_LOADED:
        ap.error('--feed-loaded 上限是 F%.0f' % FEED_LOADED)
    # Temporary lock: grasp height and contact feedback require field validation.
    # This CLI must not command the arm until the press-down incident is resolved.
    if args.execute or args.detect_only or args.classify_test or args.place_only:
        raise SystemExit(
            'Arm motion locked after press-down incident. '
            'Validate contact height and independent feedback before re-enabling. '
            'Run without motion flags for a preview.'
        )
    if args.detect_only or args.classify_test:
        args.execute = True  # 允许走到观察位，识别完再退出

    planar = None
    if not args.place_only:
        profile = Path(args.planar_profile).expanduser() if args.planar_profile else next(
            (Path(root)/PLANAR_PROFILE for root in _workspace_roots()
             if (Path(root)/PLANAR_PROFILE).is_file()), None)
        if profile is not None:
            try:
                planar = load_profile(profile)
            except (OSError, ValueError) as exc:
                if args.planar_profile or args.grasp_source == 'official-planar':
                    ap.error(str(exc))
                print('[标定] 无法加载厂家平面对照：%s' % exc, flush=True)
        if args.grasp_source == 'official-planar' and planar is None:
            ap.error('未找到厂家平面标定配置')
        if args.grasp_source == 'official-planar' and args.execute:
            measured_planes = [plane for plane in planar['planes']
                               if args.target in plane['labels'] and plane['top_z_mm'] is not None
                               and plane['image_size'] is not None]
            if not measured_planes:
                ap.error('厂家平面标定只可预览：实际执行需显式目标、实测顶面高度和标定分辨率')
            planar = dict(planar, planes=measured_planes)

    # 命令行给的速度要覆盖模块级常量，所以这里显式写回全局
    FEED_FAST = float(args.feed_fast)
    FEED_SLOW = float(args.feed_slow)
    print('速度：快档 F%.0f (%.0fmm/s)、慢档 F%.0f (%.0fmm/s)'
          % (FEED_FAST, speed_of(FEED_FAST), FEED_SLOW, speed_of(FEED_SLOW)),
          flush=True)

    home = parse_xyz(args.home)
    # 兜底落点解析成 (x, y)，自动放置里当 tuple 用
    args.place_default = parse_xy(args.place_default)
    # same/none/here：不收臂，就站在初始位这一帧上识别
    if args.observe.strip().lower() in ('same', 'none', 'here'):
        obs = None
    else:
        obs = parse_xyz(args.observe)

    rospy.init_node('arm_pick_place', anonymous=True)
    arm = Arm(loaded_feed=min(args.feed_loaded, FEED_SLOW, FEED_FAST))

    # 标定必须在任何反投影、任何动作之前读好：A/B、内参、方块高度、吸盘偏移
    # 全部来自标定文件，命令行只用来覆盖（--block-z 等留空就走文件值）。
    calib = load_calibration(files=args.calib, use_camera_info=True)
    if args.block_z is None:
        if calib['block_z'] is None:
            raise SystemExit('标定文件里没有 support_z_mm，无法确定方块顶面高度，'
                             '请用 --block-z 手动指定')
        args.block_z = calib['block_z']
    if args.grasp_offset is None:
        if calib['grasp_offset'] is None:
            raise SystemExit('标定文件里没有 cup_offset_mm，无法确定吸盘偏移，'
                             '请用 --grasp-offset 手动指定')
        args.grasp_offset = calib['grasp_offset']
    if args.stack_height is None:
        args.stack_height = calib['stack_height']
    if args.layer_rise is None:
        args.layer_rise = args.stack_height
    print('[layer] physical layer spacing: %.1f mm' % args.layer_rise, flush=True)
    print('实际使用：block_z=%.1f  grasp_offset=%.1f  stack_height=%.1f'
          % (args.block_z, args.grasp_offset, args.stack_height), flush=True)

    # 只放置：方块已经在吸盘上，跳过识别和抓取，直接走放下那一段。
    # 这条路径不需要知道方块在哪，因为它就挂在吸盘上跟着走。
    if args.place_only:
        if not args.place_at:
            print('--place-only 必须同时给 --place-at x,y', flush=True)
            return 1
        px, py, pz = parse_place(args.place_at, args.block_z + args.grasp_offset)
        print('=== 只执行放置 -> (%.1f, %.1f, %.1f) ===' % (px, py, pz), flush=True)
        arm.vacuum_requested = True  # --place-only 从持物状态接管
        arm.attach()
        arm.goto(home[0], home[1], args.lift_z, tag='从初始位降到抬升高度')
        do_place(arm, px, py, pz, args.lift_z, args.step)
        arm.goto(home[0], home[1], args.lift_z, tag='平移回初始位上方')
        arm.goto(home[0], home[1], home[2], tag='回到初始位')
        _, path = snapshot('09_placed')
        print('DONE  终点图: %s' % path, flush=True)
        return 0

    print('=== 1. 出发：开机初始位 (%.0f, %.0f, %.0f) ===' % home, flush=True)
    home_r = math.hypot(home[0], home[1])
    home_deg = math.degrees(math.atan2(home[1], home[0]))
    if args.execute:
        arm.attach()
        # 先把起点钉死在初始位：上电后臂可能停在别处，后面所有动作都假设从这里出发。
        # 这是全流程唯一一处「不知道从哪出发」，行程算不出来，只能给保守值。
        arm.goto(home[0], home[1], home[2], dwell=3.0, tag='回到开机位')

    # 让视野：原地降高度 -> 只转关节1 转到最左。半径全程不变，
    # 大臂小臂姿态不变，只有底座在转，所以机械臂不会伸到方块上面去挡镜头。
    print('=== 2. 转关节1 到最左，给相机让视野 ===', flush=True)
    if args.execute and obs is not None:
        obs_r = math.hypot(obs[0], obs[1])
        obs_deg = math.degrees(math.atan2(obs[1], obs[0]))
        # 先降到观察高度：高度变化放在半径不变的竖直线上，逆解最稳
        arm.goto(home[0], home[1], obs[2], tag='原地降到观察高度')
        arm.swing(home_r, obs[2], home_deg, obs_deg)
        if abs(obs_r - home_r) > 15:
            # 观察位半径和初始位差得多时才补一条，正常情况不用走
            arm.goto(obs[0], obs[1], obs[2], tag='半径微调到观察位')
        print('    观察位 u=%.1f° r=%.0f z=%.0f' % (obs_deg, obs_r, obs[2]), flush=True)

    print('=== 3. 识别方块 ===', flush=True)
    wait_still()
    colour, colour_path = snapshot('00_detect')
    depth = BRIDGE.imgmsg_to_cv2(
        rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image, timeout=5),
        'passthrough')
    # 每帧现算的「比地面高多少」图。方块顶面在它上面是常数，
    # 一层约 110mm、两层约 220mm，用来判断目标是不是叠在别的方块上。
    rise = rise_map(depth)
    if rise is None:
        print('    地面基线拟合失败，层数判断退回按桌面层处理', flush=True)

    if args.classify_test:
        blobs, _m = find_all_blocks(colour, depth)
        print('深度找到 %d 个凸起块' % len(blobs), flush=True)
        for i, bb in enumerate(blobs):
            bx, by, bw, bh = bb['box']
            sc = classify_roi(colour, bb['box'])
            uv = _top_face_uv(bb['box'], depth) or (bx + bw / 2.0, float(by))
            patch = depth[by:by + bh, bx:bx + bw]
            sel = patch[patch > 0]
            dmm = int(np.median(sel)) if sel.size else 0
            ax = ay = ar = float('nan')
            if dmm > 0:
                ax, ay = pixel_to_arm(uv[0], uv[1], dmm, args.block_z)
                ar = math.hypot(ax, ay)
            print('  #%d 框=(%d,%d,%dx%d) 面积=%.0f 填充=%.2f 顶面=(%.0f,%.0f) 深度=%d'
                  ' -> 臂(%.0f,%.0f) r=%.0f'
                  % (i, bx, by, bw, bh, bb['area'], bb['fill'],
                     uv[0], uv[1], dmm, ax, ay, ar), flush=True)
            print('     分类: ' + '  '.join('%s=%.3f' % (l, s) for l, s in sc), flush=True)
        cv2.imwrite('/tmp/pp_classify.jpg', colour)
        return 0

    found = []
    seen = []          # 画面里每个方块的臂坐标，抓完后要放置时直接查这张表
    if args.target:
        print('    === 按名称识别：目标 %s ===' % args.target, flush=True)
        # 叠同名方块时桌上必须还有第二个，只识别到一个就等于没得放
        need_hits = 2 if args.place_on.strip().lower() in ('same', 'target', 'self') else 1
        block, mask, found = find_named_block(colour, depth, args.target,
                                             min_hits=need_hits)
        cv2.imwrite('/tmp/pp_mask.png', mask)
        if found:
            print('    画面里识别到 %d 个方块：' % len(found), flush=True)
            for d in sorted(found, key=lambda t: -t.score):
                mark = ' <== 目标' if d.label == args.target else ''
                dbx = (int(round(d.x - d.width / 2)), int(round(d.y - d.height / 2)),
                       int(round(d.width)), int(round(d.height)))
                duv = _top_face_uv(dbx, depth) or (d.x, d.y - d.height * 0.32)
                dpatch = depth[max(0, dbx[1]):max(0, dbx[1]) + max(1, dbx[3]),
                               max(0, dbx[0]):max(0, dbx[0]) + max(1, dbx[2])]
                dsel = dpatch[dpatch > 0]
                dmm = int(np.median(dsel)) if dsel.size else 0
                # 这个方块在第几层：叠着的要按抬高后的顶面高度反投影，
                # 否则 x/y 会偏出去十几到几十毫米
                dlayer, drise = measure_layer(rise, dbx, args.layer_rise,
                                              args.stack_height)
                if dlayer is None:
                    print('      %s: layer unknown; no arm coordinates generated' % d.label, flush=True)
                    continue
                dtop_z = args.block_z + dlayer * args.stack_height
                ax = ay = ar = float('nan')
                if dmm > 0:
                    ax, ay = pixel_to_arm(duv[0], duv[1], dmm, dtop_z)
                    ay += args.y_offset
                    ar = math.hypot(ax, ay)
                    # 只收落在可达范围内的，离群点不参与后面的放置选点
                    if 80.0 < ar < 355.0:
                        seen.append({'label': d.label, 'x': ax, 'y': ay,
                                     'r': ar, 'score': d.score, 'layer': dlayer,
                                     'top_z': dtop_z})
                print('      %-10s score=%.2f 中心=(%.0f,%.0f) 框=%.0fx%.0f '
                      '顶面=(%.0f,%.0f) 深度=%d 层=%d(凸起%s) -> 臂(%.0f,%.0f) 半径=%.0f%s'
                      % (d.label, d.score, d.x, d.y, d.width, d.height,
                         duv[0], duv[1], dmm, dlayer,
                         ('%.0fmm' % drise) if drise is not None else '?',
                         ax, ay, ar, mark), flush=True)
        else:
            print('    一个方块都没识别到', flush=True)
    else:
        block, mask = find_block(colour, depth)
        cv2.imwrite('/tmp/pp_mask.png', mask)

    if block is None:
        print('NO BLOCK FOUND（画面里没找到%s）'
              % ('名为 %s 的方块' % args.target if args.target else '方块'), flush=True)
        print('检测图: %s' % colour_path, flush=True)
        return 1

    debug = colour.copy()
    for d in found:
        x0 = int(round(d.x - d.width / 2))
        y0 = int(round(d.y - d.height / 2))
        hit = d.label == args.target
        cv2.rectangle(debug, (x0, y0),
                      (int(round(x0 + d.width)), int(round(y0 + d.height))),
                      (0, 255, 0) if hit else (160, 160, 160), 2)
        cv2.putText(debug, '%s %.2f' % (d.label, d.score), (x0, max(14, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if hit else (160, 160, 160), 2)
    bx, by, bw, bh = block['box']
    cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (0, 200, 255), 2)
    cv2.drawMarker(debug, (int(block['u']), int(block['v'])), (0, 255, 0),
                   cv2.MARKER_CROSS, 22, 2)
    cv2.imwrite('/tmp/pp_detect_annotated.jpg', debug)

    # 目标在第几层：cake 开局就叠在别的方块上，顶面比桌面高两个方块身，
    # 抓它的吸盘必须同步抬高，否则要么吸不到、要么把下层方块压坏。
    layer, measured = measure_layer(rise, block['box'], args.layer_rise,
                                   args.stack_height)
    if layer is None:
        print('[layer] unknown: cannot calculate grasp height; stop', flush=True)
        return 3
    if args.layer >= 0 and args.layer != layer:
        print('[layer] requested layer disagrees with measurement; stop', flush=True)
        return 3
    top_z = args.block_z + layer * args.stack_height
    if layer > 0:
        print('    === 目标在第 %d 层：顶面比桌面高 %.0fmm，抓取高度整体抬高 %.0fmm ==='
              % (layer, layer * args.stack_height, layer * args.stack_height),
              flush=True)
    else:
        print('    目标在桌面层（顶面凸起 %s），按常规高度抓取'
              % ('%.0fmm' % measured if measured is not None else '未测到'), flush=True)

    x, y = pixel_to_arm(block['u'], block['v'], block['depth'], top_z)
    if planar is not None:
        pixel = (block['u'], block['v'])
        label = block.get('label') or args.target
        image_size = [colour.shape[1], colour.shape[0]]
        selected = args.grasp_source == 'official-planar'
        try:
            delta, model = project_top({'official_planar': planar}, label, pixel, top_z,
                                       image_size, require_height=selected and args.execute)
        except ValueError as exc:
            if selected:
                print('[标定] 拒绝厂家平面坐标：%s' % exc, flush=True)
                return 4
            print('[标定] 厂家平面对照不可用：%s' % exc, flush=True)
        else:
            print('[标定] 厂家平面 %s → (%.1f, %.1f) mm；DLT (%.1f, %.1f) mm；XY 差 %.1f mm'
                  % (model['source_file'], delta[0], delta[1], x, y,
                     math.hypot(delta[0] - x, delta[1] - y)), flush=True)
            if selected:
                x, y = delta[:2]
    y += args.y_offset
    print('[定位] 目标：%s；图像中心：(u=%.1f, v=%.1f) 像素；深度：%d mm' %
          (block.get('label') or args.target or '未分类目标',block['u'],block['v'],block['depth']),flush=True)
    print('[转换] 相机坐标系：当前旧入口未保留 header.frame_id（来源话题 /camera/color/image_raw）；机械臂坐标系：arm_base；坐标单位：毫米',flush=True)
    print('[转换] 目标机械臂坐标：(%.1f, %.1f, %.1f) mm' % (x,y,top_z),flush=True)
    print('    像素 uv=(%.1f, %.1f) 深度=%d 面积=%.0f 填充=%.2f 凸起=%.0fmm%s' %
          (block['u'], block['v'], block['depth'], block['area'],
           block['fill'], block['rise'],
           ' 名称=%s(%.2f)' % (block['label'], block['score'])
           if block.get('label') else ''), flush=True)

    # 几何自检：10cm 的方块在这个深度上应该占多少像素，对不上就说明框错了东西。
    # 按名称识别时模板框会带透视外扩，宽度本来就偏大，只提示不拦。
    expect_px = 100.0 * FX / max(block['depth'], 1)
    bx0, by0, bw0, bh0 = block['box']
    print('    尺寸校验：实宽 %dpx，10cm 方块应约 %.0fpx（偏差 %+.0f%%）' %
          (bw0, expect_px, (bw0 - expect_px) / expect_px * 100), flush=True)
    if not 0.6 < bw0 / expect_px < 1.5:
        if args.target:
            print('    尺寸偏差偏大，但名称已确认，继续执行', flush=True)
        else:
            print('尺寸对不上，框到的不是 10cm 方块，拒绝执行', flush=True)
            return 3
    print('    臂坐标 x=%.1f y=%.1f 顶面 z=%.1f（第 %d 层）%s' %
          (x, y, top_z, layer, ' (顶面)' if block.get('top_face') else ''),
          flush=True)

    radius = math.hypot(x, y)
    if not 90.0 < x < 345.0 or not 80.0 < radius < 355.0:
        print('反投影结果离群（x=%.1f y=%.1f 半径=%.0f），拒绝执行，避免撞臂'
              % (x, y, radius), flush=True)
        return 2
    if x > 320.0:
        print('    注意：x=%.0f 已接近最大臂展，若固件回 E22 就是够不着' % x, flush=True)

    if not args.execute:
        print('dry run：检测通过，加 --execute 才动臂', flush=True)
        return 0

    if args.detect_only:
        print('detect-only：识别与反投影已完成，到此停下，不动臂抓取', flush=True)
        return 0

    # 从观察位回正前方：同样只转关节1，半径不变。
    # 不要一条 G0 从最左直接斜插到方块上方——那是横穿整个工作区的斜线，
    # 末端会从方块侧面扫过去。
    # 抓取高度跟着层数走：吸盘唇要落在方块顶面上，
    # 第二层的顶面比桌面层高一个方块身，所以 z 也整体抬高同样的量。
    grasp_z = top_z + args.grasp_offset
    if not (math.isfinite(grasp_z) and math.isfinite(args.lift_z) and args.lift_z > grasp_z):
        raise ValueError('带载起提终点必须高于抓取高度')
    # 巡航高度也要跟着抬：吸盘唇比末端板低 82mm，叠在第二层的方块
    # 顶面已经在 -32，如果还按 70 平移过去，吸盘唇只有 20mm 余量，
    # 稍微下沉就刮到方块。给到 grasp_z + 40，保证有 40mm 的接近余量。
    approach_z = max(SAFE_Z, grasp_z + 40.0)
    print('=== 4. 转回正前方，平移到方块正上方 ===', flush=True)
    if obs is not None:
        obs_r = math.hypot(obs[0], obs[1])
        obs_deg = math.degrees(math.atan2(obs[1], obs[0]))
        arm.swing(obs_r, approach_z, obs_deg, home_deg)
    else:
        arm.goto(home[0], home[1], approach_z, tag='降到安全高度')
    arm.goto(x, y, approach_z, tag='方块正上方')

    print('=== 5. 原地垂直下降到抓取高度 (z=%.1f) ===' % grasp_z, flush=True)
    arm.descend(x, y, approach_z, grasp_z, step=args.step)
    snapshot('01_at_block')

    print('=== 6. 吸住 ===', flush=True)
    arm.pump_on(hold=2.5)

    print('=== 7. 纯垂直抬升（x/y 锁定不动）===', flush=True)
    # 每次 G0 都会重新起停，带载离开支撑面到转运高度只发一个固定 XY 的目标。
    arm.ascend(x, y, grasp_z, args.lift_z, feed=arm.loaded_feed)
    _, lifted_path = snapshot('02_lifted')
    print('    起提指令等待结束，持物待确认: %s' % lifted_path, flush=True)

    placed = False
    if args.place_on:
        name = args.place_on.strip().lower()
        if name in ('same', 'target', 'self'):
            name = (args.target or '').strip().lower()
        if not name:
            print('--place-on same 必须同时给 --target，改为直接带回初始位', flush=True)
        else:
            cand = [s for s in seen if s['label'] == name]
            # 同名叠同名时要把手里这个剔掉：它刚才就在 (x, y)，
            # 那个坐标现在已经是空的，落上去会放到空气里。
            if name == args.target:
                cand = [s for s in cand if math.hypot(s['x'] - x, s['y'] - y) > 60.0]
            if not cand:
                print('桌上找不到可叠的 %s 方块，改为直接带回初始位' % name, flush=True)
            else:
                tgt = max(cand, key=lambda s: s['score'])
                px, py = tgt['x'], tgt['y']
                # 落点抬高一个方块高度，基准是被叠那块的顶面：
                # 它本身要是叠在第二层（比如开局就在高处的 cake），
                # 落点还要再加一层，否则会把下面摞好的两摞压塌。
                pz = tgt['top_z'] + args.grasp_offset + args.stack_height
                print('=== 8. 叠到 %s 方块上方 (%.1f, %.1f, %.1f)（它顶面 z=%.1f）==='
                      % (name, px, py, pz, tgt['top_z']), flush=True)
                do_place(arm, px, py, pz, args.lift_z, args.step)
                placed = True
    elif args.place_at:
        px, py, pz = parse_place(args.place_at, args.block_z + args.grasp_offset)
        print('=== 8. 放到指定点 (%.1f, %.1f, %.1f) ===' % (px, py, pz), flush=True)
        do_place(arm, px, py, pz, args.lift_z, args.step)
        placed = True
    elif not args.no_place:
        # 默认行为：不写死坐标，带着方块摆到最左边看一眼桌面，
        # 绕开已有的方块放；方块摆满了就叠第二层（见 auto_place）。
        placed = auto_place(arm, args, home, obs, args.block_z,
                            args.stack_height, args.layer_rise)
    else:
        print('=== 8. --no-place：不做放置，直接把方块带回初始位 ===', flush=True)

    print('=== 9. 回开机初始位 ===', flush=True)
    arm.goto(home[0], home[1], args.lift_z, tag='平移回初始位上方')
    arm.goto(home[0], home[1], home[2], tag='回到初始位')

    _, final_path = snapshot('03_home')
    print('DONE  终点图: %s' % final_path, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
