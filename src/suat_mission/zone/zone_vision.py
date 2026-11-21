"""视觉辅助函数及历史分区标记工具。

分拣区场景约定：调用前机械臂由外部移至最左侧，机器人到达四个分拣观察点之一。
地面第一层为 3×3 个边长约 10 cm 的立方物块；中央上方第二层为蛋糕物块。
普通物块类别为小熊、时钟、香蕉。

目标流程：采集一帧后逐类别识别，在同一画面叠加四类的名称、检测框与匹配分数，
并报告数量；一层高约 10 cm，二层高约 20 cm，层数应依据有效深度及已核实的
支撑平面判断，无法确认时保留未知。仅有匹配分数不能计算真实识别率；识别率
需要带真实标注的测试样本及明确的统计口径。

下方标记寻找、解码与 zone_mapping 分区映射是保留的历史功能，不代表分拣区
需要识别黑色标准码。本段记录目标场景，不表示逐类结果的同帧叠加已经实现。
"""
import os
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:                                    # 同目录导入；单独拷这一个文件时用下面的兜底
    from zone_mapping import normalize_zone
except ImportError:                     # pragma: no cover
    import re as _re

    def normalize_zone(text):
        if not text:
            return None
        found = set(_re.findall(r'(?<![A-Z])([BCD])(?![A-Z])', str(text).upper()))
        return found.pop() if len(found) == 1 else None

MARKER_RECT = 160           # 二维码整流后的边长（像素）
SIG_GRID = 16               # 无法解码时，用 16x16 占位图当指纹
SIG_CROP = 0.80             # 指纹只看中间 80%，把外面的黑框切掉（黑框占满一圈会让所有码长得一样）



def imread(path: str) -> Optional[np.ndarray]:
    """读图，支持中文/空格路径（Windows 上 cv2.imread 遇非 ASCII 路径会静默返回 None）。"""
    if not path:
        return None
    try:
        data = np.fromfile(path, dtype=np.uint8)
    except (IOError, OSError):
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite(path: str, image: np.ndarray) -> bool:
    """写图，同样绕开非 ASCII 路径。"""
    ext = os.path.splitext(path)[1] or '.jpg'
    ok, buffer = cv2.imencode(ext, image)
    if not ok:
        return False
    with open(path, 'wb') as handle:
        handle.write(buffer.tobytes())
    return True


# --------------------------------------------------------------------------- #
# 相机取帧（比赛现场唯一正确的输入来源：照片只是离线调试用）
# --------------------------------------------------------------------------- #
DEFAULT_COLOR_TOPIC = '/camera/color/image_raw'


def grab_ros_frame(topic: str = DEFAULT_COLOR_TOPIC, timeout: float = 6.0) -> np.ndarray:
    """从 ROS 话题抓一帧 BGR 图。

    只做一次订阅 + wait_for_message：不启动/重启相机，不发任何控制指令，
    所以机械臂和相机状态都不会被动到。调用方负责先 source ROS 环境。
    """
    import rospy                                   # 只在这条路径上依赖 ROS
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    if not rospy.core.is_initialized():
        rospy.init_node('zone_grab', anonymous=True, disable_signals=True)
    message = rospy.wait_for_message(topic, Image, timeout=timeout)
    return CvBridge().imgmsg_to_cv2(message, 'bgr8')


def grab_frame(topic: str = DEFAULT_COLOR_TOPIC, timeout: float = 6.0,
               camera_index: int = 0, allow_usb: bool = False) -> np.ndarray:
    """抓一帧。默认只认 ROS 话题；allow_usb 打开后才退回本机 USB 摄像头。

    为什么默认不退回：笔记本/工控机上随便一个 Camera 0 都能开，拿到的却是天花板或
    桌面，识别出来一堆"unknown"还照样打印结论——比直接报错危险得多。
    """
    try:
        return grab_ros_frame(topic, timeout)
    except Exception as ros_error:                 # noqa: BLE001 - 任何 ROS 侧问题都该退回
        if not allow_usb:
            raise RuntimeError('从话题 %s 取帧失败：%s' % (topic, ros_error))
        capture = cv2.VideoCapture(camera_index)
        if not capture.isOpened():
            raise RuntimeError('ROS 话题 %s 取帧失败（%s），USB 摄像头 %d 也打不开'
                               % (topic, ros_error, camera_index))
        try:
            for _ in range(5):                     # 丢掉前几帧，等曝光稳定
                ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError('USB 摄像头 %d 读帧失败' % camera_index)
            return frame
        finally:
            capture.release()


# --------------------------------------------------------------------------- #
# 历史分区标记功能；分拣区物块识别不应将黑色垫片当作标准码。
# --------------------------------------------------------------------------- #
def _order_quad(pts: np.ndarray) -> np.ndarray:
    """把四个点排成 左上/右上/右下/左下。"""
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(s)], pts[np.argmin(d)],
                     pts[np.argmax(s)], pts[np.argmax(d)]], dtype=np.float32)


def _warp(image: np.ndarray, quad: np.ndarray, size: int = MARKER_RECT) -> np.ndarray:
    src = _order_quad(quad)
    dst = np.float32([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]])
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, matrix, (size, size), flags=cv2.INTER_CUBIC)


def find_markers(image: np.ndarray, min_side_ratio: float = 0.012,
                 max_side_ratio: float = 0.25, max_mean_gray: float = 135.0,
                 min_fill: float = 0.55, work_width: int = 1280) -> List[dict]:
    """找出画面里的"黑色实心方块"标记，按 x 从左到右排序。

    返回 [{'quad': 4x2, 'cx': float, 'side': float, 'rect': 灰度方块图}, ...]

    判据：外接旋转矩形近正方 + 尺寸合理 + 外接框被填满(实心) + 内部偏暗(黑底白图案)。
    然后按"同一排、差不多大"聚成一簇，取最像的那一簇，避免把地上的大字当成码。
    """
    if image is None or not image.size:
        raise ValueError('空图像')
    scale = min(1.0, work_width / float(image.shape[1]))
    small = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else image
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV, 41, 9)
    # 白图案会在黑方块里开出洞，先闭运算把洞补上，方块才是一个整块
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    image_area = float(gray.shape[0] * gray.shape[1])
    candidates: List[dict] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < image_area * 1e-4 or area > image_area * 0.09:
            continue
        rect = cv2.minAreaRect(contour)
        (cx, cy), (rw, rh), _ = rect
        side = min(rw, rh)
        if side < gray.shape[1] * min_side_ratio or side > gray.shape[1] * max_side_ratio:
            continue
        # 1.75 而不是 1.45：贴地的码被机械臂/线缆挡掉一角时，外接旋转矩形会明显拉长，
        # 原来 1.45 会把真实存在的第三个码直接丢掉（实测 1.47 就被过滤了）
        if not 0.70 <= max(rw, rh) / max(side, 1e-6) <= 1.75:
            continue
        fill = area / max(rw * rh, 1e-6)          # 实心度
        if fill < min_fill:
            continue
        box = cv2.boxPoints(rect).astype(np.int32)
        mask = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(mask, [box], -1, 255, -1)
        mask = cv2.erode(mask, np.ones((7, 7), np.uint8))
        if not mask.any():
            continue
        mean_gray = float(cv2.mean(gray, mask=mask)[0])
        if mean_gray > max_mean_gray:             # 白底、纸箱这类亮面
            continue
        candidates.append({'cy': float(cy), 'cx': float(cx), 'side': float(side),
                           'fill': float(fill), 'mean': mean_gray, 'box': box})

    chosen = _best_row(candidates)
    if not chosen:                                 # 没凑齐一排，退化成按实心度取最好的三个
        chosen = sorted(candidates, key=lambda c: (-c['fill'], c['mean']))[:3]

    markers = []
    for item in sorted(chosen, key=lambda c: c['cx']):
        quad = item['box'].astype(np.float32) / scale
        cx = float(item['box'][:, 0].mean() / scale)
        rect_gray = _warp(gray, item['box'].astype(np.float32))
        wide = _warp(gray, expand_quad(item['box'].astype(np.float32), 0.15))
        markers.append({'quad': quad, 'cx': cx, 'side': float(item['side'] / scale),
                        'fill': item['fill'], 'rect': rect_gray, 'rect_wide': wide})
    return markers


def _best_row(candidates: List[dict]) -> List[dict]:
    """把候选按"同一排、大小接近"聚簇，返回最像三个码的那一簇。"""
    if not candidates:
        return []
    remaining = sorted(candidates, key=lambda c: c['cx'])
    groups: List[List[dict]] = []
    for item in remaining:
        for group in groups:
            ref = group[0]
            same_row = abs(item['cy'] - ref['cy']) <= 0.45 * min(item['side'], ref['side'])
            same_size = abs(item['side'] - ref['side']) <= 0.40 * max(item['side'], ref['side'])
            if same_row and same_size:
                group.append(item)
                break
        else:
            groups.append([item])
    groups = [g for g in groups if len(g) >= 3]
    if not groups:
        return []
    groups.sort(key=lambda g: (-len(g), -float(np.mean([c['fill'] for c in g])),
                               float(np.mean([c['mean'] for c in g]))))
    return _pick_spread(groups[0], 3)


def _pick_spread(candidates: List[dict], count: int = 3) -> List[dict]:
    """从一排候选里挑 count 个"横向撒得最开"的。

    为什么：一个真二维码会贡献 3 个定位方块（左上/右上/左下），它们都在同一个码里、
    彼此只隔半个码宽。如果直接按实心度取前三个，很可能三个都来自同一张码，
    "左中右"顺序就全错了。按"最小间距最大"来挑，才会一个码取一个。
    """
    if len(candidates) <= count:
        return sorted(candidates, key=lambda c: c['cx'])
    if len(candidates) > 12:                       # 组合爆炸时退回贪心
        ordered = sorted(candidates, key=lambda c: c['cx'])
        picked = [ordered[0], ordered[-1]]
        rest = [c for c in ordered[1:-1]]
        while len(picked) < count and rest:
            far = max(rest, key=lambda c: min(abs(c['cx'] - p['cx']) for p in picked))
            picked.append(far)
            rest.remove(far)
        return sorted(picked, key=lambda c: c['cx'])

    from itertools import combinations
    best, best_score = None, -1.0
    for combo in combinations(candidates, count):
        xs = sorted(c['cx'] for c in combo)
        gaps = [b - a for a, b in zip(xs, xs[1:])]
        score = min(gaps) if gaps else 0.0
        score += 0.001 * float(np.mean([c['fill'] for c in combo]))   # 间距打平时看谁更实
        if score > best_score:
            best, best_score = combo, score
    return sorted(best or candidates[:count], key=lambda c: c['cx'])


def expand_quad(quad: np.ndarray, margin: float = 0.12) -> np.ndarray:
    """把四边形围绕中心放大一点，给二维码留出安静区。"""
    pts = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    center = pts.mean(axis=0)
    return (center + (pts - center) * (1.0 + margin)).astype(np.float32)


def decode_marker(rect: np.ndarray) -> str:
    """尽量把整流后的方块解成文本，解不出来返回 ''。

    依次尝试：QRCodeDetector（原尺寸/放大/补安静区）-> 可选 pyzbar。
    这是保留的可选标记解码工具；分拣区没有要求识别黑色标准码。
    """
    if rect is None or not rect.size:
        return ''
    gray = rect if rect.ndim == 2 else cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    detector = cv2.QRCodeDetector()
    for attempt in (gray, cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)):
        for pad in (0, 12):
            img = cv2.copyMakeBorder(attempt, pad, pad, pad, pad,
                                     cv2.BORDER_CONSTANT, value=255) if pad else attempt
            try:
                text, _, _ = detector.detectAndDecode(img)
            except cv2.error:
                text = ''
            if text:
                return text.strip()
        try:                                    # 部分 OpenCV 版本只有 multi 接口能出结果
            ok, texts, _, _ = detector.detectAndDecodeMulti(attempt)
            if ok and any(texts):
                return next(t for t in texts if t).strip()
        except (cv2.error, AttributeError):
            pass
    try:
        from pyzbar import pyzbar         # 可选依赖
        for symbol in pyzbar.decode(gray):
            if symbol.data:
                return symbol.data.decode('utf-8', 'ignore').strip()
    except Exception:
        pass
    return ''


def marker_signature(rect: np.ndarray, grid: int = SIG_GRID, crop: float = SIG_CROP) -> str:
    """解码不出来时的兜底指纹：把"白图案占位"压成 grid*grid 的位图，转十六进制。

    两个细节决定它能不能用：
      * 先切掉外圈黑框。带框的话 16x16 里有一圈全是 1，三个码的距离只剩 4~6 位，
        根本分不开；切到中间 80% 后同样的三个码能差到 50~70 位（共 256 位）。
      * 先做 5%~95% 分位拉伸再 Otsu，抗曝光/白平衡差异。
    """
    if rect is None or not rect.size:
        return ''
    gray = rect if rect.ndim == 2 else cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape[:2]
    y0, x0 = int(height * (1 - crop) / 2), int(width * (1 - crop) / 2)
    inner = gray[y0:height - y0, x0:width - x0]
    if inner.size == 0:
        inner = gray
    lo, hi = np.percentile(inner, 5), np.percentile(inner, 95)
    scaled = np.clip((inner.astype(np.float32) - lo) * 255.0 / max(1.0, float(hi - lo)), 0, 255)
    _, binary = cv2.threshold(scaled.astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    small = cv2.resize(binary, (grid, grid), interpolation=cv2.INTER_AREA)
    bits = 0
    for value in small.ravel():
        bits = (bits << 1) | (1 if value > 127 else 0)
    return '%0*x' % (grid * grid // 4, bits)


def signature_distance(a: str, b: str) -> int:
    """两个指纹的汉明距离；长度不一致返回一个很大的数。"""
    if not a or not b or len(a) != len(b):
        return 10 ** 6
    return bin(int(a, 16) ^ int(b, 16)).count('1')


def load_marker_library(path: Optional[str]) -> Dict[str, str]:
    """可选：预先把每个区二维码的照片存成 <区>.png / <区>.jpg，用指纹做匹配。

    目录形如 zone_markers/B.png、zone_markers/C.png、zone_markers/D.png。
    有它就多一条"真的认出是哪个区"的路；没有就只按位置约定，不影响结果正确性。
    """
    library: Dict[str, str] = {}
    if not path or not os.path.isdir(path):
        return library
    for name in sorted(os.listdir(path)):
        stem, ext = os.path.splitext(name)
        if ext.lower() not in ('.png', '.jpg', '.jpeg', '.bmp'):
            continue
        zone = stem.strip().upper()[:1]
        if zone not in ('B', 'C', 'D'):
            continue
        img = imread(os.path.join(path, name))
        if img is None:
            continue
        if img.shape[0] != MARKER_RECT or img.shape[1] != MARKER_RECT:
            found = find_markers(img)          # 给的是现场照片：抠出里面那个码
            if not found:
                print('zone_markers/%s 里没找到方块标记，跳过' % name)
                continue
            img = max(found, key=lambda item: item['side'])['rect']
        library[zone] = marker_signature(img)
    return library


def _decode_qr_multi(image: np.ndarray, work_width: int = 1400) -> List[dict]:
    """整图找二维码。返回 [{'cx','quad','text'}]，按 cx 排好。

    两条路，因为版本差异很大：
      * OpenCV >= 4.5：detectAndDecodeMulti 一次给全部；
      * OpenCV 4.2（机器人就是）：multi 接口基本不出结果，只能反复调单码
        detectAndDecode，每找到一张就用白色多边形盖掉，再找下一张。
    """
    scale = min(1.0, work_width / float(image.shape[1]))
    work = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else image
    gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY) if work.ndim == 3 else work
    detector = cv2.QRCodeDetector()
    found: List[dict] = []

    try:
        ok, texts, points, _ = detector.detectAndDecodeMulti(gray)
        if ok and points is not None:
            for text, quad in zip(texts, points):
                quad = np.asarray(quad, dtype=np.float32).reshape(4, 2)
                found.append({'cx': float(quad[:, 0].mean()), 'quad': quad,
                              'text': (text or '').strip()})
    except (cv2.error, AttributeError):
        pass

    if len(found) < 2:                       # 4.2 的 multi 靠不住，逐张来
        found = []
        canvas = gray.copy()
        for _ in range(6):
            try:
                text, points, _ = detector.detectAndDecode(canvas)
            except cv2.error:
                break
            if points is None:
                break
            quad = np.asarray(points, dtype=np.float32).reshape(-1, 2)[:4]
            cx = float(quad[:, 0].mean())
            if text:
                found.append({'cx': cx, 'quad': quad, 'text': text.strip()})
            # 盖掉这一张，否则下一轮还是它
            hull = cv2.convexHull(quad.astype(np.int32))
            cv2.fillConvexPoly(canvas, hull, 255)
            if not text:
                continue

    for item in found:
        item['quad'] = item['quad'] / scale
        item['cx'] = item['cx'] / scale
    found.sort(key=lambda item: item['cx'])
    return found


def expand_quad(quad: np.ndarray, margin: float = 0.12) -> np.ndarray:
    """把四边形围绕中心放大一点，给二维码留出安静区。"""
    pts = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    center = pts.mean(axis=0)
    return (center + (pts - center) * (1.0 + margin)).astype(np.float32)


def decode_marker(rect: np.ndarray) -> str:
    """尽量把整流后的方块解成文本，解不出来返回 ''。

    依次尝试：QRCodeDetector（原尺寸/放大/补安静区）-> 可选 pyzbar。
    这是保留的可选标记解码工具；分拣区没有要求识别黑色标准码。
    """
    if rect is None or not rect.size:
        return ''
    gray = rect if rect.ndim == 2 else cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    detector = cv2.QRCodeDetector()
    for attempt in (gray, cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)):
        for pad in (0, 12):
            img = cv2.copyMakeBorder(attempt, pad, pad, pad, pad,
                                     cv2.BORDER_CONSTANT, value=255) if pad else attempt
            try:
                text, _, _ = detector.detectAndDecode(img)
            except cv2.error:
                text = ''
            if text:
                return text.strip()
        try:                                    # 部分 OpenCV 版本只有 multi 接口能出结果
            ok, texts, _, _ = detector.detectAndDecodeMulti(attempt)
            if ok and any(texts):
                return next(t for t in texts if t).strip()
        except (cv2.error, AttributeError):
            pass
    try:
        from pyzbar import pyzbar         # 可选依赖
        for symbol in pyzbar.decode(gray):
            if symbol.data:
                return symbol.data.decode('utf-8', 'ignore').strip()
    except Exception:
        pass
    return ''


def marker_signature(rect: np.ndarray, grid: int = SIG_GRID) -> str:
    """Use the normalized white glyph inside the black floor marker."""
    if rect is None or not rect.size:
        return ''
    gray = rect if rect.ndim == 2 else cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    # Both reference photos and detected crops can still contain perspective.
    # Reuse the black marker boundary before comparing their white glyphs.
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    _, dark = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boundary = max(contours, key=cv2.contourArea, default=None)
    if boundary is not None:
        quad = cv2.approxPolyDP(boundary, .018 * cv2.arcLength(boundary, True), True)
        if len(quad) == 4 and cv2.isContourConvex(quad) and cv2.contourArea(quad) > .48 * gray.size:
            gray = _warp(gray, quad)
    height, width = gray.shape
    margin_x, margin_y = round(.19 * width), round(.19 * height)
    inner = gray[margin_y:height-margin_y, margin_x:width-margin_x]
    _, binary = cv2.threshold(inner, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
    keep = stats[:, cv2.CC_STAT_AREA] >= max(5, inner.size * .009)
    keep[0] = False
    glyph = keep[labels].astype(np.uint8) * 255
    points = cv2.findNonZero(glyph)
    if points is None:
        return ''
    left, top, width, height = cv2.boundingRect(points)
    scale = (grid - 2) / max(width, height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    shape = cv2.resize(glyph[top:top+height, left:left+width], size, interpolation=cv2.INTER_AREA)
    small = np.zeros((grid, grid), dtype=np.uint8)
    x, y = (grid - size[0]) // 2, (grid - size[1]) // 2
    small[y:y+size[1], x:x+size[0]] = shape
    bits = 0
    for value in small.ravel():
        bits = (bits << 1) | (1 if value > 127 else 0)
    return '%0*x' % (grid * grid // 4, bits)


def signature_distance(a: str, b: str) -> int:
    """两个指纹的汉明距离；长度不一致返回一个很大的数。"""
    if not a or not b or len(a) != len(b):
        return 10 ** 6
    return bin(int(a, 16) ^ int(b, 16)).count('1')


def load_marker_library(path: Optional[str]) -> Dict[str, str]:
    """可选：预先把每个区二维码的照片存成 <区>.png / <区>.jpg，用指纹做匹配。

    目录形如 zone_markers/B.png、zone_markers/C.png、zone_markers/D.png。
    有它就多一条"真的认出是哪个区"的路；没有就只按位置约定，不影响结果正确性。
    """
    library: Dict[str, str] = {}
    if not path or not os.path.isdir(path):
        return library
    for name in sorted(os.listdir(path)):
        stem, ext = os.path.splitext(name)
        if ext.lower() not in ('.png', '.jpg', '.jpeg', '.bmp'):
            continue
        zone = stem.strip().upper()[:1]
        if zone not in ('B', 'C', 'D'):
            continue
        img = imread(os.path.join(path, name))
        if img is None:
            continue
        if img.shape[0] != MARKER_RECT or img.shape[1] != MARKER_RECT:
            found = find_markers(img)          # 给的是现场照片：抠出里面那个码
            if not found:
                print('zone_markers/%s 里没找到方块标记，跳过' % name)
                continue
            img = max(found, key=lambda item: item['side'])['rect']
        library[zone] = marker_signature(img)
    return library


def _decode_qr_multi(image: np.ndarray, work_width: int = 1400) -> List[dict]:
    """整图多码检测。标准二维码用这条路，比"找黑方块"可靠得多。

    返回 [{'cx': float, 'quad': 4x2(原图坐标), 'text': str}, ...]，按 cx 排好。
    """
    scale = min(1.0, work_width / float(image.shape[1]))
    work = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else image
    gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY) if work.ndim == 3 else work
    detector = cv2.QRCodeDetector()
    found: List[dict] = []
    try:
        ok, texts, points, _ = detector.detectAndDecodeMulti(gray)
    except (cv2.error, AttributeError):
        return []
    if not ok or points is None:
        return []
    for text, quad in zip(texts, points):
        quad = np.asarray(quad, dtype=np.float32).reshape(4, 2) / scale
        found.append({'cx': float(quad[:, 0].mean()), 'quad': quad, 'text': (text or '').strip()})
    found.sort(key=lambda item: item['cx'])
    return found


def read_zone_markers(image: np.ndarray, library: Optional[Dict[str, str]] = None,
                      max_distance: int = 60, signature_margin: int = 12,
                      min_signature_side: float = 40.0
                      ) -> Tuple[List[Optional[str]], List[dict]]:
    """读出三个区标记（左->右）。返回 (zone_or_None 列表, 调试信息列表)。

    顺序：
      1. 标准二维码多码检测（真二维码走这条）；
      2. 黑方块定位 + 整流解码（自制标记、看不清的码）；
      3. 都解不出就返回 None，交给位置约定。
    """
    qr_items = _decode_qr_multi(image)
    markers: List[dict] = []
    if len(qr_items) >= 2:
        for item in qr_items:
            rect = _warp(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image,
                         item['quad'])
            markers.append({'cx': item['cx'], 'quad': item['quad'], 'side': 0.0,
                            'text': item['text'], 'rect': rect, 'rect_wide': rect})
    if len(markers) != 3:
        markers = []
        candidates = []
        if library:
            # Prefer the row on the floor in the stopped board camera view.
            # The full frame can also contain dark clock hands and board supports.
            top, bottom = round(.37 * image.shape[0]), round(.80 * image.shape[0])
            candidates = find_markers(image[top:bottom], min_side_ratio=min_signature_side / image.shape[1])
            if len(candidates) == 3:
                for marker in candidates:
                    marker['quad'][:, 1] += top
            else:
                candidates = []
        for marker in candidates or find_markers(image):
            text = decode_marker(marker['rect'])
            if not text and marker.get('rect_wide') is not None:
                text = decode_marker(marker['rect_wide'])
            rect = marker['rect']
            markers.append({'cx': marker['cx'], 'quad': marker['quad'], 'side': marker['side'],
                            'text': text, 'rect': rect, 'rect_wide': marker.get('rect_wide')})

    zones: List[Optional[str]] = []
    debug: List[dict] = []
    for marker in markers:
        text = marker.get('text') or ''
        signature = marker_signature(marker['rect'])
        zone = None
        source = 'none'
        if text:
            zone = normalize_zone(text)
            source = 'text' if zone else 'text-unparsed'
        if zone is None and library and marker['side'] < min_signature_side:
            # Upsampling a smaller observation does not restore its glyph.
            source = 'signature-skipped(side=%.0f)' % marker['side']
        if zone is None and library and marker['side'] >= min_signature_side:
            scored = sorted((signature_distance(signature, ref), z) for z, ref in library.items())
            if scored and scored[0][0] <= max_distance:
                runner_up = scored[1][0] if len(scored) > 1 else 10 ** 6
                # 必须"明显比第二名像"，否则宁可不认：认错区比不认要糟
                if runner_up - scored[0][0] >= signature_margin:
                    zone, source = scored[0][1], 'signature(%d)' % scored[0][0]
                else:
                    source = 'signature-ambiguous(%d/%d)' % (scored[0][0], runner_up)
        zones.append(zone)
        debug.append({'cx': marker['cx'], 'side': marker['side'], 'text': text,
                      'zone': zone, 'source': source, 'quad': marker['quad'],
                      'signature': signature})
    return zones, debug
    return zones, debug


# --------------------------------------------------------------------------- #
# 图案（方块顶面印的图）
# --------------------------------------------------------------------------- #
DEFAULT_LABELS = ('clock', 'teddybear', 'banana', 'cake')


def detect_sorting_target(image, templates, target='banana'):
    """Find multiple banana faces. Scores are shape similarities, not probabilities."""
    if target != 'banana':
        raise ValueError('currently only --target banana is supported')
    if not templates.get('banana'):
        raise ValueError('banana template missing')
    refs = _yellow_contours(templates['banana'][0])
    if not refs:
        raise ValueError('banana template has no yellow contour')
    ref = max(refs, key=cv2.contourArea)
    height, width = image.shape[:2]
    found = []
    for contour in _yellow_contours(image, saturation_min=55):
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if not (w >= 45 and h >= 28 and area >= 500 and x > 1 and y > 1
                and x + w < width - 1 and y + h < height - 1):
            continue
        distance = float(cv2.matchShapes(contour, ref, cv2.CONTOURS_MATCH_I1, 0))
        if not np.isfinite(distance):
            continue
        partial = distance > 0.25
        # A narrow, perspective-flattened yellow face may be partially covered.
        # Keep it explicitly tentative; it is not an independently verified block.
        if distance > 0.95 or (partial and (h / float(w) > 0.55 or w < 65)):
            continue
        found.append({'label': target, 'score': 1.0 / (1.0 + distance),
                      'source': 'yellow_shape', 'partial': partial,
                      'cx': x + w / 2.0, 'cy': y + h / 2.0,
                      'w': float(w), 'h': float(h), 'quad': None})
    found.sort(key=lambda d: (d['cy'], d['cx']))
    return found


def sorting_support_plane(depth_m, camera_matrix):
    """Estimate a dominant depth plane; callers must treat it as a candidate."""
    h, w = depth_m.shape
    yy, xx = np.mgrid[0:h:5, 0:w:5]
    z = depth_m[::5, ::5]
    valid = np.isfinite(z) & (z > 0.1) & (z < 2.0)
    k = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('invalid camera intrinsics')
    points = np.column_stack(((xx[valid]-k[0, 2])*z[valid]/k[0, 0],
                              (yy[valid]-k[1, 2])*z[valid]/k[1, 1], z[valid]))
    if len(points) < 500:
        return None
    rng = np.random.RandomState(4)
    best = None
    for _ in range(100):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a, c-a)
        length = np.linalg.norm(n)
        if length < 1.e-8:
            continue
        n /= length
        if abs(n[2]) < .5:
            continue
        offset = float(n @ a)
        inliers = np.abs(points @ n - offset) < .008
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = (count, inliers)
    if best is None or best[0] < max(300, int(len(points)*.20)):
        return None
    selected = points[best[1]]
    centre = np.median(selected, axis=0)
    _, _, vh = np.linalg.svd(selected-centre, full_matrices=False)
    normal = vh[-1]
    offset = float(normal @ centre)
    if offset < 0:
        normal, offset = -normal, -offset
    return {'normal': normal, 'offset_m': offset, 'inliers': int(best[0]),
            'samples': int(len(points))}


def sorting_target_height(det, depth_m, camera_matrix, plane):
    """Median signed distance to candidate support plane; side faces are unknown."""
    if plane is None or det.get('partial'):
        return None, None
    x, y = int(round(det['cx'])), int(round(det['cy']))
    patch = depth_m[max(0, y-4):y+5, max(0, x-4):x+5]
    valid = patch[np.isfinite(patch) & (patch > .1) & (patch < 2.)]
    if len(valid) < 15:
        return None, None
    z = float(np.median(valid))
    k = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    point = np.array([(x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z])
    height = float(plane['offset_m'] - plane['normal'] @ point)
    layer = 1 if .07 <= height <= .13 else 2 if .17 <= height <= .23 else None
    return height, layer


def load_templates(template_dir: str, labels: Sequence[str] = DEFAULT_LABELS) -> Dict[str, List[np.ndarray]]:
    """按 <label>.png 读取模板，一个 label 可以有多个视角。"""
    templates: Dict[str, List[np.ndarray]] = {}
    if not os.path.isdir(template_dir):
        raise IOError('模板目录不存在: %s' % template_dir)
    for name in sorted(os.listdir(template_dir)):
        stem, ext = os.path.splitext(name)
        if ext.lower() not in ('.png', '.jpg', '.jpeg', '.bmp'):
            continue
        label = stem.split('_rule')[0].split('_')[0]
        if label not in labels:
            continue
        img = imread(os.path.join(template_dir, name))
        if img is not None and img.size:
            templates.setdefault(label, []).append(img)
    return templates


def _yellow_contours(image: np.ndarray, saturation_min: int = 80) -> list:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([15, saturation_min, 70], np.uint8),
                       np.array([40, 255, 255], np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def _make_detector(prefer: str = 'auto'):
    """优先 SIFT：印刷图案是低纹理的平涂画，ORB 在 banana 模板上只剩 11 个特征点。

    机器人上是 Ubuntu 20.04 + OpenCV 4.2，SIFT 还在专利期、没有 cv2.SIFT_create，
    只能退回 ORB；缺的那类图案由 _template_detect() 补。prefer='orb' 用来在本地
    复现机器人的配置，别拿本机的 SIFT 结果当成现场结果。
    """
    if prefer in ('auto', 'sift') and hasattr(cv2, 'SIFT_create'):
        try:
            return cv2.SIFT_create(nfeatures=4000), cv2.BFMatcher(cv2.NORM_L2), 0.75, 'sift'
        except cv2.error:
            pass
    return cv2.ORB_create(nfeatures=4000), cv2.BFMatcher(cv2.NORM_HAMMING), 0.75, 'orb'


def _feature_detect(image: np.ndarray, templates: Dict[str, List[np.ndarray]],
                    min_inliers: int = 10, min_ratio: float = 0.45,
                    max_per_label: int = 3, prefer: str = 'auto') -> List[dict]:
    """特征点 + 单应性，逐个模板反复 RANSAC，允许同一种图案出现多次。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector, matcher, ratio, source = _make_detector(prefer)
    scene_keys, scene_desc = detector.detectAndCompute(gray, None)
    if scene_desc is None or len(scene_keys) < 8:
        return []
    hscene, wscene = gray.shape
    out: List[dict] = []

    for label, images in templates.items():
        for template in images:
            tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            ref_keys, ref_desc = detector.detectAndCompute(tpl_gray, None)
            if ref_desc is None or len(ref_keys) < 8:
                continue
            pairs = matcher.knnMatch(ref_desc, scene_desc, k=2)
            matches = [p[0] for p in pairs if len(p) == 2 and p[0].distance < ratio * p[1].distance]
            th, tw = tpl_gray.shape
            corners = np.float32([[0, 0], [tw, 0], [tw, th], [0, th]]).reshape(-1, 1, 2)
            for _ in range(max_per_label):
                if len(matches) < min_inliers:
                    break
                src = np.float32([ref_keys[m.queryIdx].pt for m in matches])
                dst = np.float32([scene_keys[m.trainIdx].pt for m in matches])
                matrix, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
                if matrix is None or mask is None or not np.isfinite(matrix).all():
                    break
                mask = mask.ravel()
                inliers = int(mask.sum())
                score = inliers / float(len(matches))
                if inliers < min_inliers or score < min_ratio:
                    break
                quad = cv2.perspectiveTransform(corners, matrix).reshape(-1, 2)
                matches = [m for m, keep in zip(matches, mask.astype(bool)) if not keep]
                if not np.isfinite(quad).all() or not cv2.isContourConvex(quad.astype(np.float32)):
                    continue
                area = abs(cv2.contourArea(quad.astype(np.float32)))
                if area < hscene * wscene * 0.001 or area > hscene * wscene * 0.45:
                    continue
                if quad[:, 0].min() < 0 or quad[:, 1].min() < 0 or \
                        quad[:, 0].max() >= wscene or quad[:, 1].max() >= hscene:
                    continue
                x1, y1 = quad.min(axis=0)
                x2, y2 = quad.max(axis=0)
                out.append({'label': label, 'score': score, 'source': source,
                            'cx': float((x1 + x2) / 2), 'cy': float((y1 + y2) / 2),
                            'w': float(x2 - x1), 'h': float(y2 - y1), 'quad': quad})
    return out


def _shape_detect(image: np.ndarray, templates: Dict[str, List[np.ndarray]],
                  max_shape_distance: float = 0.25) -> List[dict]:
    """香蕉这种"大面积纯色、几乎没有纹理"的图案，特征点太少，改用黄色轮廓形状匹配。"""
    if 'banana' not in templates:
        return []
    refs = []
    for template in templates['banana']:
        contours = _yellow_contours(template)
        if contours:
            refs.append(max(contours, key=cv2.contourArea))
    if not refs:
        return []
    h, w = image.shape[:2]
    out: List[dict] = []
    for contour in _yellow_contours(image):
        area = cv2.contourArea(contour)
        if not h * w * 0.0005 <= area <= h * w * 0.05:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue
        distance = min(cv2.matchShapes(contour, ref, cv2.CONTOURS_MATCH_I1, 0) for ref in refs)
        if not np.isfinite(distance) or distance > max_shape_distance:
            continue
        out.append({'label': 'banana', 'score': float(1.0 / (1.0 + distance)), 'source': 'shape',
                    'cx': float(x + bw / 2.0), 'cy': float(y + bh / 2.0),
                    'w': float(bw), 'h': float(bh), 'quad': None})
    return out


def _coarse_ncc(a: np.ndarray, b: np.ndarray, size: int = 32) -> float:
    """把两幅图都压到 size*size 再算归一化相关。

    为什么不用原尺寸的 matchTemplate：相机 640x480 上图案只有几十像素，
    细节（钟表的数字、泰迪熊的毛线）全糊在一起，原尺寸 NCC 只有 0.52，
    而正确位置明明是对的；压到 32x32 后细节差异被平均掉，正确类别能到 0.63，
    错类别基本为负，区分度一下就出来了。
    """
    if a is None or b is None or not a.size or not b.size:
        return -1.0
    first = cv2.resize(a, (size, size), interpolation=cv2.INTER_AREA).astype(np.float32)
    second = cv2.resize(b, (size, size), interpolation=cv2.INTER_AREA).astype(np.float32)
    first -= first.mean()
    second -= second.mean()
    denom = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denom < 1e-6:
        return -1.0
    return float(first.ravel() @ second.ravel() / denom)


def _verify_labels(image: np.ndarray, detections: List[dict],
                   templates: Dict[str, List[np.ndarray]],
                   min_score: float = 0.40) -> List[dict]:
    """逐框复核类别：裁剪 -> 32x32 -> 和四个模板分别比相关，谁高算谁。

    这一步是"宁可改判也不要错判"：卡通图案在低分辨率下互相很像
    （圆钟表 vs 圆蛋糕），特征法和模板法都可能看错，但把框抠出来直接比
    整块图案的明暗布局，正确类别通常能领先 0.2 以上。
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    out: List[dict] = []
    for det in detections:
        x1, y1, x2, y2 = (int(round(v)) for v in _box_of(det))
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(gray.shape[1], x2), min(gray.shape[0], y2)
        if x2 - x1 < 12 or y2 - y1 < 12:
            continue
        crop = gray[y1:y2, x1:x2]
        scores = {}
        for label, images in templates.items():
            scores[label] = max(_coarse_ncc(crop, cv2.cvtColor(t, cv2.COLOR_BGR2GRAY))
                                for t in images)
        best = max(scores, key=scores.get)
        if scores[best] < min_score:
            continue                       # 四个都不像：这个框本身就是噪声
        item = dict(det)
        if best != item['label']:
            item['relabelled_from'] = item['label']
        item['label'] = best
        item['verify'] = round(scores[best], 3)
        item['verify_all'] = {k: round(v, 3) for k, v in scores.items()}
        out.append(item)
    return out


def _template_detect(image: np.ndarray, templates: Dict[str, List[np.ndarray]],
                     labels: Sequence[str], min_score: float = 0.50,
                     scales: Sequence[float] = tuple(round(v, 2) for v in
                                                    np.arange(0.35, 1.41, 0.05))) -> List[dict]:
    """补漏用的多尺度模板匹配（归一化互相关）。

    为什么需要它：机器人是 OpenCV 4.2，没有 SIFT；ORB 在 banana 模板上只有 11 个特征点，
    一个都配不上，于是"三类里少一类"整场就配错区。平涂印刷图用灰度 NCC 反而很稳，
    代价是慢（一张图 2~3 秒），所以只在特征法漏掉某个类别时才跑。
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hscene, wscene = gray.shape
    out: List[dict] = []
    for label in labels:
        best = None
        for template in templates.get(label, []):
            tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            for scale in scales:
                size = (max(8, int(tpl_gray.shape[1] * scale)), max(8, int(tpl_gray.shape[0] * scale)))
                if size[0] >= wscene or size[1] >= hscene:
                    continue
                resized = cv2.resize(tpl_gray, size, interpolation=cv2.INTER_AREA)
                try:
                    response = cv2.matchTemplate(gray, resized, cv2.TM_CCOEFF_NORMED)
                except cv2.error:
                    continue
                _, score, _, loc = cv2.minMaxLoc(response)
                if best is None or score > best['score']:
                    best = {'label': label, 'score': float(score), 'source': 'tpl',
                            'cx': float(loc[0] + size[0] / 2.0), 'cy': float(loc[1] + size[1] / 2.0),
                            'w': float(size[0]), 'h': float(size[1]), 'quad': None}
        if best is not None and best['score'] >= min_score:
            out.append(best)
    return out


def _iou(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _box_of(det: dict) -> Tuple[float, float, float, float]:
    return (det['cx'] - det['w'] / 2, det['cy'] - det['h'] / 2,
            det['cx'] + det['w'] / 2, det['cy'] + det['h'] / 2)


def detect_patterns(image: np.ndarray, templates: Dict[str, List[np.ndarray]],
                    min_inliers: int = 10, min_ratio: float = 0.45,
                    iou_suppress: float = 0.30, work_width: int = 1400,
                    prefer: str = 'auto', template_min_score: float = 0.50) -> List[dict]:
    """找出画面里所有图案，按 x 从左到右排序。

    去重规则：同一位置只保留分数最高的那个（不管它被认成什么类别）——
    宁可留下一个高分的判断，也不要留两个互相矛盾的。

    先把大图缩到 work_width 再提特征：4000 个 ORB 点摊在一张 1200 万像素的图上，
    落到图案上只剩个位数；缩到 1400 宽特征密度才够。
    """
    if image is None or not image.size:
        raise ValueError('空图像')
    scale = min(1.0, work_width / float(image.shape[1]))
    work = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else image
    candidates = _feature_detect(work, templates, min_inliers, min_ratio, prefer=prefer)
    candidates += _shape_detect(work, templates)
    found_labels = {c['label'] for c in candidates}
    if len(found_labels) < 3 or len(candidates) < 3:
        # 缺类别就四个模板全搜一遍（不只是缺的那两个）：漏检常常是"认错成别的"，
        # 只补"缺失的类别"会把正确位置让给一个高分错类
        candidates += _template_detect(work, templates, list(templates),
                                       min(0.45, template_min_score))
    if not candidates:
        return []

    candidates = _verify_labels(work, candidates, templates)
    if not candidates:
        return []

    kept: List[dict] = []
    for det in sorted(candidates, key=lambda d: d['verify'], reverse=True):
        if any(_iou(_box_of(det), _box_of(other)) > iou_suppress for other in kept):
            continue
        kept.append(det)
        if len(kept) == 3:
            break
    if scale < 1:
        for det in kept:
            for key in ('cx', 'cy', 'w', 'h'):
                det[key] /= scale
            if det.get('quad') is not None:
                det['quad'] = det['quad'] / scale
    kept.sort(key=lambda d: d['cx'])
    return kept


def read_patterns(image: np.ndarray, templates: Dict[str, List[np.ndarray]],
                  expected: int = 3, **kwargs) -> Tuple[List[str], List[dict], List[str]]:
    """读出工作台图案，左->右。返回 (代号列表, 检测详情, 警告列表)。

    严格按 expect 个返回：多出来的截掉并告警，少了的用 'unknown' 占位并告警，
    保证上层永远拿到 3 个槽位，配对时下标不会错位。
    """
    detections = detect_patterns(image, templates, **kwargs)
    warnings: List[str] = []
    labels = [d['label'] for d in detections]
    if len(labels) > expected:
        warnings.append('检测到 %d 个图案（%s），只取置信度最高的 %d 个'
                        % (len(labels), labels, expected))
        best = sorted(detections, key=lambda d: -d['score'])[:expected]
        labels = [d['label'] for d in sorted(best, key=lambda d: d['cx'])]
    while len(labels) < expected:
        labels.append('unknown')
        warnings.append('只认出 %d 个图案，缺失槽位记为 unknown' % (len(labels) - 1))
    if len(set(labels)) != len(labels):
        warnings.append('图案代号有重复：%s（同一种图案被识别了两次）' % labels)
    return labels, detections, warnings
