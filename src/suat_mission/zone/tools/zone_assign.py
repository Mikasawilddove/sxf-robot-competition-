#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""历史分区工具及分拣视觉测试入口。

分拣测试前，由外部先将机械臂移至最左侧并将机器人移动到四个分拣观察点之一。
一层是 3×3 个边长约 10 cm 的方块，中央二层是蛋糕；其他三类为小熊、时钟、
香蕉各三个。

目标需求是抓取同一帧后，逐次针对一种类别识别，最后在该帧叠加四类结果，
展示检测数量、类别、框和各算法匹配分数；识别率需有人工真值及统计口径。
当前 --target 一次只显示所选类别，没有实现四类结果同帧累积；--target 分支
不移动机械臂。旧 --camera 等分类入口仍可能调用机械臂移动及标记读取逻辑，
不能将这段注释当作旧入口行为已改变。
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from zone_mapping import UNKNOWN, format_report, map_patterns_to_zones   # noqa: E402


def _split(text, cast=str):
    return [cast(piece.strip()) for piece in text.split(',') if piece.strip()]


def _crop(image, box):
    if not box:
        return image
    x0, y0, x1, y1 = _split(box, int)
    return image[y0:y1, x0:x1]


def _annotate(image, markers, detections, result=None):
    import cv2
    vis = image.copy()
    scale = max(0.6, image.shape[1] / 2600.0)
    thick = max(2, int(round(image.shape[1] / 1400.0)))
    for index, marker in enumerate(markers):
        quad = marker['quad'].astype(int)
        color = (0, 0, 255) if marker.get('zone') is None else (0, 160, 0)
        cv2.polylines(vis, [quad], True, color, thick)
        label = marker.get('zone') or '?'        # 解不出也认不出时只写个问号，别糊一串指纹
        cv2.putText(vis, '%d:%s' % (index + 1, label),
                    (int(quad[:, 0].min()), int(quad[:, 1].min()) - int(12 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)
    zone_of = result.zone_of if result is not None else {}
    for det in detections:
        if det.get('quad') is not None:
            cv2.polylines(vis, [det['quad'].astype(int)], True, (255, 0, 0), thick)
        else:                                   # 模板匹配没有四边形，就画正框
            x1, y1 = int(det['cx'] - det['w'] / 2), int(det['cy'] - det['h'] / 2)
            cv2.rectangle(vis, (x1, y1), (x1 + int(det['w']), y1 + int(det['h'])), (255, 0, 0), thick)
        zone = zone_of.get(det['label'])
        # 只写 ASCII：机器人上的 cv2.putText 不支持中文，会渲染成 ???，所以区号直接写字母
        text = '%s -> %s' % (det['label'], zone) if zone else det['label']
        cv2.putText(vis, text, (int(det['cx'] - det['w'] / 2), int(det['cy'] - det['h'] / 2) - int(14 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 0, 0), thick)
    return vis


def _park_arm(pose_text='', quiet=False):
    """抓帧前把机械臂挪到一侧。挡着相机的臂会让三个码只检出两个。"""
    try:
        from zone_arm import PARK_POSE, park
    except ImportError as exc:
        print('挪臂需要 zone_arm.py 和 ROS：%s' % exc, file=sys.stderr)
        return False
    pose = PARK_POSE
    if pose_text:
        try:
            pose = tuple(float(v) for v in pose_text.split(','))
        except ValueError:
            print('--park-pose 需要 x,y,z 三个数', file=sys.stderr)
            return False
        if len(pose) != 3:
            print('--park-pose 需要 x,y,z 三个数', file=sys.stderr)
            return False
    if not quiet:
        print('先把机械臂挪到一侧（别挡住相机）...', file=sys.stderr)
    return park(pose)


def _friendly_parser():
    """参数被粘贴拆断时给一句人话提示，而不是只丢一段 usage。"""

    class _Parser(argparse.ArgumentParser):
        def error(self, message):
            if 'unrecognized arguments' in message or 'expected one argument' in message:
                sys.stderr.write(
                    '\n提示: 命令很可能在粘贴时被换行/回车拆断了（--scene 变成 --sc + ene）。\n'
                    '      现场请用最短的写法：\n'
                    '        python3 tools/zone_assign.py --camera\n'
                    '      或用位置参数：\n'
                    '        python3 tools/zone_assign.py /tmp/bench.jpg\n')
            argparse.ArgumentParser.error(self, message)

    return _Parser(description='把工作台上的三个图案映射到 B/C/D 三个收取区')


def _sorting_target_snapshot(args):
    """One synchronized RGB-D snapshot; never commands the arm."""
    import threading
    import cv2
    import numpy as np
    import message_filters
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image
    from zone_vision import (detect_sorting_target, load_templates,
                             sorting_support_plane, sorting_target_height)

    if not rospy.core.is_initialized():
        rospy.init_node('sorting_target_snapshot', anonymous=True, disable_signals=True)
    received = threading.Event()
    frames = []
    def on_frame(color, depth, info):
        if not frames:
            frames.append((color, depth, info))
            received.set()
    color_sub = message_filters.Subscriber(args.ros_topic, Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub, info_sub], 8, .12)
    sync.registerCallback(on_frame)
    if not received.wait(10):
        print('[识别] 失败：10 秒内未收到同步的彩色图、对齐深度和相机参数', file=sys.stderr)
        return 3
    color, depth_msg, info = frames[0]
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(color, 'bgr8')
    depth = bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(float)
    if depth_msg.encoding == '16UC1':
        depth *= .001
    elif depth_msg.encoding != '32FC1':
        print('[定位] 失败：不支持深度编码 %s' % depth_msg.encoding, file=sys.stderr)
        return 3
    if depth.shape != image.shape[:2]:
        print('[定位] 失败：对齐深度和图像尺寸不一致', file=sys.stderr)
        return 3
    templates = load_templates(args.templates)
    detections = detect_sorting_target(image, templates, args.target)
    plane = sorting_support_plane(depth, info.K)
    visual = image.copy()
    print('[相机] 单帧时间：%.3f；彩色/深度时间差：%.3f s' %
          (color.header.stamp.to_sec(), abs(color.header.stamp.to_sec()-depth_msg.header.stamp.to_sec())))
    print('[识别] 目标：%s；方法：黄色轮廓与模板形状匹配' % args.target)
    print('[识别] 完整图案：%d；遮挡/侧面候选：%d' %
          (sum(not d['partial'] for d in detections), sum(d['partial'] for d in detections)))
    print('[识别] 分数是形状相似度，不是经过校准的识别概率或召回率')
    if plane is None:
        print('[定位] 未找到足够支持的平面；层数未知')
    else:
        print('[定位] 支撑平面候选：%d/%d 深度点；每层高度 0.10 m，需现场核对' %
              (plane['inliers'], plane['samples']))
    for index, det in enumerate(detections, 1):
        height, layer = sorting_target_height(det, depth, info.K, plane)
        x1 = int(det['cx']-det['w']/2); y1 = int(det['cy']-det['h']/2)
        x2 = int(det['cx']+det['w']/2); y2 = int(det['cy']+det['h']/2)
        kind = '遮挡候选' if det['partial'] else '完整图案'
        level = str(layer) if layer else '?'
        print('[识别] %d: %s，%s，中心=(%d,%d)，相似度=%.3f，层=%s，高度=%s' %
              (index, det['label'], kind, det['cx'], det['cy'], det['score'], level,
               '未知' if height is None else '%.3f m' % height))
        box_color = (0, 165, 255) if det['partial'] else (0, 220, 0)
        cv2.rectangle(visual, (x1, y1), (x2, y2), box_color, 2)
        label = '%s%s %.2f L%s' % (det['label'], '?' if det['partial'] else '', det['score'], level)
        cv2.putText(visual, label, (x1, max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .5, box_color, 2)
    print('[识别] 说明：候选可能是误检或同一方块的另一个面；需人工核对，不能直接作为抓取依据。', flush=True)
    pub = rospy.Publisher('/suat/perception/target_image', Image, queue_size=1, latch=True)
    annotated = bridge.cv2_to_imgmsg(visual, 'bgr8'); annotated.header = color.header
    pub.publish(annotated)
    if os.environ.get('DISPLAY') and not args.no_window:
        print('[画面] 已打开标注图；按任意键关闭。', flush=True)
        cv2.imshow('Sorting target snapshot', visual)
        cv2.waitKey(0)
        cv2.destroyWindow('Sorting target snapshot')
    else:
        print('[画面] 无 DISPLAY 或已指定 --no-window；只输出识别结果。', flush=True)
        rospy.sleep(1.)
    return 0


def main(argv=None):
    parser = _friendly_parser()
    parser.add_argument('path', nargs='?', help='位置参数写法：图片路径（等价于 --scene）')
    parser.add_argument('--scene', help='同时含二维码和图案的一张照片')
    parser.add_argument('--camera', action='store_true',
                        help='当场抓一帧相机图当 --scene 用（比赛现场用这个）')
    parser.add_argument('--camera-marker', action='store_true',
                        help='在分类区抓一帧，只用来读三个二维码')
    parser.add_argument('--camera-pattern', action='store_true',
                        help='在工作台抓一帧，只用来认三个图案')
    parser.add_argument('--ros-topic', default='/camera/color/image_raw',
                        help='相机话题，默认 /camera/color/image_raw')
    parser.add_argument('--snap', default='', metavar='OUT.jpg',
                        help='只抓一帧存下来，不做识别（调机位/裁剪用）')
    parser.add_argument('--save-frame', default='', help='把抓到的原始帧另存一份')
    parser.add_argument('--no-park', action='store_true',
                        help='抓帧前不挪机械臂（默认会先挪到一侧，别挡住相机）')
    parser.add_argument('--park-pose', default='',
                        help='避让位 x,y,z，默认贴 y 负方向一侧 (60,-170,150)')
    parser.add_argument('--usb-camera', action='store_true',
                        help='ROS 不通时允许退回本机 USB 摄像头（默认不允许，免得拿错画面）')
    parser.add_argument('--marker-image', help='只含分类区三个二维码的照片')
    parser.add_argument('--pattern-image', help='只含工作台三个图案的照片')
    parser.add_argument('--patterns', help='直接给三个图案代号，左->右，逗号分隔')
    parser.add_argument('--marker-codes', help='直接给三个二维码的解码结果，左->右，逗号分隔')
    parser.add_argument('--marker-crop', help='二维码区域 x0,y0,x1,y1')
    parser.add_argument('--pattern-crop', help='图案区域 x0,y0,x1,y1')
    parser.add_argument('--templates', default=os.path.join(os.path.dirname(__file__), '..', '_tpl'),
                        help='图案模板目录，默认 ../_tpl')
    parser.add_argument('--detector', default='auto', choices=('auto', 'sift', 'orb'),
                        help='图案特征检测器；机器人是 OpenCV 4.2 没有 SIFT，用 orb 可复现现场表现')
    parser.add_argument('--marker-library', default='',
                        help='可选：zone_markers/B.png、C.png、D.png，用于按指纹认区')
    parser.add_argument('--save', default='', help='保存带框的调试图')
    parser.add_argument('--json', action='store_true', help='额外输出 JSON')
    parser.add_argument('--quiet', action='store_true', help='只输出结论行')
    parser.add_argument('--target', choices=('banana',), help='只识别指定分拣目标，目前支持 banana')
    parser.add_argument('--no-window', action='store_true', help='目标识别时不打开标注窗口')
    args = parser.parse_args(argv)

    if args.target:
        return _sorting_target_snapshot(args)

    patterns = _split(args.patterns) if args.patterns else None
    marker_texts = _split(args.marker_codes) if args.marker_codes is not None else None
    zone_source = 'qr'
    detections, markers = [], []
    scene_image = None

    if not args.scene and args.path:                 # 位置参数写法
        args.scene = args.path

    if (args.snap or args.camera or args.camera_marker or args.camera_pattern) \
            and not args.no_park:
        _park_arm(args.park_pose, args.quiet)

    if args.snap:                                    # 只抓一帧，方便调机位/裁剪
        try:
            from zone_vision import grab_frame, imwrite
        except ImportError as exc:
            print('抓帧需要 OpenCV：%s' % exc, file=sys.stderr)
            return 2
        try:
            frame = grab_frame(args.ros_topic, allow_usb=args.usb_camera)
        except RuntimeError as exc:
            print('抓帧失败: %s' % exc, file=sys.stderr)
            return 3
        print('抓帧: %dx%d  话题 %s' % (frame.shape[1], frame.shape[0], args.ros_topic))
        print('已保存: %s' % args.snap if imwrite(args.snap, frame) else '保存失败: %s' % args.snap)
        return 0

    if patterns is None and not (args.pattern_image or args.scene or args.camera
                                 or args.camera_pattern):
        print('图案识别需要 --camera / --scene / --pattern-image（也可直接用 --patterns 给结果）',
              file=sys.stderr)
        return 2

    if patterns is None or marker_texts is None:
        try:
            from zone_vision import (grab_frame, imread, imwrite, load_marker_library,
                                     load_templates, read_patterns, read_zone_markers)
        except ImportError as exc:
            print('需要 OpenCV 才能看图：%s' % exc, file=sys.stderr)
            return 2

        def _grab(tag, save_path=''):
            try:
                frame = grab_frame(args.ros_topic, allow_usb=args.usb_camera)
            except RuntimeError as exc:
                print('抓帧失败（%s）: %s' % (tag, exc), file=sys.stderr)
                return None
            print('抓帧 %s: %dx%d  话题 %s'
                  % (tag, frame.shape[1], frame.shape[0], args.ros_topic), file=sys.stderr)
            if save_path and imwrite(save_path, frame):
                print('原始帧: %s' % save_path, file=sys.stderr)
            return frame

        if args.scene:
            if not os.path.isfile(args.scene):
                print('找不到图片: %s' % args.scene, file=sys.stderr)
                return 2
            scene_image = imread(args.scene)
            if scene_image is None:
                print('读不到图片（路径含中文？）: %s' % args.scene, file=sys.stderr)
                return 2
        if args.camera:
            scene_image = _grab('scene', args.save_frame)
            if scene_image is None:
                return 3

        pattern_image = scene_image
        marker_image = scene_image
        if args.pattern_image:
            pattern_image = imread(args.pattern_image)
            if pattern_image is None:
                print('读不到图案图: %s' % args.pattern_image, file=sys.stderr)
                return 2
        if args.marker_image:
            marker_image = imread(args.marker_image)
            if marker_image is None:
                print('读不到二维码图: %s' % args.marker_image, file=sys.stderr)
                return 2
        if args.camera_pattern:
            pattern_image = _grab('pattern', args.save_frame)
            if pattern_image is None:
                return 3
        if args.camera_marker:
            marker_image = _grab('marker', args.save_frame)
            if marker_image is None:
                return 3

        if patterns is None:
            if pattern_image is None:
                print('图案识别需要 --pattern-image / --scene / --camera', file=sys.stderr)
                return 2
            templates = load_templates(args.templates)
            patterns, detections, pattern_warnings = read_patterns(
                _crop(pattern_image, args.pattern_crop), templates, prefer=args.detector)
            for warning in pattern_warnings:
                print('图案: %s' % warning, file=sys.stderr)

        if marker_texts is None:
            if marker_image is None:
                marker_texts = []
                print('没有二维码图像，按"左->右 = B/C/D"的位置约定配对', file=sys.stderr)
            else:
                region = _crop(marker_image, args.marker_crop)
                library = load_marker_library(args.marker_library)
                _, markers = read_zone_markers(region, library)
                # 解不出文本时，指纹认出来的区也当成"读到的区"传下去
                marker_texts = [item['text'] or item.get('zone') or None for item in markers]
                if markers and not all(item['text'] for item in markers):
                    zone_source = 'fingerprint' if any(item.get('zone') for item in markers) else 'qr'
                if not args.quiet:
                    print('二维码(左->右): %s' % ', '.join(
                        '%s[%s]' % (item.get('zone') or '?', item.get('text') or '未解码')
                        for item in markers))
                if len(markers) != 3:
                    print('警告: 找到 %d 个二维码方块，不是 3 个' % len(markers), file=sys.stderr)

    result = map_patterns_to_zones(patterns, marker_texts, source=zone_source)
    if args.quiet:
        print(' | '.join('%s -> %s区' % (slot.pattern, slot.zone) for slot in result.slots))
        print('导航: ' + ' -> '.join(result.nav_order))
    else:
        print(format_report(result, '图案 -> 收取区'))
        if detections:
            print('  检测详情: ' + ', '.join(
                '%s(%.2f,%s,x=%.0f)' % (item['label'], item['score'], item['source'], item['cx'])
                for item in detections))

    if args.json:
        print(result.as_json())

    if args.save and scene_image is not None:
        if imwrite(args.save, _annotate(scene_image, markers, detections, result)):
            print('调试图: %s' % args.save)

    if any(slot.pattern == UNKNOWN for slot in result.slots):
        print('识别不完整: %s —— 画面里没有认全三块图案，这个结论不能拿去抓取'
              % ' / '.join('%s=%s' % (slot.position_name, slot.pattern) for slot in result.slots),
              file=sys.stderr)
        return 4
    return 0


if __name__ == '__main__':
    sys.exit(main())
