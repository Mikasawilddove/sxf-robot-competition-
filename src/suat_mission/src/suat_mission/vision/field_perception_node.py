#!/usr/bin/env python3
"""Production camera adapter: synchronized RGB-D + CameraInfo -> formal observations.

Read-only: this node never publishes motion, arm or pump commands.

Publishes:
  /objects                    Detection2DArray selected by explicit objects_mode
  /suat/perception/sorting_objects  full-image Detection2DArray
  /suat/perception/board_objects    board-only Detection2DArray in board mode
  /suat/perception/sorting_observations  uncalibrated per-frame diagnostic JSON
  /suat/perception/instances  std_msgs/String JSON accepted by
                              suat_mission.observations.decode_scene
  /suat/perception/status     std_msgs/String JSON diagnostics (sync, rates, rejections)

Evidence: when ~evidence_dir is set the first synced frame is written there as
  color.png / aligned_depth.png / camera_info.json / frame_meta.json.
Limits are reported honestly: an empty ~calibration_id means the instance topic
is not published (the formal consumer rejects an empty calibration_id).
"""
import argparse
import os
import json
from typing import Dict, List, Optional, Sequence, Tuple
import math
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from suat_mission.vision.perception import ALL_ITEMS, COCO_IDS, OBJECTS
from suat_mission.vision.template_matcher import TemplateMatcher
from suat_mission.vision.feature_matcher import CakeFeatureMatcher
from suat_mission.vision.board_recognition import BoardGeometry


class FieldPerception:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~aligned_depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.color_info_topic = rospy.get_param('~color_info_topic', '/camera/color/camera_info')
        self.depth_info_topic = rospy.get_param('~depth_info_topic', '/camera/aligned_depth_to_color/camera_info')
        self.sync_slop = float(rospy.get_param('~sync_slop', 0.12))
        self.calibration_id = str(rospy.get_param('~calibration_id', '')).strip()
        self.evidence_dir = str(rospy.get_param('~evidence_dir', '')).strip()

        # --- existing algorithms, reused unchanged ---
        paths = rospy.get_param('~templates', {})
        if set(paths) != set(ALL_ITEMS):
            raise ValueError('templates must provide exactly ' + ','.join(sorted(ALL_ITEMS)))
        self.templates = {}
        for label, names in paths.items():
            if not isinstance(names, list) or not names:
                raise ValueError('template paths must be nonempty lists: ' + label)
            images = []
            for name in names:
                image = cv2.imread(str(Path(str(name)).expanduser()), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('unable to read template ' + str(name))
                images.append(image)
            self.templates[label] = images
        self.method = str(rospy.get_param('~method', 'template'))
        if self.method not in ('template', 'hybrid'):
            raise ValueError('method must be template or hybrid')
        self.image_scale = float(rospy.get_param('~image_scale', 1.0))
        if not 0.25 <= self.image_scale <= 1.0:
            raise ValueError('image_scale must be between 0.25 and 1.0')
        threshold = float(rospy.get_param('~threshold', 0.6))
        scales = [float(s) * self.image_scale for s in rospy.get_param('~scales', [0.8, 1.0, 1.2])]
        rotations = [int(r) for r in rospy.get_param('~rotations', [0])]
        match_templates = {k: v for k, v in self.templates.items() if self.method != 'hybrid' or k != 'cake'}
        self.matcher = TemplateMatcher(match_templates, scales, rotations, threshold)
        self.cake_matcher = CakeFeatureMatcher(self.templates['cake']) if self.method == 'hybrid' else None
        self.threshold = threshold

        # --- optional board view geometry: normalized-board matches -> /objects ---
        geometry_path = str(rospy.get_param('~board_geometry', '')).strip()
        self.board_geometry = None
        self.board_matcher = None
        self.board_geometry_id = None
        if geometry_path:
            config = json.loads(Path(geometry_path).read_text(encoding='utf-8'))
            self.board_geometry = BoardGeometry(tuple(config['image_size']), config['corners'],
                                               tuple(config['normalized_size']), config['geometry_id'])
            self.board_geometry_id = config['geometry_id']
            board_threshold = float(rospy.get_param('~board_threshold', 0.6))
            board_scales = [float(s) for s in rospy.get_param('~board_scales', [.4, .5, .6, .7, .8, .9, 1., 1.1, 1.2])]
            self.board_matcher = TemplateMatcher({k: self.templates[k] for k in OBJECTS}, board_scales,
                                                 [0], board_threshold)

        # --- depth gating for pickup instances ---
        depth_cfg = rospy.get_param('~depth', {}) or {}
        self.depth_validate = bool(depth_cfg.get('validate', True))
        self.depth_min = float(depth_cfg.get('min_m', 0.15))
        self.depth_max = float(depth_cfg.get('max_m', 1.30))
        self.depth_window = int(depth_cfg.get('window_px', 9))

        tracking = rospy.get_param('~tracking', {}) or {}
        self.min_support = int(tracking.get('min_support_frames', 3))
        self.track_distance = float(tracking.get('track_distance_px', 35.0))
        self.track_timeout = float(tracking.get('track_timeout_s', 1.5))
        self.identity_bucket = int(tracking.get('identity_bucket_px', 24))
        publish = rospy.get_param('~publish', {}) or {}
        instance_hz = float(publish.get('instance_hz', 5.0))
        status_hz = float(publish.get('status_hz', 1.0))

        self.objects_topic = rospy.get_param('~objects_topic', '/objects')
        self.sorting_topic = rospy.get_param('~sorting_objects_topic', '/suat/perception/sorting_objects')
        self.board_topic = rospy.get_param('~board_objects_topic', '/suat/perception/board_objects')
        self.observations_topic = rospy.get_param('~sorting_observations_topic', '/suat/perception/sorting_observations')
        self.instances_topic = rospy.get_param('~instances_topic', '/suat/perception/instances')
        self.status_topic = rospy.get_param('~status_topic', '/suat/perception/status')

        self.objects_pub = rospy.Publisher(self.objects_topic, Detection2DArray, queue_size=1)
        self.sorting_pub = rospy.Publisher(self.sorting_topic, Detection2DArray, queue_size=1)
        self.board_pub = rospy.Publisher(self.board_topic, Detection2DArray, queue_size=1)
        self.observations_pub = rospy.Publisher(self.observations_topic, String, queue_size=1)
        self.instances_pub = rospy.Publisher(self.instances_topic, String, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)

        self.tracks = []
        self.scene_id = 'field-%d' % int(rospy.Time.now().to_sec())
        self.version = 0
        self.latest_header = None
        self.intrinsics = None
        self.stats = {'frames_seen': 0, 'frames_processed': 0, 'frames_rejected': 0,
                      'last_sync_skew_s': None, 'max_sync_skew_s': 0.0,
                      'last_detections': {}, 'instances_published': 0,
                      'last_error': None, 'started_ros': rospy.Time.now().to_sec(),
                      'color_encoding': None, 'depth_encoding': None,
                      'color_size': None, 'depth_size': None, 'evidence': None}
        self.evidence_saved = False

        color = message_filters.Subscriber(self.image_topic, Image)
        depth = message_filters.Subscriber(self.depth_topic, Image)
        cinfo = message_filters.Subscriber(self.color_info_topic, CameraInfo)
        dinfo = message_filters.Subscriber(self.depth_info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer([color, depth, cinfo, dinfo],
                                                                queue_size=12, slop=self.sync_slop)
        self.sync.registerCallback(self.on_frame)
        rospy.Timer(rospy.Duration(1.0 / max(instance_hz, 0.2)), self.publish_instances)
        rospy.Timer(rospy.Duration(1.0 / max(status_hz, 0.2)), self.publish_status)
        rospy.loginfo('field_perception started: board_geometry=%s calibration_id=%r',
                      self.board_geometry_id, self.calibration_id)

    # ------------------------------------------------------------------ frames
    def on_frame(self, color_msg, depth_msg, color_info, depth_info):
        self.stats['frames_seen'] += 1
        skew = abs(color_msg.header.stamp.to_sec() - depth_msg.header.stamp.to_sec())
        self.stats['last_sync_skew_s'] = round(skew, 4)
        self.stats['max_sync_skew_s'] = max(self.stats['max_sync_skew_s'], skew)
        try:
            image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            if self.image_scale != 1.0:
                image = cv2.resize(image, None, fx=self.image_scale, fy=self.image_scale,
                                   interpolation=cv2.INTER_AREA)
            depth_scale = 0.001 if depth_msg.encoding == '16UC1' else 1.0
            self.intrinsics = {'K': list(color_info.K), 'D': list(color_info.D),
                               'width': color_info.width, 'height': color_info.height,
                               'frame_id': color_info.header.frame_id}
            detections = self.detect_full_image(image)
            mode = str(rospy.get_param('~objects_mode', 'sorting')).strip().lower()
            self.stats['objects_mode'] = mode
            self.publish_objects(detections, color_msg.header, self.sorting_pub, False)
            self.publish_sorting_observations(detections, depth, depth_scale, color_msg.header)
            if mode == 'sorting':
                self.publish_objects(detections, color_msg.header)
            elif mode == 'board':
                board_detections = self.detect_board(image) or []
                self.publish_objects(board_detections, color_msg.header, self.board_pub, False)
                self.publish_objects(board_detections, color_msg.header)
            else:
                rospy.logerr_throttle(5.0, 'invalid objects_mode %r: publishing empty /objects', mode)
                self.publish_objects([], color_msg.header)
            self.update_tracks(detections, depth, depth_scale, color_msg.header.stamp.to_sec())
            self.latest_header = color_msg.header
            self.stats['frames_processed'] += 1
            self.stats['color_encoding'] = color_msg.encoding
            self.stats['depth_encoding'] = depth_msg.encoding
            self.stats['color_size'] = [color_msg.width, color_msg.height]
            self.stats['depth_size'] = [depth_msg.width, depth_msg.height]
            if not self.evidence_saved and self.evidence_dir:
                self.save_evidence(image, depth, color_info, depth_info, color_msg, depth_msg, skew)
        except (CvBridgeError, ValueError, cv2.error) as exc:
            self.stats['frames_rejected'] += 1
            self.stats['last_error'] = str(exc)
            rospy.logwarn_throttle(5.0, 'field_perception frame rejected: %s', exc)

    def detect_full_image(self, image):
        detections = self.matcher.detect(image)
        if self.cake_matcher is not None:
            detections.extend(self.cake_matcher.detect(image))
        return detections

    def detect_board(self, image):
        """Normalized board matches mapped back to original pixels (board stage only)."""
        if self.board_geometry is None or self.board_matcher is None:
            return None
        if tuple(image.shape[1::-1]) != tuple(self.board_geometry.image_size):
            return None
        normalized = self.board_geometry.normalize(image)
        matches = self.board_matcher.detect_board(normalized)
        mapped = [self.board_geometry.original_detection(d) for d in matches.get('detections', [])]
        return mapped or None

    # ---------------------------------------------------------------- publishers
    def publish_objects(self, detections, header, publisher=None, update_stats=True):
        output = Detection2DArray()
        output.header = header
        inverse = {name: number for number, name in COCO_IDS.items()}
        for detection in detections:
            item = Detection2D()
            item.header = header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = inverse[detection.label]
            hypothesis.score = float(detection.score)
            item.results = [hypothesis]
            item.bbox.center.x = float(detection.x) / self.image_scale
            item.bbox.center.y = float(detection.y) / self.image_scale
            item.bbox.size_x = float(detection.width) / self.image_scale
            item.bbox.size_y = float(detection.height) / self.image_scale
            output.detections.append(item)
        (publisher or self.objects_pub).publish(output)
        counts = {}
        for detection in detections:
            counts[detection.label] = counts.get(detection.label, 0) + 1
        if update_stats:
            self.stats['last_detections'] = counts

    def publish_sorting_observations(self, detections, depth, depth_scale, header):
        objects = []
        for detection in detections:
            x = float(detection.x) / self.image_scale
            y = float(detection.y) / self.image_scale
            depth_m = self.median_depth(depth, x, y, depth_scale)
            valid = depth_m is not None and self.depth_min <= depth_m <= self.depth_max
            objects.append({'label': detection.label, 'score': float(detection.score),
                            'center_px': [x, y],
                            'size_px': [float(detection.width) / self.image_scale,
                                        float(detection.height) / self.image_scale],
                            'depth_m': depth_m, 'depth_in_configured_range': valid})
        payload = {'stamp': header.stamp.to_sec(), 'frame_id': header.frame_id,
                   'source': 'full_image', 'purpose': 'diagnostic_only',
                   'calibration_id': self.calibration_id or None,
                   'physical_identity': 'not_verified', 'objects': objects}
        self.observations_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def median_depth(self, depth, x, y, depth_scale):
        half = max(1, self.depth_window // 2)
        xi, yi = int(round(x)), int(round(y))
        if not (half <= xi < depth.shape[1] - half and half <= yi < depth.shape[0] - half):
            return None
        patch = depth[yi - half:yi + half + 1, xi - half:xi + half + 1].astype(float) * depth_scale
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def update_tracks(self, detections, depth, depth_scale, stamp):
        now = time.monotonic()
        survivors = []
        for track in self.tracks:
            if now - track['last_monotonic'] <= self.track_timeout:
                survivors.append(track)
        self.tracks = survivors
        for detection in detections:
            if detection.label not in ALL_ITEMS or detection.score < self.threshold:
                continue
            depth_m = self.median_depth(depth, detection.x / self.image_scale,
                                        detection.y / self.image_scale, depth_scale)
            if self.depth_validate and (depth_m is None or not self.depth_min <= depth_m <= self.depth_max):
                continue
            best, best_distance = None, self.track_distance
            for track in self.tracks:
                if track['label'] != detection.label:
                    continue
                distance = math.hypot(track['x'] - detection.x, track['y'] - detection.y)
                if distance < best_distance:
                    best, best_distance = track, distance
            if best is None:
                self.tracks.append({'label': detection.label, 'x': float(detection.x),
                                    'y': float(detection.y), 'width': float(detection.width),
                                    'height': float(detection.height), 'score': float(detection.score),
                                    'count': 1, 'depth_m': depth_m,
                                    'first_stamp': stamp, 'last_stamp': stamp,
                                    'last_monotonic': now})
            else:
                count = best['count']
                best.update(x=(best['x'] * count + detection.x) / (count + 1),
                            y=(best['y'] * count + detection.y) / (count + 1),
                            width=(best['width'] * count + detection.width) / (count + 1),
                            height=(best['height'] * count + detection.height) / (count + 1),
                            score=(best['score'] * count + detection.score) / (count + 1),
                            depth_m=depth_m if depth_m is not None else best['depth_m'],
                            count=count + 1, last_stamp=stamp, last_monotonic=now)

    def publish_instances(self, _event):
        if self.latest_header is None:
            return
        if not self.calibration_id:
            self.stats['last_error'] = 'calibration_id_missing: instances not published'
            return
        objects = []
        used = set()
        for track in self.tracks:
            if track['count'] < self.min_support:
                continue
            bucket_x = int(round(track['x'] / self.identity_bucket))
            bucket_y = int(round(track['y'] / self.identity_bucket))
            object_id = '%s-px%d-%d' % (track['label'], bucket_x, bucket_y)
            suffix = 1
            while object_id in used:
                suffix += 1
                object_id = '%s-px%d-%d-%d' % (track['label'], bucket_x, bucket_y, suffix)
            used.add(object_id)
            objects.append({'label': track['label'], 'score': round(min(1.0, max(0.0, track['score'])), 4),
                            'x': round(track['x'], 2), 'y': round(track['y'], 2),
                            'width': round(track['width'], 2), 'height': round(track['height'], 2),
                            'object_id': object_id,
                            'depth_m': None if track['depth_m'] is None else round(track['depth_m'], 3),
                            'support_frames': int(track['count']),
                            'identity_basis': 'camera_position_bucket_not_verified_physical_identity'})
        if not objects:
            return
        self.version += 1
        payload = {'stamp': self.latest_header.stamp.to_sec(), 'scene_id': self.scene_id,
                   'version': self.version, 'frame_id': self.latest_header.frame_id,
                   'calibration_id': self.calibration_id,
                   'support_frames': max(o['support_frames'] for o in objects),
                   'camera': self.intrinsics, 'objects': objects}
        self.instances_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.stats['instances_published'] += 1

    def publish_status(self, _event):
        status = dict(self.stats, calibration_id=self.calibration_id,
                      board_geometry_id=self.board_geometry_id,
                      tracks=len(self.tracks),
                      instance_topic_active=bool(self.calibration_id),
                      subscribe=[self.image_topic, self.depth_topic, self.color_info_topic, self.depth_info_topic],
                      publish=[self.objects_topic, self.sorting_topic, self.board_topic,
                               self.observations_topic, self.instances_topic, self.status_topic])
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def save_evidence(self, image, depth, color_info, depth_info, color_msg, depth_msg, skew):
        try:
            directory = Path(self.evidence_dir)
            directory.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(directory / 'color.png'), image)
            cv2.imwrite(str(directory / 'aligned_depth.png'),
                        depth if depth.dtype == np.uint16 else (depth * 1000).astype(np.uint16))
            (directory / 'camera_info.json').write_text(json.dumps(
                {'color': {'K': list(color_info.K), 'D': list(color_info.D), 'width': color_info.width,
                           'height': color_info.height, 'frame_id': color_info.header.frame_id},
                 'depth': {'K': list(depth_info.K), 'D': list(depth_info.D), 'width': depth_info.width,
                           'height': depth_info.height, 'frame_id': depth_info.header.frame_id}},
                indent=2), encoding='utf-8')
            (directory / 'frame_meta.json').write_text(json.dumps(
                {'color_stamp': color_msg.header.stamp.to_sec(), 'depth_stamp': depth_msg.header.stamp.to_sec(),
                 'sync_skew_s': skew, 'color_encoding': color_msg.encoding,
                 'depth_encoding': depth_msg.encoding, 'ros_stamp': rospy.Time.now().to_sec(),
                 'board_geometry_id': self.board_geometry_id}, indent=2), encoding='utf-8')
            self.evidence_saved = True
            self.stats['evidence'] = str(directory)
            rospy.loginfo('field_perception evidence written to %s', directory)
        except (OSError, cv2.error, ValueError) as exc:
            self.stats['last_error'] = 'evidence: ' + str(exc)
            rospy.logwarn_throttle(5.0, 'field_perception evidence failed: %s', exc)



# Single-snapshot sorting inspection; isolated from formal grasp instances.
DEFAULT_LABELS = ('clock', 'teddybear', 'banana', 'cake')

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


def _sorting_rotated_matches(image, templates, target, threshold):
    """Multi-instance template matching over in-plane angles and scales."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    candidates = []
    for template in templates.get(target, []):
        source = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        side = max(source.shape)
        square = np.full((side, side), 255, dtype=np.uint8)
        dy = (side-source.shape[0]) // 2
        dx = (side-source.shape[1]) // 2
        square[dy:dy+source.shape[0], dx:dx+source.shape[1]] = source
        for angle in range(0, 360, 30):
            transform = cv2.getRotationMatrix2D((side/2., side/2.), angle, 1.)
            rotated = cv2.warpAffine(square, transform, (side, side),
                                     flags=cv2.INTER_LINEAR, borderValue=255)
            for scale in np.arange(.35, .96, .05):
                size = int(round(side * scale))
                if size >= min(gray.shape) or size < 16:
                    continue
                ref = cv2.resize(rotated, (size, size), interpolation=cv2.INTER_AREA)
                response = cv2.matchTemplate(gray, ref, cv2.TM_CCOEFF_NORMED)
                peaks = (response >= threshold) & (response == cv2.dilate(response, np.ones((5, 5), np.uint8)))
                ys, xs = np.where(peaks)
                if len(xs) > 12:
                    selected = np.argpartition(response[ys, xs], -12)[-12:]
                    xs, ys = xs[selected], ys[selected]
                for x, y in zip(xs, ys):
                    candidates.append({'label': target, 'score': float(response[y, x]),
                        'source': 'rotated_ncc', 'partial': False, 'cx': float(x+size/2.),
                        'cy': float(y+size/2.), 'w': float(size), 'h': float(size),
                        'quad': None, 'angle_deg': int(angle)})
    kept = []
    for candidate in sorted(candidates, key=lambda d: d['score'], reverse=True):
        if any(_iou(_box_of(candidate), _box_of(other)) > .25 for other in kept):
            continue
        kept.append(candidate)
    return sorted(kept, key=lambda d: (d['cy'], d['cx']))


def _sorting_edge_matches(image, templates, target, threshold):
    """Match cropped templates only against a thin strip at image boundaries."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    candidates = []
    for template in templates.get(target, []):
        source = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        side = max(source.shape)
        square = np.full((side, side), 255, dtype=np.uint8)
        dy, dx = (side-source.shape[0]) // 2, (side-source.shape[1]) // 2
        square[dy:dy+source.shape[0], dx:dx+source.shape[1]] = source
        for angle in range(0, 360, 30):
            transform = cv2.getRotationMatrix2D((side/2., side/2.), angle, 1.)
            rotated = cv2.warpAffine(square, transform, (side, side),
                                     flags=cv2.INTER_LINEAR, borderValue=255)
            for scale in np.arange(.35, .96, .05):
                size = int(round(side*scale))
                if size < 24 or size >= min(height, width):
                    continue
                ref = cv2.resize(rotated, (size, size), interpolation=cv2.INTER_AREA)
                for fraction in (.20, .35, .50, .65):
                    cut = int(round(size*fraction))
                    remainder = size-cut
                    if remainder < 18:
                        continue
                    margin = 8
                    views = (
                        ('top', ref[cut:, :], gray[:min(height, remainder+margin), :], 0, 0),
                        ('bottom', ref[:remainder, :], gray[max(0, height-remainder-margin):, :], 0, max(0, height-remainder-margin)),
                        ('left', ref[:, cut:], gray[:, :min(width, remainder+margin)], 0, 0),
                        ('right', ref[:, :remainder], gray[:, max(0, width-remainder-margin):], max(0, width-remainder-margin), 0))
                    for edge, piece, region, ox, oy in views:
                        if min(piece.shape) < 18 or np.std(piece) < 5:
                            continue
                        response = cv2.matchTemplate(region, piece, cv2.TM_CCOEFF_NORMED)
                        peaks = (response >= threshold) & (response == cv2.dilate(response, np.ones((5, 5), np.uint8)))
                        ys, xs = np.where(peaks)
                        if len(xs) > 6:
                            chosen = np.argpartition(response[ys, xs], -6)[-6:]
                            xs, ys = xs[chosen], ys[chosen]
                        ph, pw = piece.shape
                        for x, y in zip(xs, ys):
                            px, py = int(x)+ox, int(y)+oy
                            candidates.append({'label': target, 'score': float(response[y,x]),
                                'source': 'edge_partial_ncc', 'partial': True,
                                'cx': px+pw/2., 'cy': py+ph/2., 'w': float(pw),
                                'h': float(ph), 'quad': None, 'angle_deg': int(angle),
                                'image_edge': edge})
    kept = []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    for item in sorted(candidates, key=lambda d: d['score'], reverse=True):
        # The visible teddy template has brown coloration; clock faces are
        # predominantly neutral. This is a heuristic for edge-only ambiguity.
        x1, y1, x2, y2 = (int(round(v)) for v in _box_of(item))
        patch = hsv[max(0, y1):min(height, y2), max(0, x1):min(width, x2)]
        if not patch.size:
            continue
        brown = ((patch[:, :, 0] < 35) & (patch[:, :, 1] > 45) &
                 (patch[:, :, 2] > 40)).mean()
        if (target == 'teddybear' and brown < .06) or (target == 'clock' and brown > .12):
            continue
        if not any(_iou(_box_of(item), _box_of(other)) > .25 for other in kept):
            kept.append(item)
    return kept


def detect_sorting_target(image, templates, target='banana'):
    """Find multiple target faces. Scores are similarities, not probabilities."""
    if target not in ('banana', 'clock', 'teddybear', 'cake'):
        raise ValueError('unsupported target: ' + str(target))
    if not templates.get(target):
        raise ValueError(target + ' template missing')
    if target != 'banana':
        thresholds = {'clock': .66, 'teddybear': .72, 'cake': .72}
        candidates = _sorting_rotated_matches(image, templates, target, thresholds[target])
        if target in ('clock', 'teddybear'):
            candidates.extend(_sorting_edge_matches(image, templates, target, thresholds[target]))
        # Like the classification-area reader, use ORB/homography for detailed
        # designs. Unlike that reader, retain every spatially distinct instance.
        for item in _feature_detect(image, {target: templates[target]}, prefer='orb'):
            item = dict(item, partial=False)
            candidates.append(item)
        if target == 'teddybear':
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            verified = []
            for item in candidates:
                x1, y1, x2, y2 = (int(round(v)) for v in _box_of(item))
                patch = hsv[max(0, y1):min(hsv.shape[0], y2), max(0, x1):min(hsv.shape[1], x2)]
                if not patch.size:
                    continue
                brown = ((patch[:, :, 0] < 35) & (patch[:, :, 1] > 45) &
                         (patch[:, :, 2] > 40)).mean()
                if brown < .06:
                    continue
                verified.append(item)
            candidates = verified
        kept = []
        for item in sorted(candidates, key=lambda d: d['score'], reverse=True):
            if not any(_iou(_box_of(item), _box_of(other)) > .25 for other in kept):
                kept.append(item)
        return sorted(kept, key=lambda d: (d['cy'], d['cx']))
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
        edge = x <= 2 or y <= 2 or x + w >= width - 2 or y + h >= height - 2
        # Small brown details of the bear can have a yellow hue. Away from a
        # frame edge they are too small to represent a printed banana face.
        if edge:
            enough = w >= 30 and h >= 18 and area >= 350
        else:
            enough = w >= 50 and h >= 30 and area >= 650
        if not enough:
            continue
        distance = float(cv2.matchShapes(contour, ref, cv2.CONTOURS_MATCH_I1, 0))
        if not np.isfinite(distance):
            continue
        partial = distance > 0.25 or edge
        # A cropped or partly occluded top is still eligible; depth determines
        # whether its visible surface is parallel to the support plane.
        if distance > (0.85 if edge else 0.65):
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
    if 'top_height_m' in det:
        return det['top_height_m'], det['top_layer']
    if plane is None:
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


def sorting_top_face(det, depth_m, camera_matrix, support_plane):
    """Accept only depth-supported faces parallel to the candidate support plane."""
    if support_plane is None:
        return False, '无支撑平面'
    h, w = depth_m.shape
    x0 = max(0, int(det['cx'] - det['w'] * .32))
    x1 = min(w, int(det['cx'] + det['w'] * .32))
    y0 = max(0, int(det['cy'] - det['h'] * .32))
    y1 = min(h, int(det['cy'] + det['h'] * .32))
    if x1-x0 < 8 or y1-y0 < 8:
        return False, '检测框过小'
    yy, xx = np.mgrid[y0:y1:2, x0:x1:2]
    z = depth_m[y0:y1:2, x0:x1:2]
    valid = np.isfinite(z) & (z > .1) & (z < 2.)
    if int(valid.sum()) < 35:
        return False, '顶面有效深度不足'
    k = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    points = np.column_stack(((xx[valid]-k[0, 2])*z[valid]/k[0, 0],
                              (yy[valid]-k[1, 2])*z[valid]/k[1, 1], z[valid]))
    rng = np.random.RandomState(17)
    best = None
    for _ in range(50):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1.e-8:
            continue
        normal /= length
        inliers = np.abs((points-a) @ normal) < .006
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = (count, inliers)
    if best is None or best[0] < max(30, int(len(points)*.45)):
        return False, '目标平面不稳定'
    selected = points[best[1]]
    _, _, vh = np.linalg.svd(selected-np.median(selected, axis=0), full_matrices=False)
    alignment = abs(float(vh[-1] @ support_plane['normal']))
    if alignment < .92:
        return False, '目标面与支撑平面不平行'
    height = float(np.median(support_plane['offset_m'] - selected @ support_plane['normal']))
    layer = 1 if .07 <= height <= .13 else 2 if .17 <= height <= .23 else None
    if layer is None:
        return False, '顶面高度或深度无效'
    if det['label'] == 'cake' and layer != 2:
        return False, '蛋糕高度与第二层不一致'
    det['top_height_m'], det['top_layer'] = height, layer
    return True, '顶面候选'


def deduplicate_sorting_faces(detections):
    """Remove same-label, same-layer near-contained boxes after depth checks."""
    def rank(det):
        return (not det.get('partial', False), det['w'] * det['h'], det['score'])
    kept = []
    for item in sorted(detections, key=rank, reverse=True):
        a = _box_of(item)
        duplicate = False
        for other in kept:
            if item['label'] != other['label'] or item['top_layer'] != other['top_layer']:
                continue
            if abs(item['top_height_m'] - other['top_height_m']) > .025:
                continue
            b = _box_of(other)
            overlap = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
            smaller = min(item['w']*item['h'], other['w']*other['h'])
            containment = overlap / smaller if smaller > 0 else 0.
            distance = math.hypot(item['cx']-other['cx'], item['cy']-other['cy'])
            extent = max(item['w'], item['h'], other['w'], other['h'])
            partial_overlap = (item.get('partial') or other.get('partial')) and containment >= .45 and distance <= .55 * extent
            if containment >= .8 or partial_overlap:
                duplicate = True
                break
        if not duplicate:
            kept.append(item)
    return sorted(kept, key=lambda d: (d['label'], d['cy'], d['cx']))


def _sorting_target_snapshot(args):
    """One synchronized RGB-D snapshot; never commands the arm."""
    import threading
    from concurrent.futures import ThreadPoolExecutor
    import cv2
    import numpy as np
    import message_filters
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image

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
    labels = ('banana', 'clock', 'teddybear', 'cake') if args.target == 'all' else (args.target,)
    analysis_started = time.monotonic()
    if len(labels) > 1:
        with ThreadPoolExecutor(max_workers=3) as pool:
            groups = list(pool.map(lambda label: detect_sorting_target(image, templates, label), labels))
    else:
        groups = [detect_sorting_target(image, templates, labels[0])]
    candidates = [item for group in groups for item in group]
    plane = sorting_support_plane(depth, info.K)
    detections = []
    rejected = {}
    for candidate in candidates:
        accepted, reason = sorting_top_face(candidate, depth, info.K, plane)
        if accepted:
            detections.append(candidate)
        else:
            rejected[reason] = rejected.get(reason, 0) + 1
    before_dedup = len(detections)
    detections = deduplicate_sorting_faces(detections)
    duplicates_removed = before_dedup - len(detections)
    # Resolve the observed clock-on-bear mistake only where the two class
    # candidates overlap on the same physical layer. Do not reject a clock
    # merely because an orange arm or another coloured object covers part of it.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    false_clocks = set()
    for ci, clock in enumerate(detections):
        if clock['label'] != 'clock':
            continue
        a = _box_of(clock)
        for bear in detections:
            if bear['label'] != 'teddybear' or bear['top_layer'] != clock['top_layer']:
                continue
            b = _box_of(bear)
            overlap = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
            area = min(clock['w']*clock['h'], bear['w']*bear['h'])
            if area <= 0 or overlap / area <= .6:
                continue
            x1, y1, x2, y2 = (int(round(v)) for v in b)
            patch = hsv[max(0, y1):min(hsv.shape[0], y2), max(0, x1):min(hsv.shape[1], x2)]
            if patch.size and ((patch[:,:,0] < 35) & (patch[:,:,1] > 45) & (patch[:,:,2] > 40)).mean() >= .12:
                false_clocks.add(ci)
                break
    if false_clocks:
        detections = [item for i, item in enumerate(detections) if i not in false_clocks]
    # Do not silently choose between competing class-specific score types.
    for i, first in enumerate(detections):
        for other in detections[i+1:]:
            if first['label'] == other['label'] or first['top_layer'] != other['top_layer']:
                continue
            a, b = _box_of(first), _box_of(other)
            intersection = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
            area = min(first['w']*first['h'], other['w']*other['h'])
            if area > 0 and intersection/area > .6:
                first['class_conflict'] = other['class_conflict'] = True
    visual = image.copy()
    print('[相机] 单帧时间：%.3f；彩色/深度时间差：%.3f s' %
          (color.header.stamp.to_sec(), abs(color.header.stamp.to_sec()-depth_msg.header.stamp.to_sec())))
    print('[识别] 图像分析耗时：%.2f s' % (time.monotonic() - analysis_started))
    print('[识别] 目标：%s；同一帧逐类别检测' % args.target)
    print('[识别] 顶面候选：%d；其中不完整：%d' %
          (len(detections), sum(d['partial'] for d in detections)))
    if duplicates_removed:
        print('[识别] 已合并同一顶面的重复候选：%d' % duplicates_removed)
    for label in labels:
        expected = 1 if label == 'cake' else 3
        print('[识别] %s：%d 个候选，场地预期 %d 个（不要求全部在视野内）' %
              (label, sum(d['label'] == label for d in detections), expected))
    print('[识别] 仅保留深度支持的顶面；未计入候选：%s' % (rejected or '无'))
    if 'cake' in labels:
        print('[定位] 场地规则：蛋糕预期在第二层；仍需深度核对')
    print('[识别] 分数含义随方法不同，不是经过校准的识别概率或召回率')
    if plane is None:
        print('[定位] 未找到足够支持的平面；层数未知')
    else:
        print('[定位] 支撑平面候选：%d/%d 深度点；每层高度 0.10 m，需现场核对' %
              (plane['inliers'], plane['samples']))
    results = []
    for index, det in enumerate(detections, 1):
        height, layer = sorting_target_height(det, depth, info.K, plane)
        if det['label'] == 'cake' and layer != 2:
            # The field rule expects cake on layer 2; disagreement or missing
            # depth is not evidence that this particular detection is there.
            layer = None
        # Expand the displayed box slightly without changing matching or centre.
        x1 = max(0, int(det['cx']-det['w']*.55)); y1 = max(0, int(det['cy']-det['h']*.55))
        x2 = min(image.shape[1]-1, int(det['cx']+det['w']*.55))
        y2 = min(image.shape[0]-1, int(det['cy']+det['h']*.55))
        kind = '不完整顶面' if det['partial'] else '完整图案'
        level = str(layer) if layer else '?'
        print('[识别] %d: %s，%s%s，中心=(%d,%d)，方法=%s，分数=%.3f，层=%s，高度=%s' %
              (index, det['label'], kind, '，类别冲突' if det.get('class_conflict') else '', det['cx'], det['cy'], det['source'], det['score'], level,
               '未知' if height is None else '%.3f m' % height))
        results.append({'label': det['label'], 'score': float(det['score']),
                        'score_method': det['source'],
                        'center_px': [float(det['cx']), float(det['cy'])],
                        'bbox_px': [x1, y1, x2, y2], 'layer': layer,
                        'height_m': None if height is None else float(height),
                        'partial': bool(det['partial']),
                        'class_conflict': bool(det.get('class_conflict', False))})
        box_color = {'banana': (0, 220, 255), 'clock': (255, 180, 0),
                     'teddybear': (0, 220, 0), 'cake': (255, 0, 255)}[det['label']]
        cv2.rectangle(visual, (x1, y1), (x2, y2), box_color, 2)
        label = '%s%s%s %.2f L%s' % (det['label'], '?' if det.get('class_conflict') else '',
                                    ' partial' if det['partial'] else '', det['score'], level)
        cv2.putText(visual, label, (x1, max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .5, box_color, 2)
    print('[识别] 说明：候选可能是误检或同一方块的另一个面；需人工核对，不能直接作为抓取依据。', flush=True)
    payload = {'stamp': color.header.stamp.to_sec(), 'frame_id': color.header.frame_id,
               'source': 'single_rgbd_snapshot', 'purpose': 'diagnostic_only',
               'score_comparable_across_methods': False,
               'support_plane': 'candidate' if plane is not None else 'unknown',
               'objects': results}
    encoded = json.dumps(payload, ensure_ascii=False, allow_nan=False)
    result_pub = rospy.Publisher('/suat/perception/sorting_snapshot', String, queue_size=1, latch=True)
    result_pub.publish(String(data=encoded))
    pub = rospy.Publisher('/suat/perception/target_image', Image, queue_size=1, latch=True)
    annotated = bridge.cv2_to_imgmsg(visual, 'bgr8'); annotated.header = color.header
    pub.publish(annotated)
    if os.environ.get('DISPLAY') and not args.no_window:
        print('[画面] 已打开标注图；按任意键关闭。', flush=True)
        cv2.imshow('Sorting target snapshot', visual)
        cv2.waitKey(0)
        # The user may already have closed the window using its close button.
        # OpenCV 4.2 GTK raises on destroyWindow for an unregistered window.
        cv2.destroyAllWindows()
    else:
        print('[画面] 无 DISPLAY 或已指定 --no-window；只输出识别结果。', flush=True)
        rospy.sleep(1.)
    return 0


def main():
    argv = rospy.myargv()[1:]
    if any(arg == '--target' or arg.startswith('--target=') for arg in argv):
        import rospkg
        parser = argparse.ArgumentParser(description='Read-only sorting RGB-D snapshot')
        parser.add_argument('--target', required=True, choices=('all', 'banana', 'clock', 'teddybear', 'cake'))
        parser.add_argument('--ros-topic', default='/camera/color/image_raw')
        parser.add_argument('--templates', default=str(Path(rospkg.RosPack().get_path('suat_mission')) / 'assets' / 'templates'))
        parser.add_argument('--no-window', action='store_true')
        return _sorting_target_snapshot(parser.parse_args(argv))
    rospy.init_node('suat_field_perception')
    FieldPerception()
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
