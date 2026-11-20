#!/usr/bin/env python3
"""手眼标定采集节点。

流程：让机械臂末端带着标记走到若干已知臂坐标位姿，每个点位停稳后
用相机测出标记的相机系 3D 坐标，攒够对应点对（臂系 <-> 相机系）后存盘，
交给 handeye.py 的 fit_rigid 解出外参。

安全约束（这套臂没有闭环反馈，只能靠开环小步走）：
- 必须先知道起点。优先读 /SwiftproState_topic（由 swiftpro_read_node 提供真值），
  否则要求显式给 ~start_xyz；两者都没有就拒绝启动。
- 每步只走 ~step_mm，走完停 ~dwell_s，用分段小步代替速度控制，
  因为驱动固定发 "G0 ... F10000"，没有速度参数。
- 每个目标点先做工作空间校验，越界不执行。
- 默认 dry_run 只打印路径，不驱动机械臂。
"""
import json
import math
import os
import time

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from swiftpro.msg import SwiftproState, position, status

from suat_mission.manipulation.arm_workspace import check_point
from suat_mission.manipulation.depth_geometry import (CameraIntrinsics, DepthConfig, back_project,
                                         sample_depth_centroid)


def _parse_list(value, name):
    """launch 里把列表写成字符串是常态，这里统一容错。"""
    if isinstance(value, (list, tuple)):
        return [float(v) for v in value]
    if isinstance(value, str):
        text = value.strip().strip('[]')
        if not text:
            return []
        return [float(v) for v in text.replace(';', ',').split(',') if v.strip()]
    raise ValueError('%s is not a list: %r' % (name, value))


class HandeyeCollect:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = False
        self.intrinsics = None
        self.latest_color = None      # (stamp, bgr)
        self.latest_depth = None      # (stamp, uint16 mm)
        self.latest_arm = None        # (stamp, (x, y, z))
        self.pairs = []

        self.dry_run = bool(rospy.get_param('~dry_run', True))
        self.aruco_id = int(rospy.get_param('~aruco_id', 7))
        self.aruco_size_mm = float(rospy.get_param('~aruco_size_mm', 40.0))
        self.aruco_dict_name = str(rospy.get_param('~aruco_dict', 'DICT_4X4_50'))
        self.step_mm = float(rospy.get_param('~step_mm', 3.0))
        self.dwell_s = float(rospy.get_param('~dwell_s', 0.35))
        self.settle_frames = int(rospy.get_param('~settle_frames', 5))
        self.settle_px = float(rospy.get_param('~settle_px', 1.5))
        self.out_file = os.path.expanduser(str(rospy.get_param(
            '~out_file', '~/SUAT/spark/data/calibration/handeye_pairs.json')))
        self.depth_cfg = DepthConfig(
            min_depth_mm=float(rospy.get_param('~min_depth_mm', 120.0)),
            max_depth_mm=float(rospy.get_param('~max_depth_mm', 1200.0)),
            cluster_tolerance_mm=float(rospy.get_param('~cluster_tolerance_mm', 12.0)),
            min_valid_pixels=int(rospy.get_param('~min_valid_pixels', 20)),
            min_cluster_ratio=float(rospy.get_param('~min_cluster_ratio', 0.2)),
            max_residual_mm=float(rospy.get_param('~max_residual_mm', 6.0)))
        self.z_min = float(rospy.get_param('~arm_z_min', -60.0))
        self.z_max = float(rospy.get_param('~arm_z_max', 220.0))
        self.travel_z = float(rospy.get_param('~travel_z_mm', 120.0))
        self.start_xyz = _parse_list(rospy.get_param('~start_xyz', ''), '~start_xyz')

        # ArUco 字典
        self.detector = None
        try:
            import cv2
            aruco = cv2.aruco
            dict_id = getattr(aruco, self.aruco_dict_name, aruco.DICT_4X4_50)
            self.dictionary = aruco.Dictionary_get(dict_id)
            self.detector_params = aruco.DetectorParameters_create()
            self.detector = aruco
        except Exception as exc:      # noqa: BLE001 - 没装 aruco 就只报错不崩
            rospy.logwarn('ArUco unavailable: %s', exc)

        self.arm_pub = rospy.Publisher('/position_write_topic', position, queue_size=1)
        # M17 attach：read_node 启动会发 M2019 松开电机，之后 G0 会被固件静默丢弃，
        # 所以每次移动前都自己重新上力，不依赖驱动启动时那一次。
        self.motor_pub = rospy.Publisher('/swiftpro_status_topic', status, queue_size=1)
        rospy.Subscriber('/camera/color/image_raw', Image, self._on_color, queue_size=1)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image,
                         self._on_depth, queue_size=1)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self._on_info, queue_size=1)
        rospy.Subscriber('/SwiftproState_topic', SwiftproState, self._on_arm, queue_size=1)

        rospy.Service('~collect', Trigger, self._srv_collect)
        rospy.Service('~save', Trigger, self._srv_save)
        rospy.Service('~run', Trigger, self._srv_run)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        rospy.loginfo('handeye_collect ready; dry_run=%s aruco_id=%d', self.dry_run, self.aruco_id)

    # ---------------- 订阅回调 ----------------
    def _on_color(self, msg):
        self.latest_color = (msg.header.stamp.to_sec(), self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def _on_depth(self, msg):
        self.latest_depth = (msg.header.stamp.to_sec(), self.bridge.imgmsg_to_cv2(msg, '16UC1'))

    def _on_info(self, msg):
        if self.intrinsics is None:
            self.intrinsics = CameraIntrinsics.from_k(msg.K)
            rospy.loginfo('intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f',
                          msg.K[0], msg.K[4], msg.K[2], msg.K[5])

    def _on_arm(self, msg):
        self.latest_arm = (rospy.Time.now().to_sec(), (float(msg.x), float(msg.y), float(msg.z)))

    # ---------------- 视觉：标记 -> 相机系 3D ----------------
    def _detect_marker(self):
        """返回 (u, v, cam_xyz, bbox) 或 None。中心点优先用角点求均值。"""
        if self.detector is None or self.latest_color is None:
            return None
        import cv2
        stamp, bgr = self.latest_color
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray, self.dictionary, parameters=self.detector_params)
        if ids is None:
            return None
        for quad, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) != self.aruco_id:
                continue
            pts = quad.reshape(4, 2)
            u, v = float(pts[:, 0].mean()), float(pts[:, 1].mean())
            x0, x1 = float(pts[:, 0].min()), float(pts[:, 0].max())
            y0, y1 = float(pts[:, 1].min()), float(pts[:, 1].max())
            w, h = x1 - x0, y1 - y0
            if w < 6 or h < 6:
                return None
            if self.latest_depth is None or self.intrinsics is None:
                return None
            try:
                sample = sample_depth_centroid(self.latest_depth[1], (u, v, w, h),
                                               self.depth_cfg)
            except ValueError as exc:
                rospy.loginfo_throttle(1.0, 'marker depth rejected: %s', exc)
                return None
            point = back_project(sample.u, sample.v, sample.z_mm, self.intrinsics)
            return sample.u, sample.v, tuple(float(c) for c in point), (x0, y0, w, h)
        return None

    def _wait_settled(self, timeout_s=6.0):
        """等标记在画面里稳定下来，返回最后一次检测结果。"""
        samples = []
        t0 = time.time()
        while time.time() - t0 < timeout_s and not rospy.is_shutdown():
            hit = self._detect_marker()
            if hit is not None:
                samples.append(hit)
                if len(samples) >= self.settle_frames:
                    us = np.array([s[0] for s in samples])
                    vs = np.array([s[1] for s in samples])
                    if us.std() <= self.settle_px and vs.std() <= self.settle_px:
                        return samples[-1]
                    samples = samples[1:]
            else:
                samples = []
            rospy.sleep(0.05)
        return None

    # ---------------- 运动 ----------------
    def _current_arm(self):
        if self.latest_arm is not None:
            return self.latest_arm[1]
        if len(self.start_xyz) == 3:
            return tuple(self.start_xyz)
        return None

    def _check_reach(self, x, y, z):
        verdict = check_point(x, y, z, margin_mm=5.0,
                              z_min_mm=self.z_min, z_max_mm=self.z_max)
        return None if verdict.ok else verdict.reason

    def _move_to(self, target):
        start = self._current_arm()
        if start is None:
            raise RuntimeError('no current arm pose; give ~start_xyz or run swiftpro_read_node')
        if not self.dry_run:
            # 每次运动前重新上力；电机被 M2019 detach 时 G0 会被固件静默丢弃。
            for _ in range(3):
                self.motor_pub.publish(status(1))
                rospy.sleep(0.1)
            rospy.sleep(0.4)
        span = max(abs(target[i] - start[i]) for i in range(3))
        steps = max(1, int(math.ceil(span / self.step_mm)))
        for k in range(1, steps + 1):
            ratio = float(k) / steps
            way = tuple(start[i] + (target[i] - start[i]) * ratio for i in range(3))
            if self.dry_run:
                if k == steps:
                    rospy.loginfo('[dry] -> %.1f %.1f %.1f', *way)
                continue
            msg = position()
            msg.x, msg.y, msg.z = way
            self.arm_pub.publish(msg)
            rospy.sleep(self.dwell_s)
        if not self.dry_run:
            self.latest_arm = (rospy.Time.now().to_sec(), tuple(target))

    def _move_safe(self, target):
        """先竖直抬到安全高度，再水平走，最后竖直下降。

        末端贴桌面时（实测起点 z≈1mm）任何斜线移动都会刮台面，
        所以三段式是硬约束，不是优化。
        """
        start = self._current_arm()
        if start is None:
            raise RuntimeError('no current arm pose; give ~start_xyz')
        sx, sy, sz = start
        tx, ty, tz = target
        travel_z = min(max(sz, tz, self.travel_z), self.z_max)
        if travel_z > sz + 1.0:
            self._move_to((sx, sy, travel_z))
        if abs(tx - sx) > 1.0 or abs(ty - sy) > 1.0:
            self._move_to((tx, ty, travel_z))
        if abs(tz - travel_z) > 1.0:
            self._move_to((tx, ty, tz))
        self.latest_arm = (rospy.Time.now().to_sec(), (tx, ty, tz))

    def _run_points(self, targets):
        for idx, target in enumerate(targets):
            if rospy.is_shutdown():
                return 'aborted'
            reason = self._check_reach(*target)
            if reason:
                rospy.logwarn('skip point %d %s: %s', idx, target, reason)
                continue
            rospy.loginfo('moving to point %d/%d: %.1f %.1f %.1f', idx + 1, len(targets), *target)
            self._move_safe(target)
            hit = self._wait_settled()
            if hit is None:
                rospy.logwarn('point %d: marker not settled, skipped', idx)
                continue
            u, v, cam, bbox = hit
            self.pairs.append({'arm': [round(c, 2) for c in target],
                               'cam': [round(c, 2) for c in cam],
                               'pixel': [round(u, 1), round(v, 1)],
                               'bbox': [round(b, 1) for b in bbox]})
            rospy.loginfo('point %d recorded: arm=%s cam=%s', idx, self.pairs[-1]['arm'],
                          self.pairs[-1]['cam'])
        return 'ok'

    # ---------------- 服务 ----------------
    def _srv_collect(self, _req):
        hit = self._wait_settled()
        if hit is None:
            return TriggerResponse(False, 'marker id=%d not detected' % self.aruco_id)
        u, v, cam, bbox = hit
        arm = self._current_arm()
        if arm is None:
            return TriggerResponse(False, 'no arm pose available')
        self.pairs.append({'arm': [round(c, 2) for c in arm],
                           'cam': [round(c, 2) for c in cam],
                           'pixel': [round(u, 1), round(v, 1)],
                           'bbox': [round(b, 1) for b in bbox]})
        return TriggerResponse(True, 'recorded %d pairs' % len(self.pairs))

    def _srv_save(self, _req):
        if len(self.pairs) < 4:
            return TriggerResponse(False, 'need >= 4 pairs, have %d' % len(self.pairs))
        os.makedirs(os.path.dirname(self.out_file), exist_ok=True)
        payload = {'created': time.strftime('%Y-%m-%dT%H:%M:%S'),
                   'aruco_id': self.aruco_id,
                   'aruco_size_mm': self.aruco_size_mm,
                   'intrinsics': vars(self.intrinsics) if self.intrinsics else None,
                   'pairs': self.pairs}
        with open(self.out_file, 'w') as handle:
            json.dump(payload, handle, indent=2)
        rospy.loginfo('saved %d pairs to %s', len(self.pairs), self.out_file)
        return TriggerResponse(True, '%d pairs -> %s' % (len(self.pairs), self.out_file))

    def _srv_run(self, _req):
        raw = rospy.get_param('~points', '')
        try:
            flat = _parse_list(raw, '~points')
        except ValueError as exc:
            return TriggerResponse(False, str(exc))
        if len(flat) < 12 or len(flat) % 3:
            return TriggerResponse(False, '~points needs >= 4 xyz triples, got %d values' % len(flat))
        targets = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        self.pairs = []
        result = self._run_points(targets)
        return TriggerResponse(result == 'ok', '%s, %d pairs' % (result, len(self.pairs)))


def main():
    rospy.init_node('handeye_collect')
    node = HandeyeCollect()
    rospy.spin()


if __name__ == '__main__':
    main()
