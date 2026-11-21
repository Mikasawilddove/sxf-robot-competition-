#!/usr/bin/env python3
"""RGB-D 抓取入口：框选 -> 深度质心 -> 机械臂坐标 -> 分段路径 -> 吸取抬升。

默认只算不动（dry_run）。只有参数 ~execute:=true 且收到 /suat/arm/command 的
"pick" 才会向 /position_write_topic 与 /pump_topic 发布指令。

定位有两种模式：
- handeye：配置 ~extrinsics_file 时，用相机到机械臂的 4x4 外参做真三维换算；
- planar ：无外参时退回厂商标定的二维线性映射，z 取固定 ~grasp_z_mm，
           保证没有外参也能先跑通流程。
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from swiftpro.msg import position, status
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray

import message_filters

from suat_mission.manipulation.arm_path import ArmLimits, ArmPathConfig, path_summary, plan_pick
from suat_mission.manipulation.arm_workspace import check_point, grasp_height
from suat_mission.manipulation.depth_geometry import (CameraIntrinsics, DepthConfig, Extrinsics,
                                         localize)
from suat_mission.vision.perception import COCO_IDS


def list_param(name, default):
    """Read a list parameter, tolerating the "[a, b, c]" string form.

    roslaunch <param value="[...]"/> lands on the parameter server as a plain
    string, so tuple(raw) would silently split it into characters.
    """
    raw = rospy.get_param(name, default)
    if isinstance(raw, str):
        text = raw.strip()
        if text.startswith('[') and text.endswith(']'):
            text = text[1:-1]
        return [item.strip().strip('\'"') for item in text.split(',') if item.strip()]
    return list(raw)


class ArmVisionGrasp:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.busy = False
        self.latest_frame = None
        self.latest_detections = None
        self.intrinsics = None
        self.extrinsics = None
        self.planar = None
        self.latest_target = None

        self.color_topic = rospy.get_param('~color_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param(
            '~depth_topic', '/camera/aligned_depth_to_color/image_raw')
        info_topic = rospy.get_param('~camera_info_topic', '/camera/color/camera_info')
        objects_topic = rospy.get_param('~objects_topic', '/objects')

        self.execute = bool(rospy.get_param('~execute', False))
        self.attach_motor = bool(rospy.get_param('~attach_motor', True))
        self.attach_settle_s = float(rospy.get_param('~attach_settle_s', 0.6))
        self.labels = tuple(str(v) for v in list_param(
            '~target_labels', ('teddybear', 'clock', 'banana', 'cake')))
        self.min_quality = float(rospy.get_param('~min_quality', 0.35))
        self.frame_slack = float(rospy.get_param('~frame_slack_s', 0.5))
        self.target_max_age = float(rospy.get_param('~target_max_age_s', self.frame_slack))
        if not math.isfinite(self.target_max_age) or self.target_max_age <= 0:
            raise ValueError('target_max_age_s must be finite and positive')
        self.grasp_z_offset = float(rospy.get_param('~grasp_z_offset_mm', 0.0))
        self.grasp_z_default = float(rospy.get_param('~grasp_z_mm', -50.0))

        # 方块高度已知（比赛是 10cm 立方体）：抓取高度 = 支撑面高度 + 方块高度。
        # 给了 support_z_mm 就以它为准，否则退回单点 grasp_z_mm。
        self.block_height = float(rospy.get_param('~block_height_mm', 100.0))
        support = float(rospy.get_param('~support_z_mm', float('nan')))
        self.support_z = support if support == support else None
        self.reach_margin = float(rospy.get_param('~reach_margin_mm', 10.0))
        self.front_only = bool(rospy.get_param('~reach_front_only', True))

        self.depth_cfg = DepthConfig(
            min_depth_mm=float(rospy.get_param('~min_depth_mm', 120.0)),
            max_depth_mm=float(rospy.get_param('~max_depth_mm', 700.0)),
            shrink=float(rospy.get_param('~bbox_shrink', 0.15)),
            cluster_tolerance_mm=float(rospy.get_param('~cluster_tolerance_mm', 12.0)),
            min_valid_pixels=int(rospy.get_param('~min_valid_pixels', 60)),
            max_spread_mm=float(rospy.get_param('~max_spread_mm', 40.0)),
            max_residual_mm=float(rospy.get_param('~max_residual_mm', 6.0)),
            max_gradient_mm_px=float(rospy.get_param('~max_gradient_mm_px', 0.8)))

        self.limits = ArmLimits(
            x_min=float(rospy.get_param('~x_min', 40.0)),
            x_max=float(rospy.get_param('~x_max', 280.0)),
            y_min=float(rospy.get_param('~y_min', -180.0)),
            y_max=float(rospy.get_param('~y_max', 180.0)),
            z_min=float(rospy.get_param('~z_min', -80.0)),
            z_max=float(rospy.get_param('~z_max', 200.0)))

        self.path_cfg = ArmPathConfig(
            safe_z_mm=float(rospy.get_param('~safe_z_mm', 150.0)),
            approach_mm=float(rospy.get_param('~approach_mm', 60.0)),
            lift_mm=float(rospy.get_param('~lift_mm', 80.0)),
            lateral_step_mm=float(rospy.get_param('~lateral_step_mm', 15.0)),
            vertical_step_mm=float(rospy.get_param('~vertical_step_mm', 6.0)),
            lateral_speed_mm_s=float(rospy.get_param('~lateral_speed_mm_s', 60.0)),
            vertical_speed_mm_s=float(rospy.get_param('~vertical_speed_mm_s', 25.0)))

        start = list_param('~arm_start_pose', [50.0, 180.0, 160.0])
        if len(start) != 3:
            raise ValueError('~arm_start_pose must have three values, got %r' % (start,))
        self.start_pose = tuple(float(v) for v in start)

        extrinsics_file = rospy.get_param('~extrinsics_file', '')
        if extrinsics_file:
            self.extrinsics = self._load_extrinsics(os.path.expanduser(extrinsics_file))
        calibration_file = rospy.get_param('~calibration_file', '')
        if calibration_file and self.extrinsics is None:
            self.planar = self._load_planar(os.path.expanduser(calibration_file))
        if self.extrinsics is None and self.planar is None:
            rospy.logwarn('No extrinsics or planar calibration; targets will be published in '
                          'camera frame only and cannot be executed.')

        self.arm_pub = rospy.Publisher('/position_write_topic', position, queue_size=1)
        self.pump_pub = rospy.Publisher('/pump_topic', status, queue_size=1)
        # M17 attach。swiftpro_read_node 启动时会发 M2019 让电机松开，
        # 一旦电机被 detach，G0 会被固件静默丢弃（串口照常写入、臂纹丝不动），
        # 所以执行抓取前必须自己再 attach 一次，不能依赖驱动启动时那一次。
        self.status_pub = rospy.Publisher('/swiftpro_status_topic', status, queue_size=1)
        self.target_pub = rospy.Publisher('/suat/arm/target', String, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher('/suat/arm/debug_image', Image, queue_size=1)
        self.marker_pub = rospy.Publisher('/suat/arm/marker', Marker, queue_size=1)

        rospy.Subscriber(info_topic, CameraInfo, self._on_info, queue_size=1)
        rospy.Subscriber(objects_topic, Detection2DArray, self._on_objects, queue_size=1)
        rospy.Subscriber('/suat/arm/command', String, self._on_command, queue_size=1)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [message_filters.Subscriber(self.color_topic, Image),
             message_filters.Subscriber(self.depth_topic, Image)],
            queue_size=5, slop=0.08)
        self.sync.registerCallback(self._on_frame)
        rospy.loginfo('arm_vision_grasp ready; execute=%s mode=%s',
                      self.execute,
                      'handeye' if self.extrinsics else ('planar' if self.planar else 'none'))

    @staticmethod
    def _load_extrinsics(path):
        with open(path, 'r') as handle:
            content = handle.read().strip()
        try:
            values = json.loads(content)['values']
        except (ValueError, KeyError, TypeError):
            values = [float(v) for v in content.split()]
        return Extrinsics.from_values(values)

    @staticmethod
    def _load_planar(path):
        from suat_mission.manipulation.calibration import PlanarCalibration
        with open(path, 'r') as handle:
            return PlanarCalibration.from_values(handle.read().split())

    def _on_info(self, msg):
        if self.intrinsics is None:
            try:
                self.intrinsics = CameraIntrinsics.from_k(msg.K)
                rospy.loginfo('camera intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f',
                              self.intrinsics.fx, self.intrinsics.fy,
                              self.intrinsics.cx, self.intrinsics.cy)
            except ValueError as exc:
                rospy.logerr('rejected camera info: %s', exc)

    def _on_frame(self, color_msg, depth_msg):
        received = time.monotonic()
        color_stamp = color_msg.header.stamp.to_sec()
        depth_stamp = depth_msg.header.stamp.to_sec()
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except CvBridgeError as exc:
            self._discard_frame()
            rospy.logwarn_throttle(5.0, 'image conversion failed: %s', exc)
            return
        if depth.dtype != np.uint16:
            self._discard_frame()
            rospy.logwarn_throttle(10.0, 'aligned depth is %s, expected uint16 millimetres',
                                   depth.dtype)
            return
        with self.lock:
            if self.latest_frame is not None and color_stamp < self.latest_frame[0]:
                return
            self.latest_frame = (color_stamp, color, depth, depth_stamp, received)
            self.latest_target = None
        self._update_target(color_msg.header)

    def _discard_frame(self):
        with self.lock:
            self.latest_frame = self.latest_target = None

    def _on_objects(self, msg):
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            if self.latest_detections is not None and stamp < self.latest_detections[0]:
                return
            self.latest_detections = (stamp, msg, time.monotonic())
            self.latest_target = None
        self._update_target(msg.header)

    def _update_target(self, header):
        with self.lock:
            frame = self.latest_frame
            detections = self.latest_detections
        if frame is None or detections is None or self.intrinsics is None:
            return
        frame_stamp, color, depth, depth_stamp, _ = frame
        detect_stamp, msg, _ = detections
        stamps = (frame_stamp, detect_stamp, depth_stamp)
        if min(stamps) <= 0 or max(stamps) - min(stamps) > self.frame_slack:
            return

        best = None
        for item in msg.detections:
            label = next((COCO_IDS.get(h.id) for h in item.results if COCO_IDS.get(h.id)), None)
            if label is None or label not in self.labels:
                continue
            bbox = (item.bbox.center.x, item.bbox.center.y, item.bbox.size_x, item.bbox.size_y)
            try:
                point = localize(depth, bbox, self.intrinsics,
                                 self.extrinsics, self.depth_cfg)
            except ValueError as exc:
                rospy.loginfo_throttle(2.0, '%s rejected: %s', label, exc)
                continue
            arm_xyz = self._to_arm(point)
            reach = self._reach_check(arm_xyz)
            if best is None or point.quality > best['point'].quality:
                best = {'label': label, 'point': point, 'arm': arm_xyz,
                        'reach': reach, 'bbox': bbox}
        if best is None:
            return

        payload = self.publish_target(best['label'], best['point'], best['arm'],
                                      best['reach'], frame, detections, best['bbox'][2:])
        if payload is None:
            return
        if best['arm'] is None:
            rospy.loginfo_throttle(5.0, '%s located in camera frame only: depth=%.1f mm',
                                   best['label'], best['point'].z_mm)
        elif best['reach'] is not None and not best['reach'].ok:
            rospy.loginfo_throttle(2.0, '%s not graspable: %s',
                                   best['label'], best['reach'].reason)
        self._publish_debug(color, best, payload)

    def _grasp_z(self):
        """抓取高度：知道支撑面就按方块高度算，否则用单点标定值。"""
        if self.support_z is None:
            return self.grasp_z_default
        return grasp_height(self.support_z, self.block_height)

    def _to_arm(self, point):
        if point.arm_xyz_mm is not None:
            return point.arm_xyz_mm
        if self.planar is None:
            return None
        try:
            x, y = self.planar.project(point.u, point.v,
                                       (self.limits.x_min, self.limits.x_max,
                                        self.limits.y_min, self.limits.y_max))
        except ValueError as exc:
            rospy.loginfo_throttle(2.0, 'planar projection rejected: %s', exc)
            return None
        return (x, y, self._grasp_z())

    def _reach_check(self, arm_xyz):
        if arm_xyz is None:
            return None
        return check_point(arm_xyz[0], arm_xyz[1], arm_xyz[2],
                           margin_mm=self.reach_margin,
                           z_min_mm=self.limits.z_min,
                           z_max_mm=self.limits.z_max,
                           front_only=self.front_only)

    def _fresh_target(self):
        # Called under self.lock. ROS time covers source age, monotonic time covers a stalled clock.
        target = self.latest_target
        if target is None or not target['accepted'] or self.latest_frame is None or self.latest_detections is None:
            return None
        now = rospy.Time.now().to_sec()
        wall_age = time.monotonic() - min(self.latest_frame[4], self.latest_detections[2])
        if (0 <= now - target['stamp'] <= self.target_max_age and now >= target['newest_stamp']
                and 0 <= wall_age <= self.target_max_age):
            return target
        return None

    def _on_command(self, msg):
        if msg.data.strip().lower() != 'pick':
            return
        if not self.execute:
            rospy.logwarn('pick ignored: node is in dry-run mode (~execute:=false)')
            return
        with self.lock:
            if self.busy:
                rospy.logwarn('pick ignored: a motion is already running')
                return
            target = self._fresh_target()
            if target is None:
                rospy.logwarn('pick ignored: no fresh synchronized accepted target')
                return
            self.busy = True
        threading.Thread(target=self._run_pick, args=(target,), daemon=True).start()

    def _attach_motor(self):
        """发 M17 让电机上力。

        swiftpro_read_node 一启动就会发 M2019（松开电机），只要它跑过，
        电机就处于 detach 状态、后续 G0 全部被固件丢弃，
        表现是"指令写进串口了、蜂鸣器也响、但机械臂一毫米都不动"。
        """
        if not self.attach_motor:
            return
        for _ in range(3):
            self.status_pub.publish(status(1))
            rospy.sleep(0.15)
        rospy.sleep(max(0.0, self.attach_settle_s))
        rospy.loginfo('motor attached (M17 x3) before motion')

    def _run_pick(self, target):
        pump_on = False
        try:
            self._attach_motor()
            until = time.monotonic() + self.target_max_age
            current = None
            while time.monotonic() < until and not rospy.is_shutdown():
                with self.lock:
                    current = self._fresh_target()
                if current is not None:
                    break
                time.sleep(.02)
            if (current is None or current['label'] != target['label']
                    or abs(current['u'] - target['u']) > target['bbox_size'][0] / 2
                    or abs(current['v'] - target['v']) > target['bbox_size'][1] / 2):
                raise ValueError('target lost or changed before arm motion')
            target = current
            grasp = (target['arm_xyz'][0], target['arm_xyz'][1],
                     target['arm_xyz'][2] + self.grasp_z_offset)
            pts = plan_pick(self.start_pose, grasp, self.limits, self.path_cfg)
            rospy.loginfo('pick path: %s', path_summary(pts))
            for index, waypoint in enumerate(pts):
                self.arm_pub.publish(position(waypoint.x, waypoint.y, waypoint.z))
                rospy.sleep(max(0.02, waypoint.dwell_s))
                last_descend = (waypoint.stage == 'descend' and
                                (index + 1 >= len(pts) or pts[index + 1].stage != 'descend'))
                if last_descend:
                    rospy.loginfo('seal at (%.1f, %.1f, %.1f)', waypoint.x, waypoint.y, waypoint.z)
                    self.pump_pub.publish(status(1))
                    pump_on = True
                    rospy.sleep(max(0.0, self.path_cfg.seal_hold_s))
            rospy.loginfo('pick sequence done; suction %s', 'on' if pump_on else 'off')
        except Exception as exc:
            rospy.logerr('pick aborted: %s', exc)
        finally:
            with self.lock:
                self.busy = False

    def publish_target(self, label, point, arm_xyz, reach, frame, detections, bbox_size):
        reach_ok = reach.ok if reach is not None else True
        accepted = (point.quality >= self.min_quality and arm_xyz is not None
                    and reach_ok)
        payload = {
            'label': label,
            'u': round(point.u, 1),
            'v': round(point.v, 1),
            'depth_mm': round(point.z_mm, 1),
            'camera_xyz': [round(v, 1) for v in point.camera_xyz_mm],
            'arm_xyz': [round(v, 1) for v in arm_xyz] if arm_xyz else None,
            'quality': point.quality,
            'spread_mm': round(point.spread_mm, 1),
            'residual_mm': round(point.residual_mm, 1),
            'tilt_deg': round(point.tilt_deg, 1),
            'block_height_mm': self.block_height,
            'grasp_z_mm': round(self._grasp_z(), 1),
            'reach_ok': reach_ok,
            'reach_reason': reach.describe() if reach is not None else 'no arm frame',
            'accepted': accepted,
            'stamp': min(frame[0], frame[3], detections[0]),
            'newest_stamp': max(frame[0], frame[3], detections[0]),
            'bbox_size': [float(point_size) for point_size in bbox_size],
        }
        with self.lock:
            if frame is not self.latest_frame or detections is not self.latest_detections:
                return None
            self.latest_target = payload
        self.target_pub.publish(String(data=json.dumps(payload)))
        self._publish_marker(arm_xyz, accepted)
        return payload

    def _publish_debug(self, color, best, payload):
        image = color.copy()
        cx, cy, width, height = best['bbox']
        point = best['point']
        cv2.rectangle(image, (int(cx - width / 2), int(cy - height / 2)),
                      (int(cx + width / 2), int(cy + height / 2)), (0, 200, 255), 2)
        cv2.drawMarker(image, (int(round(point.u)), int(round(point.v))),
                       (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
        lines = [best['label'], 'depth %.1f mm' % point.z_mm,
                 'quality %.2f  tilt %.1f deg' % (point.quality, point.tilt_deg),
                 'residual %.1f mm' % point.residual_mm]
        if payload['arm_xyz']:
            lines.append('arm %.1f %.1f %.1f' % tuple(payload['arm_xyz']))
            lines.append('reach %s' % ('OK' if payload['reach_ok'] else 'OUT: ' +
                                       payload['reach_reason'][:28]))
        else:
            lines.append('no arm frame')
        for index, text in enumerate(lines):
            cv2.putText(image, text, (12, 26 + index * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        try:
            message = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, 'debug image failed: %s', exc)
            return
        message.header.stamp = rospy.Time.now()
        self.debug_pub.publish(message)

    def _publish_marker(self, arm_xyz, accepted):
        marker = Marker()
        marker.header.frame_id = rospy.get_param('~marker_frame', 'base_link')
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02
        marker.color.a = 0.9
        marker.color.r = 0.1 if accepted else 0.9
        marker.color.g = 0.8 if accepted else 0.4
        marker.color.b = 0.2
        if arm_xyz:
            marker.pose.position = Point(arm_xyz[0] / 1000.0, arm_xyz[1] / 1000.0,
                                         arm_xyz[2] / 1000.0)
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('arm_vision_grasp')
    ArmVisionGrasp()
    rospy.spin()
