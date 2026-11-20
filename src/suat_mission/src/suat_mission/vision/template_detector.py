#!/usr/bin/env python3
"""Publish template matches using the same /objects contract as the demo detector."""
import os

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from suat_mission.vision.perception import ALL_ITEMS, COCO_IDS
from suat_mission.vision.template_matcher import TemplateMatcher
from suat_mission.vision.feature_matcher import CakeFeatureMatcher


class Detector:
    def __init__(self):
        paths = rospy.get_param('~templates', {})
        if set(paths) != set(ALL_ITEMS):
            raise ValueError('provide template images for cake, teddybear, clock and banana')
        templates = {}
        for label, names in paths.items():
            if not isinstance(names, list) or not names:
                raise ValueError('template paths must be nonempty lists')
            templates[label] = []
            for name in names:
                image = cv2.imread(os.path.expanduser(name), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('unable to read template ' + name)
                templates[label].append(image)
        self.method = rospy.get_param('~method', 'template')
        if self.method not in ('template', 'hybrid'):
            raise ValueError('method must be template or hybrid')
        self.image_scale = float(rospy.get_param('~image_scale', 1.0))
        if not 0.25 <= self.image_scale <= 1.0:
            raise ValueError('image_scale must be between 0.25 and 1.0')
        match_templates = {k: v for k, v in templates.items() if self.method != 'hybrid' or k != 'cake'}
        scales = [float(s) * self.image_scale for s in rospy.get_param('~scales', [0.8, 1.0, 1.2])]
        self.threshold = float(rospy.get_param('~threshold', 0.8))
        self.matcher = TemplateMatcher(match_templates, scales,
                                       rospy.get_param('~rotations', [0]),
                                       self.threshold)
        self.cake_matcher = CakeFeatureMatcher(templates['cake']) if self.method == 'hybrid' else None
        self.ids = {value: key for key, value in COCO_IDS.items()}
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/objects', Detection2DArray, queue_size=1)
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.last_log_signature = None
        self.camera_logged = False
        rospy.loginfo('[相机] 图像话题：%s', self.image_topic)
        rospy.loginfo('[识别] 方法：%s；当前阈值：%.3f', self.method, self.threshold)
        self.sub = rospy.Subscriber(self.image_topic,
                                    Image, self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if not self.camera_logged:
                rospy.loginfo('[相机] 分辨率：%d × %d', msg.width, msg.height)
                rospy.loginfo('[相机] 图像状态：正常；最后一帧时间：%.6f s（ROS 时间）',
                              msg.header.stamp.to_sec())
                self.camera_logged = True
            if self.image_scale != 1.0:
                image = cv2.resize(image, None, fx=self.image_scale, fy=self.image_scale,
                                   interpolation=cv2.INTER_AREA)
            detections = self.matcher.detect(image)
            if self.cake_matcher is not None:
                detections.extend(self.cake_matcher.detect(image))
        except (CvBridgeError, ValueError, cv2.error) as exc:
            rospy.logwarn_throttle(5.0, 'Template recognition failed: %s', exc)
            return
        output = Detection2DArray()
        output.header = msg.header
        for d in detections:
            item = Detection2D()
            item.header = msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id, hypothesis.score = self.ids[d.label], d.score
            item.results = [hypothesis]
            item.bbox.center.x, item.bbox.center.y = d.x / self.image_scale, d.y / self.image_scale
            item.bbox.size_x, item.bbox.size_y = d.width / self.image_scale, d.height / self.image_scale
            output.detections.append(item)
        self.pub.publish(output)
        # Quantize centres so normal pixel jitter does not turn this into a per-frame log.
        signature=tuple(sorted((d.label,int(round(d.x/self.image_scale/25.0)),
                                int(round(d.y/self.image_scale/25.0))) for d in detections))
        if signature!=self.last_log_signature:
            self.last_log_signature=signature
            rospy.loginfo('[识别] 目标数量：%d',len(detections))
            for index,d in enumerate(sorted(detections,key=lambda item:(item.label,item.x)),1):
                rospy.loginfo('[识别] 目标 %d：%s，中心像素=(%.1f, %.1f)，框=(%.1f × %.1f)，分数=%.3f',
                              index,d.label,d.x/self.image_scale,d.y/self.image_scale,
                              d.width/self.image_scale,d.height/self.image_scale,d.score)


if __name__ == '__main__':
    rospy.init_node('suat_template_detector')
    Detector()
    rospy.spin()
