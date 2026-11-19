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
        self.matcher = TemplateMatcher(match_templates, scales,
                                       rospy.get_param('~rotations', [0]),
                                       float(rospy.get_param('~threshold', 0.8)))
        self.cake_matcher = CakeFeatureMatcher(templates['cake']) if self.method == 'hybrid' else None
        self.ids = {value: key for key, value in COCO_IDS.items()}
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/objects', Detection2DArray, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~image_topic', '/camera/color/image_raw'),
                                    Image, self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
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


if __name__ == '__main__':
    rospy.init_node('suat_template_detector')
    Detector()
    rospy.spin()
