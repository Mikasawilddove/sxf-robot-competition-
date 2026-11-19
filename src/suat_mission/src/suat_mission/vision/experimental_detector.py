#!/usr/bin/env python3
"""Read-only ROS experiment; default output is isolated from the task node."""
import os

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from suat_mission.vision.experimental_vision import ExperimentalVision
from suat_mission.vision.perception import COCO_IDS


class Node:
    def __init__(self):
        cv2.setNumThreads(2)
        paths = rospy.get_param('~templates')
        templates = {}
        for label, names in paths.items():
            templates[label] = [cv2.imread(os.path.expanduser(name)) for name in names]
        self.detector = ExperimentalVision(templates,
                                          max_shape_distance=float(rospy.get_param('~banana_shape_max', .20)),
                                          banana_saturation_min=int(rospy.get_param('~banana_saturation_min', 80)))
        self.ids = {name: number for number, name in COCO_IDS.items()}
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', '/suat/experimental_objects'),
                                   Detection2DArray, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~image_topic', '/camera/color/image_raw'),
                                    Image, self.callback, queue_size=1, buff_size=2**24)

    def callback(self, message):
        try:
            found = self.detector.detect(self.bridge.imgmsg_to_cv2(message, 'bgr8'))
        except (CvBridgeError, ValueError, cv2.error) as error:
            rospy.logwarn_throttle(5.0, 'experimental detector: %s', error)
            return
        output = Detection2DArray()
        output.header = message.header
        for d in found:
            item = Detection2D()
            item.header = message.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id, hypothesis.score = self.ids[d.label], d.score
            item.results = [hypothesis]
            item.bbox.center.x, item.bbox.center.y = d.x, d.y
            item.bbox.size_x, item.bbox.size_y = d.width, d.height
            output.detections.append(item)
        self.pub.publish(output)


if __name__ == '__main__':
    rospy.init_node('suat_experimental_detector')
    Node()
    rospy.spin()
