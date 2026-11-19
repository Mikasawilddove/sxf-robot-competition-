#!/usr/bin/env python3
"""Read-only camera-facing plane filter; never publishes motion commands."""
import cv2
import message_filters
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

from suat_mission.vision.surface_geometry import estimate_reference_plane, height_above_reference, measure_surface


class Node:
    def __init__(self):
        info_topic = rospy.get_param('~camera_info_topic', '/camera/aligned_depth_to_color/camera_info')
        info = rospy.wait_for_message(info_topic, CameraInfo, timeout=10)
        self.intrinsics = (info.K[0], info.K[4], info.K[2], info.K[5])
        self.max_angle = float(rospy.get_param('~max_angle_deg', 40.0))
        self.max_residual = float(rospy.get_param('~max_residual_m', 0.01))
        self.use_reference_plane = bool(rospy.get_param('~use_reference_plane', False))
        self.min_height = float(rospy.get_param('~min_height_m', 0.05))
        self.max_height = float(rospy.get_param('~max_height_m', 0.25))
        if self.min_height >= self.max_height:
            raise ValueError('invalid experimental height limits')
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', '/suat/experimental_top_objects'),
                                   Detection2DArray, queue_size=1)
        objects = message_filters.Subscriber(rospy.get_param('~objects_topic', '/suat/experimental_objects'),
                                             Detection2DArray)
        depth = message_filters.Subscriber(rospy.get_param('~depth_topic', '/camera/aligned_depth_to_color/image_raw'), Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([objects, depth], queue_size=12, slop=0.15)
        self.sync.registerCallback(self.callback)

    def callback(self, objects, depth_msg):
        try:
            if depth_msg.encoding == '16UC1':
                scale = 0.001
            elif depth_msg.encoding == '32FC1':
                scale = 1.0
            else:
                rospy.logwarn_throttle(5, 'unsupported depth encoding: %s', depth_msg.encoding)
                return
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            output = Detection2DArray()
            output.header = objects.header
            plane = estimate_reference_plane(depth, self.intrinsics, depth_scale=scale) if self.use_reference_plane else None
            if self.use_reference_plane and plane is None:
                self.pub.publish(output)
                rospy.logwarn_throttle(5, 'no reliable dominant plane; rejecting experimental detections')
                return
            for item in objects.detections:
                measure = measure_surface(depth, item.bbox.center.x, item.bbox.center.y,
                                          self.intrinsics, depth_scale=scale)
                if measure is None or measure.angle_to_camera_deg > self.max_angle or measure.residual_m > self.max_residual:
                    continue
                if self.use_reference_plane:
                    height = height_above_reference(measure, item.bbox.center.x, item.bbox.center.y, self.intrinsics, plane)
                    if height is None or not self.min_height <= height <= self.max_height:
                        continue
                output.detections.append(item)
            self.pub.publish(output)
            rospy.loginfo_throttle(5, 'surface filter: %d input, %d retained',
                                   len(objects.detections), len(output.detections))
        except (CvBridgeError, ValueError, cv2.error) as error:
            rospy.logwarn_throttle(5, 'surface filter: %s', error)


if __name__ == '__main__':
    rospy.init_node('suat_experimental_surface_filter')
    Node()
    rospy.spin()
