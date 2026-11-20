#!/usr/bin/env python3
"""Show exact annotated frames during the mission and live RGB between stages."""
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def main():
    rospy.init_node('suat_mission_camera_view')
    bridge = CvBridge()
    frames = {}

    def receive(msg, key):
        frames[key] = (msg, time.monotonic())

    subscribers = [rospy.Subscriber(topic, Image, receive, callback_args=name,
                                    queue_size=1, buff_size=2**24)
                   for name, topic in (
        ('rgb', '/camera/color/image_raw'),
        ('board', '/suat/mission/debug/board'),
        ('grasp', '/suat/mission/debug/grasp'),
    )]

    window = 'Spark-I Mission Camera'
    waiting = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(waiting, 'Waiting for a fresh camera frame', (16, 36),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.imshow(window, waiting)
    cv2.waitKey(1)
    last_frame = None
    canvas = None
    while not rospy.is_shutdown():
        try:
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                break
        except cv2.error:
            break
        now = time.monotonic()
        selected = next(((key, frames[key][0]) for key, timeout in
                         (('board', 3.), ('grasp', 2.), ('rgb', 2.))
                         if key in frames and now - frames[key][1] < timeout), None)
        if selected is not None:
            key, msg = selected
            if msg is not last_frame:
                try:
                    canvas = bridge.imgmsg_to_cv2(msg, 'bgr8')
                    last_frame = msg
                except CvBridgeError as exc:
                    rospy.logwarn_throttle(5., 'Camera preview decoding failed: %s', exc)
                    canvas = None
            if canvas is not None:
                shown = canvas.copy()
                age = (rospy.Time.now() - msg.header.stamp).to_sec()
                caption = ('Live RGB' if key == 'rgb' else 'Mission ' + key) + ' | frame age %.1fs' % age
                cv2.rectangle(shown, (0, shown.shape[0] - 23), (shown.shape[1], shown.shape[0]), (0, 0, 0), -1)
                cv2.putText(shown, caption, (7, shown.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX,
                            .47, (240, 240, 240), 1, cv2.LINE_AA)
                cv2.imshow(window, shown)
        else:
            cv2.imshow(window, waiting)
        cv2.waitKey(1)
        time.sleep(.10)
    for sub in subscribers:
        sub.unregister()


if __name__ == '__main__':
    main()
