#!/usr/bin/env python3
"""Read ALVAR observations for on-site verification; never command motion."""
import math

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers


class Observer:
    def __init__(self):
        self.expected = set(rospy.get_param('~expected_ids', []))
        topic = rospy.get_param('~topic', '/ar_pose_marker')
        self.sub = rospy.Subscriber(topic, AlvarMarkers, self.callback, queue_size=1)
        rospy.loginfo('Observing %s (read-only); expected IDs: %s', topic, sorted(self.expected))

    def callback(self, msg):
        observations = []
        for marker in msg.markers:
            p = marker.pose.pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                continue
            frame = marker.pose.header.frame_id or msg.header.frame_id
            observations.append('id={} frame={} position_m=({:.3f},{:.3f},{:.3f})'.format(
                marker.id, frame, p.x, p.y, p.z))
        seen = {marker.id for marker in msg.markers}
        missing = self.expected - seen
        rospy.loginfo_throttle(2.0, 'markers: %s; expected missing: %s', observations, sorted(missing))


if __name__ == '__main__':
    rospy.init_node('suat_marker_observer')
    Observer()
    rospy.spin()
