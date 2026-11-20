#!/usr/bin/env python3
"""Record measured map->base_footprint poses without commanding the robot."""
import math
import os
import tempfile

import rospy
import tf2_ros
import yaml
from std_msgs.msg import String

VALID = {'Classification_area', 'Sorting_N', 'Sorting_W',
         'Collection_A', 'Collection_B', 'Collection_C', 'Collection_D'}


class Recorder:
    def __init__(self):
        self.path = os.path.expanduser(rospy.get_param('~output', '~/.ros/suat_waypoints.yaml'))
        self.max_age = float(rospy.get_param('~max_tf_age', 1.0))
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.sub = rospy.Subscriber('/suat/record_waypoint', String, self.record, queue_size=1)
        rospy.loginfo('Waypoint recorder ready; output: %s', self.path)

    def record(self, msg):
        name = msg.data.strip()
        if name not in VALID:
            rospy.logerr('Unknown waypoint %s; valid: %s', name, sorted(VALID))
            return
        try:
            transform = self.buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
            age = (rospy.Time.now() - transform.header.stamp).to_sec()
            if age < -0.5 or age > self.max_age:
                raise ValueError('map pose is stale: age {:.2f}s'.format(age))
            trans = transform.transform.translation
            q = transform.transform.rotation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if not math.isfinite(norm) or abs(norm - 1.0) > .01:
                raise ValueError('invalid map pose quaternion')
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            if not all(math.isfinite(v) for v in (trans.x, trans.y, yaw)):
                raise ValueError('non-finite map pose')
            data = {'waypoints': {}}
            if os.path.exists(self.path):
                with open(self.path, 'r') as f:
                    data = yaml.safe_load(f) or data
                if not isinstance(data, dict) or not isinstance(data.get('waypoints'), dict):
                    raise ValueError('existing waypoint file has invalid format')
            data['waypoints'][name] = [float(trans.x), float(trans.y), float(yaw)]
            folder = os.path.dirname(os.path.abspath(self.path))
            os.makedirs(folder, exist_ok=True)
            fd, temp_path = tempfile.mkstemp(prefix='.suat-waypoints-', suffix='.yaml', dir=folder)
            try:
                with os.fdopen(fd, 'w') as f:
                    yaml.safe_dump(data, f, allow_unicode=True, sort_keys=True)
                os.replace(temp_path, self.path)
            finally:
                if os.path.exists(temp_path):
                    os.unlink(temp_path)
            rospy.loginfo('Saved %s = %s', name, data['waypoints'][name])
        except (ValueError, OSError, yaml.YAMLError, tf2_ros.TransformException) as exc:
            rospy.logerr('Could not record waypoint: %s', exc)


if __name__ == '__main__':
    rospy.init_node('suat_waypoint_recorder')
    Recorder()
    rospy.spin()
