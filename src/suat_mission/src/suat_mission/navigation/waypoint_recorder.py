#!/usr/bin/env python3
"""Record measured map->base_footprint poses without commanding the robot."""
import math
from datetime import datetime, timezone
from pathlib import Path

import rospy
import tf2_ros
import yaml
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from suat_mission.navigation.navigation_map import verify_map
from suat_mission.navigation.pickup_keepout import KEEP_OUT_FILE, PickupKeepout
from suat_mission.navigation.saved_grid import load_saved_grid
from suat_mission.navigation.stations import STATIONS, body_radius, destination_radius, save_candidate


class Recorder:
    def __init__(self):
        self.root = Path(rospy.get_param('~workspace_root',str(Path.home()/'SUAT/spark'))).expanduser()
        session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        self.path = Path(rospy.get_param('~output_directory',
                         str(self.root/'data/waypoint_candidates'/session))).expanduser()
        config = yaml.safe_load((self.root/'src/suat_mission/config/navigation_test.yaml').read_text(encoding='utf-8'))
        self.radius = body_radius(config)
        self.keepout_file = rospy.get_param('~keepout_file', KEEP_OUT_FILE)
        self.max_age = float(rospy.get_param('~max_tf_age', 1.0))
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.sub = rospy.Subscriber('/suat/record_waypoint', String, self.record, queue_size=1)
        rospy.loginfo('Waypoint recorder ready; output: %s', self.path)

    def record(self, msg):
        name = msg.data.strip()
        if name not in STATIONS:
            rospy.logerr('Unknown waypoint %s; valid: %s', name, STATIONS)
            return
        try:
            grid = load_saved_grid(self.root)
            live = rospy.wait_for_message('/map', OccupancyGrid, timeout=2.)
            if list(verify_map(live,grid)['data']) != grid['data']:
                raise ValueError('live map cells differ from the selected saved map')
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
            pose = [trans.x, trans.y, yaw]
            keepout = PickupKeepout.load(self.root, grid, self.keepout_file)
            radius = self.radius if name == 'Start_A' else destination_radius(name,self.radius,keepout)
            if keepout.distance(pose) <= radius:
                raise ValueError('station is too close to central keepout')
            saved = save_candidate(self.path,name,pose,grid,radius,transform.header.stamp.to_nsec())
            rospy.loginfo('Saved %s %s to %s', name, pose, saved)
        except (ValueError, OSError, KeyError, rospy.ROSException, yaml.YAMLError, tf2_ros.TransformException) as exc:
            rospy.logerr('Could not record waypoint: %s', exc)


if __name__ == '__main__':
    rospy.init_node('suat_waypoint_recorder')
    Recorder()
    rospy.spin()
