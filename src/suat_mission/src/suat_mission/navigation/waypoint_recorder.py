#!/usr/bin/env python3
"""Record map-pose station candidates; task-purpose acceptance remains separate."""
import hashlib
import math
import os
from pathlib import Path
import tempfile

import rospy
import tf2_ros
import yaml
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from suat_mission.navigation.navigation_map import load_binding,verify_map,map_snapshot
from suat_mission.navigation.candidate_waypoints import make_candidate,make_mapping_candidate,candidate_document

VALID = {'Classification_area', 'Sorting_N', 'Sorting_W',
         'Collection_A', 'Collection_B', 'Collection_C', 'Collection_D'}


class Recorder:
    def __init__(self):
        self.path = os.path.expanduser(rospy.get_param('~output', '~/.ros/suat_candidate_waypoints.yaml'))
        self.root = Path(rospy.get_param('~workspace_root','/home/spark/SUAT/spark')).resolve()
        self.mapping = rospy.get_param('~mapping_candidate',False) is True
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
            binding=None if self.mapping else load_binding(self.root)
            live=rospy.wait_for_message('/map',OccupancyGrid,timeout=2.)
            snapshot=map_snapshot(live,rospy.get_param('~layout_version','')) if self.mapping else None
            if not self.mapping:verify_map(live,binding)
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
            metadata={'client_time':rospy.get_param('~client_time',''),
                      'purpose':rospy.get_param('~purpose',''),
                      'notes':rospy.get_param('~notes',''),
                      'source_images':rospy.get_param('~source_images',[]),
                      'recorded_robot_epoch':rospy.Time.now().to_sec(),
                      'tf_stamp_robot_epoch':transform.header.stamp.to_sec(),
                      'stationary_confirmation':'NOT_AUTOMATICALLY_VERIFIED'}
            pose=[float(trans.x),float(trans.y),float(yaw)]
            metadata['map_stamp_robot_epoch']=live.header.stamp.to_sec()
            record=make_mapping_candidate(name,pose,snapshot,metadata) if self.mapping else make_candidate(name,pose,binding,metadata)
            data = None
            original=None
            if os.path.exists(self.path):
                original=Path(self.path).read_bytes()
                data=yaml.safe_load(original.decode('utf-8'))
            data=candidate_document(data,record,binding,rospy.get_param('~replace',False) is True)
            folder = os.path.dirname(os.path.abspath(self.path))
            os.makedirs(folder, exist_ok=True)
            if original is not None:
                if Path(self.path).read_bytes() != original:
                    raise ValueError('candidate output changed concurrently; retry after checking it')
                history=Path(self.path+'.history')
                history.mkdir(parents=True,exist_ok=True)
                backup=history/('before_'+hashlib.sha256(original).hexdigest()+'.yaml')
                if not backup.exists():
                    with backup.open('xb') as stream:stream.write(original)
            fd, temp_path = tempfile.mkstemp(prefix='.suat-waypoints-', suffix='.yaml', dir=folder)
            try:
                with os.fdopen(fd, 'w') as f:
                    yaml.safe_dump(data, f, allow_unicode=True, sort_keys=True)
                os.replace(temp_path, self.path)
            finally:
                if os.path.exists(temp_path):
                    os.unlink(temp_path)
            rospy.loginfo('Saved %s = %s', name, data['waypoints'][name])
        except (ValueError, OSError, yaml.YAMLError, tf2_ros.TransformException,rospy.ROSException) as exc:
            rospy.logerr('Could not record waypoint: %s', exc)


if __name__ == '__main__':
    rospy.init_node('suat_waypoint_recorder')
    Recorder()
    rospy.spin()
