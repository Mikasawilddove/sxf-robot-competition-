#!/usr/bin/env python3
"""Record confirmed A-start localization only. Never publishes robot commands."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/suat_mission/src'))
from suat_mission.start_pose import make_start_pose
from suat_mission.navigation_map import load_binding, verify_map


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site-confirmed',action='store_true',help='operator confirmed A, heading D, scan alignment, arm stowed')
    parser.add_argument('--replace',action='store_true')
    parser.add_argument('--session-date',required=True,help='date from the operator PC; robot clock is not trusted')
    args=parser.parse_args()
    if not args.site_confirmed:parser.error('--site-confirmed is required; do not infer A identity from square walls')
    import cv2
    import numpy as np
    import yaml
    import rosgraph,rospy,tf2_ros
    from nav_msgs.msg import OccupancyGrid,Odometry
    from sensor_msgs.msg import LaserScan
    rospy.init_node('suat_start_a_recorder',anonymous=True)
    pubs,subs,services=rosgraph.Master(rospy.get_name()).getSystemState()
    nodes={n for _,ns in pubs+subs+services for n in ns}
    if dict(pubs).get('/cmd_vel',[]) or '/move_base' in nodes:
        raise ValueError('release base control and stop navigation before recording the start')
    if dict(pubs).get('/map',[])!=['/map_server']:
        raise ValueError('single static map_server required')
    target=ROOT/'data/waypoints/start_pose_A.yaml'
    if target.exists() and not args.replace:raise ValueError('A record exists; use explicit --replace after checking it')
    binding=load_binding(ROOT)
    live=rospy.wait_for_message('/map',OccupancyGrid,timeout=3)
    verify_map(live,binding)
    buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
    buffer.lookup_transform('map','base_footprint',rospy.Time(0),rospy.Duration(3))
    poses=[]
    for _ in range(12):
        odom=rospy.wait_for_message('/odom',Odometry,timeout=2)
        if abs(odom.twist.twist.linear.x)>=.015 or abs(odom.twist.twist.angular.z)>=.03:
            raise ValueError('robot is not stationary')
        if not -.1<=(rospy.Time.now()-odom.header.stamp).to_sec()<=.5:
            raise ValueError('stale odometry')
        tr=buffer.lookup_transform('map','base_footprint',rospy.Time(0))
        if not -1.1<=(rospy.Time.now()-tr.header.stamp).to_sec()<=.5:
            raise ValueError('stale map pose')
        p,q=tr.transform.translation,tr.transform.rotation
        if abs(math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1)>.01:
            raise ValueError('invalid pose quaternion')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        poses.append((p.x,p.y,yaw))
    p=poses[-1]
    if max(math.hypot(v[0]-p[0],v[1]-p[1]) for v in poses)>.03 or max(abs(math.atan2(math.sin(v[2]-p[2]),math.cos(v[2]-p[2]))) for v in poses)>math.radians(2):
        raise ValueError('stationary localization is changing; do not record yet')
    scan=rospy.wait_for_message('/scan',LaserScan,timeout=3)
    tr=buffer.lookup_transform('map',scan.header.frame_id,rospy.Time(0))
    q=tr.transform.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    r=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(r))*scan.angle_increment
    ids=np.flatnonzero(np.isfinite(r)&(r>=max(.12,scan.range_min))&(r<=min(4.5,scan.range_max)))[::2]
    if len(ids)<100:raise ValueError('not enough valid scan endpoints')
    x=tr.transform.translation.x+r[ids]*np.cos(angles[ids]+a)
    y=tr.transform.translation.y+r[ids]*np.sin(angles[ids]+a)
    cells=np.asarray(live.data,dtype=np.int8).reshape(live.info.height,live.info.width)
    dist=cv2.distanceTransform((cells<65).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*live.info.resolution
    cols=np.floor((x-live.info.origin.position.x)/live.info.resolution).astype(int)
    rows=np.floor((y-live.info.origin.position.y)/live.info.resolution).astype(int)
    inside=(rows>=0)&(rows<live.info.height)&(cols>=0)&(cols<live.info.width)
    dd=np.full(len(ids),.5);dd[inside]=dist[rows[inside],cols[inside]]
    data=make_start_pose(p,binding,float(np.mean(dd<=.10)),args.session_date,True)
    data['recorded_robot_epoch']=rospy.Time.now().to_sec()
    data['stationary_samples']=len(poses)
    data['source_release']=json.loads((ROOT/'ACTIVE_SOURCE_MANIFEST.json').read_text())['release']
    target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():
        raw=target.read_bytes();old=ROOT/'reports/start_pose_A'/('before_'+hashlib.sha256(raw).hexdigest()[:12]+'.yaml')
        old.parent.mkdir(parents=True,exist_ok=True);old.write_bytes(raw)
    fd,tmp=tempfile.mkstemp(prefix='.start-A-',suffix='.yaml',dir=str(target.parent))
    try:
        with os.fdopen(fd,'w') as f:yaml.safe_dump(data,f,sort_keys=True,allow_unicode=True)
        os.replace(tmp,str(target))
    finally:
        if os.path.exists(tmp):os.unlink(tmp)
    print(json.dumps({'path':str(target),'record':data,'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
                      'motion_commands':False,'navigation_started':False},ensure_ascii=True,indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
