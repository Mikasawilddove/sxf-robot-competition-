#!/usr/bin/env python3
"""Bounded stationary AMCL refresh after restoring a known saved pose; no motion."""
import json
import math
from pathlib import Path
import time


def perform_updates(read_odometry, request_update, wait):
    for _ in range(15):
        _stopped_odometry(read_odometry)
        request_update()
        wait(.3)
    return {'status':'UPDATES_COMPLETED','updates':15,'motion_commands':False,
            'accuracy_claim':'NONE; compare scan/map and field identity before navigation'}


def _stopped_odometry(read_odometry):
    v,w,age=read_odometry()
    if not all(math.isfinite(value) for value in (v,w,age)) or abs(v)>=.015 or abs(w)>=.03 or not -.1<=age<=.5:
        raise ValueError('stationary refresh requires fresh, stopped odometry')


def _ros_odometry(rospy):
    from nav_msgs.msg import Odometry
    msg=rospy.wait_for_message('/odom',Odometry,timeout=3)
    return msg.twist.twist.linear.x,msg.twist.twist.angular.z,(rospy.Time.now()-msg.header.stamp).to_sec()


def restore_start_a(root, report_path):
    """Restore a recorded A placement and check stationary scan agreement before navigation."""
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Empty
    from suat_mission.navigation.corner_egress import anchor_check
    from suat_mission.navigation.navigation_map import verify_map
    from suat_mission.navigation.saved_grid import load_saved_grid
    from suat_mission.navigation.scan_alignment import ScanMap, scan_endpoints, yaw_delta
    from suat_mission.navigation.start_pose import read_start_pose, validate_start_pose

    report={'status':'failed', 'motion_commands':False}
    try:
        grid=load_saved_grid(root)
        live=verify_map(rospy.wait_for_message('/map',OccupancyGrid,timeout=5),grid)
        if list(live['data']) != grid['data']:
            raise ValueError('live map cells differ from the selected A map')
        record=read_start_pose(root/'data/waypoints/start_pose_A.yaml')
        anchor=validate_start_pose(record,grid)
        report.update(map_path=grid['map_yaml'], anchor_pose=anchor, resolution_m=grid['resolution'])
        field=ScanMap(grid)
        buffer=tf2_ros.Buffer()
        listener=tf2_ros.TransformListener(buffer)
        rospy.wait_for_service('/request_nomotion_update',timeout=5)
        update=rospy.ServiceProxy('/request_nomotion_update',Empty)
        pub=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
        last_scan=None

        def settle(pose):
            deadline=time.monotonic()+3.
            while not pub.get_num_connections() and time.monotonic()<deadline:
                rospy.sleep(.05)
            if not pub.get_num_connections():
                raise ValueError('AMCL is not subscribed to /initialpose')
            _stopped_odometry(lambda: _ros_odometry(rospy))
            prior=PoseWithCovarianceStamped()
            prior.header.frame_id='map'
            prior.header.stamp=rospy.Time.now()
            prior.pose.pose.position.x,prior.pose.pose.position.y=pose[:2]
            prior.pose.pose.orientation.z=math.sin(pose[2]/2)
            prior.pose.pose.orientation.w=math.cos(pose[2]/2)
            for index,value in zip((0,7,35),record['initial_covariance_diagonal']):
                prior.pose.covariance[index]=value
            pub.publish(prior)
            rospy.sleep(.2)
            perform_updates(lambda: _ros_odometry(rospy),update,rospy.sleep)

        def xy_yaw(target, source, stamp):
            transform=buffer.lookup_transform(target,source,stamp,rospy.Duration(1)).transform
            p,q=transform.translation,transform.rotation
            pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            if not all(math.isfinite(v) for v in pose):
                raise ValueError('non-finite transform: '+source+' to '+target)
            return pose

        def frame():
            nonlocal last_scan
            scan=rospy.wait_for_message('/scan',LaserScan,timeout=3)
            stamp=scan.header.stamp
            if stamp.is_zero() or (last_scan is not None and stamp<=last_scan):
                raise ValueError('waiting for a new stamped laser scan at Start_A')
            if not -.1<=(rospy.Time.now()-stamp).to_sec()<=.5:
                raise ValueError('stale laser scan at Start_A')
            last_scan=stamp
            pose=xy_yaw('map','base_footprint',stamp)
            laser=xy_yaw('base_footprint',scan.header.frame_id.lstrip('/'),stamp)
            _stopped_odometry(lambda: _ros_odometry(rospy))
            points=scan_endpoints(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,laser)
            report.setdefault('frames',[]).append({'stamp':stamp.to_sec(),'map_pose':pose,'laser_pose':laser})
            return points,pose

        try:
            settle(anchor)
            fit_points,fit_pose=frame()
            rospy.sleep(.15)
            reference,baseline=frame()
            anchor_check(baseline,anchor)
            if (math.dist(fit_pose[:2],baseline[:2])>grid['resolution']/2
                    or abs(yaw_delta(fit_pose[2],baseline[2]))>math.radians(3)):
                raise ValueError('AMCL pose is still moving while stopped at Start_A')
            match=field.refine(fit_points,fit_pose,reference,baseline)
            report['scan_match']=match
            points,pose=reference,baseline
            applied=False
            if match['status']=='candidate':
                try:
                    anchor_check(match['candidate_pose'],anchor)
                except ValueError:
                    match['status']='outside_saved_A'
                else:
                    settle(match['candidate_pose'])
                    points,pose=frame()
                    actual=field.score(points,pose)
                    same_scan_baseline=field.score(points,baseline)
                    report['apply_result']={'observed':actual,'baseline_on_same_scan':same_scan_baseline}
                    applied=(actual['within_0_10m']>=.80 and field.free_center(pose)
                             and same_scan_baseline['capped_mean_m']>actual['capped_mean_m'])
                    if not applied:
                        settle(baseline)
                        points,pose=frame()
            anchor_check(pose,anchor)
            final=field.score(points,pose)
            report.update(final_pose=pose,final_fit=final)
            if not field.free_center(pose) or final['within_0_10m']<.80:
                raise ValueError('Start_A does not agree with the live laser and saved map')
            report['status']='scan_refined' if applied else 'amcl_supported'
        finally:
            pub.unregister()
            listener.unregister()
    except (ValueError,rospy.ROSException,rospy.ServiceException,tf2_ros.TransformException) as exc:
        report['error']=str(exc)
        raise RuntimeError('automatic Start_A alignment failed: '+str(exc)+'; details: '+str(report_path)) from exc
    finally:
        report_path.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return report


def main():
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Empty
    from suat_mission.navigation.navigation_map import load_binding,verify_map
    rospy.init_node('suat_localization_settler')
    root=Path(rospy.get_param('~workspace_root','/home/spark/SUAT/spark'))
    binding=load_binding(root)
    verify_map(rospy.wait_for_message('/map',OccupancyGrid,timeout=15),binding)
    rospy.wait_for_service('/request_nomotion_update',timeout=15)
    update=rospy.ServiceProxy('/request_nomotion_update',Empty)
    result=perform_updates(lambda: _ros_odometry(rospy),update,rospy.sleep)
    rospy.loginfo('Saved-pose stationary refresh: %s',json.dumps(result))


if __name__=='__main__':main()
