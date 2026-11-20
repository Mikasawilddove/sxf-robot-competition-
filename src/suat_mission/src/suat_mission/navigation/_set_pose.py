#!/usr/bin/env python3
"""把 AMCL 的初始位姿重设到指定点（x y yaw_deg），并打印收敛后的结果。"""
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw_deg = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    # 初始不确定度：默认 σ=10cm、σ_yaw=5°，比 AMCL 默认的 0.5m/12° 收紧得多，
    # 否则粒子会在大范围撒开、要开一段时间才收敛。
    var_xy = float(sys.argv[4]) if len(sys.argv) > 4 else 0.01
    yaw_sigma_deg = float(sys.argv[5]) if len(sys.argv) > 5 else 5.0
    rospy.init_node('set_initial_pose', anonymous=True)

    yaw = math.radians(yaw_deg)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    cov = [0.0] * 36
    cov[0] = cov[7] = var_xy
    cov[35] = math.radians(yaw_sigma_deg) ** 2
    msg.pose.covariance = cov

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                          queue_size=1, latch=True)
    time.sleep(1.0)
    pub.publish(msg)
    time.sleep(0.5)
    pub.publish(msg)
    print('已下发初始位姿 (%.3f, %.3f) yaw=%.1f度' % (x, y, yaw_deg))

    time.sleep(3.0)
    from geometry_msgs.msg import PoseWithCovarianceStamped as P
    cur = rospy.wait_for_message('/amcl_pose', P, timeout=10).pose.pose
    cyaw = 2.0 * math.atan2(cur.orientation.z, cur.orientation.w)
    print('AMCL 收敛到 (%.3f, %.3f) yaw=%.1f度' %
          (cur.position.x, cur.position.y, math.degrees(cyaw)))
    print('与设定差: dx=%.1fmm dy=%.1fmm dyaw=%.1f度' %
          ((cur.position.x - x) * 1000, (cur.position.y - y) * 1000,
           math.degrees(cyaw) - yaw_deg))


if __name__ == '__main__':
    main()
