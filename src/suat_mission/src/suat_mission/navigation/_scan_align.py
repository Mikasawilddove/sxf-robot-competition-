#!/usr/bin/env python3
"""量化「雷达点是否压在地图墙上」：把 /scan 变换到 map 系，逐点查地图格子。"""
import math
import time

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('scan_align', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=10)
    scan = rospy.wait_for_message('/scan', LaserScan, timeout=10)

    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    data = np.array(grid.data, dtype=np.int16).reshape(h, w)

    tf = buf.lookup_transform('map', scan.header.frame_id,
                              rospy.Time(0), rospy.Duration(3.0))
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    r = np.array(scan.ranges, dtype=np.float64)
    ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, 6.0))
    r = r[ok]
    ang = ang[ok]

    px = tx + r * np.cos(ang + yaw)
    py = ty + r * np.sin(ang + yaw)
    cx = ((px - ox) / res).astype(int)
    cy = ((py - oy) / res).astype(int)
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    cx, cy = cx[inside], cy[inside]

    vals = data[cy, cx]
    n = len(vals)
    occ = int((vals > 60).sum())
    free = int((vals == 0).sum())
    unk = int((vals < 0).sum())

    # 每个点到最近「占用」格的距离（用 5x5 邻域近似）
    hit = np.zeros(n, dtype=bool)
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            yy = np.clip(cy + dy, 0, h - 1)
            xx = np.clip(cx + dx, 0, w - 1)
            hit |= data[yy, xx] > 60

    print('地图 %dx%d res=%.3f origin=(%.1f,%.1f)' % (w, h, res, ox, oy))
    print('激光帧 %s -> map: 位置(%.3f,%.3f) yaw=%.1f度' %
          (scan.header.frame_id, tx, ty, math.degrees(yaw)))
    print('有效点 %d 个' % n)
    print('  落在墙上(±2格/±10cm): %4d  %5.1f%%' % (int(hit.sum()), 100.0 * hit.sum() / max(n, 1)))
    print('  落在空闲区        : %4d  %5.1f%%' % (free, 100.0 * free / max(n, 1)))
    print('  落在未知区        : %4d  %5.1f%%' % (unk, 100.0 * unk / max(n, 1)))
    if n:
        print('  严格落黑格        : %4d  %5.1f%%' % (occ, 100.0 * occ / n))


if __name__ == '__main__':
    main()
