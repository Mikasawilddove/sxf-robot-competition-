#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量一次「算出来的点」和「方块实际位置」差多少毫米。

做法：按现在的收敛参数算出抓取点，把吸盘降到抓取高度（不开泵），
拍一张照片存下来。照片里同时有吸盘（橙色喷嘴）和目标方块，
方块的 10cm 边长就是尺子，所以不用任何标定就能量出偏差。

用法：
    python3 _aim_probe.py --target banana --near 230,146 --tag L
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))

from suat_mission.manipulation import arm_pick_place as pp  # noqa: E402

BRIDGE = CvBridge()


def grab():
    c = BRIDGE.imgmsg_to_cv2(
        rospy.wait_for_message('/camera/color/image_raw', Image, timeout=8), 'bgr8')
    d = BRIDGE.imgmsg_to_cv2(
        rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image,
                               timeout=8), 'passthrough').astype(np.float32)
    return c, d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', default='banana')
    ap.add_argument('--near', default='', help='挑离这个像素最近的那个目标')
    ap.add_argument('--tag', default='L')
    ap.add_argument('--home', default='200,0,150')
    ap.add_argument('--observe', default='0,200,60')
    ap.add_argument('--block-z', type=float, default=-132.0)
    ap.add_argument('--grasp-offset', type=float, default=82.0)
    args = ap.parse_args()

    home = pp.parse_xyz(args.home)
    obs = pp.parse_xyz(args.observe)
    near = pp.parse_xy(args.near) if args.near else None

    rospy.init_node('aim_probe', anonymous=True)
    arm = pp.Arm()
    arm.attach()

    print('=== 1. 回初始位 -> 观察位 ===', flush=True)
    arm.goto(home[0], home[1], home[2], tag='初始位')
    arm.goto(home[0], home[1], obs[2], tag='降到观察高度')
    arm.goto(obs[0], obs[1], obs[2], tag='观察位')

    print('=== 2. 识别 ===', flush=True)
    pp.wait_still()
    colour, depth = grab()
    block, _, found = pp.find_named_block(colour, depth, args.target)
    hits = [d for d in found if d.label == args.target]
    if not hits:
        print('没识别到 %s' % args.target, flush=True)
        return 1
    if near is not None:
        hits.sort(key=lambda d: (d.x - near[0]) ** 2 + (d.y - near[1]) ** 2)
        best = hits[0]
        box = (int(round(best.x - best.width / 2)), int(round(best.y - best.height / 2)),
               int(round(best.width)), int(round(best.height)))
        uv = pp._top_face_uv(box, depth) or (best.x, best.y - best.height * 0.32)
        patch = depth[max(0, box[1]):max(0, box[1]) + box[3],
                      max(0, box[0]):max(0, box[0]) + box[2]]
        sel = patch[patch > 0]
        dmm = int(np.median(sel)) if sel.size else 0
        block = {'u': float(uv[0]), 'v': float(uv[1]), 'depth': dmm, 'box': box}

    x, y = pp.pixel_to_arm(block['u'], block['v'], block['depth'], args.block_z)
    raw = (x, y)
    x, y = pp.converge_xy(x, y, args.block_z)
    gz = args.block_z + args.grasp_offset
    print('    方块像素 (%.1f,%.1f) 深度 %d 框 %s' % (block['u'], block['v'],
                                                block['depth'], block['box']),
          flush=True)
    print('    反投影 (%.1f,%.1f) -> 收敛后 (%.1f,%.1f)，抓取高度 z=%.0f'
          % (raw[0], raw[1], x, y, gz), flush=True)

    # 停在顶面上方 12mm，不接触方块：碰一下会把方块推走，量出来的偏差就不真了
    hover = gz + 12.0
    print('=== 3. 悬停到算出来的点（不开泵，停在顶面上方 12mm） ===', flush=True)
    arm.goto(x, y, pp.SAFE_Z, tag='到目标上方')
    arm.descend(x, y, pp.SAFE_Z, hover, step=0.0, feed=pp.FEED_SLOW)
    pp.wait_still()

    print('=== 4. 拍照 ===', flush=True)
    c, d = grab()
    pj = '/tmp/aim_%s.jpg' % args.tag
    pd = '/tmp/aim_%s.npy' % args.tag
    cv2.imwrite(pj, c)
    np.save(pd, d.astype(np.uint16))
    print('    %s  %s' % (pj, pd), flush=True)
    print('    CMD %.1f %.1f %.1f' % (x, y, hover), flush=True)

    print('=== 5. 退回初始位 ===', flush=True)
    arm.ascend(x, y, hover, pp.SAFE_Z + 40.0, feed=pp.FEED_FAST)
    arm.goto(home[0], home[1], home[2], tag='回初始位')
    print('PROBEOK', flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        raise SystemExit(1)
