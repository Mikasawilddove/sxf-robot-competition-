#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证「自动避让放置」的扫桌面 + 选点：只走位拍照，不抓任何东西。

和 arm_pick_place.py 的 auto_place 用同一套函数（scan_table / free_spot /
stack_target / draw_scan），所以这里量出来的框和落点就是实跑时会用的。

用法：
    python3 _place_probe.py                 # 观察位拍一帧，打印框与落点
    python3 _place_probe.py --lift 130      # 在抬升高度拍（臂不挡镜头时的视角）
    python3 _place_probe.py --crowd 6
产出：
    /tmp/place_probe.jpg   绿框=检出的方块，红框=叠层目标，黄框=落点
    /tmp/place_probe.npy   深度图，可以拉回本地离线复算
"""
import argparse
import os
import sys

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
                               timeout=8), 'passthrough')
    return c, d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--home', default='200,0,150')
    ap.add_argument('--observe', default='0,200,60')
    ap.add_argument('--lift', type=float, default=0.0,
                    help='只在抬升高度拍（不降到观察高度）。0 表示降到 observe 的 z')
    ap.add_argument('--block-z', type=float, default=-132.0)
    ap.add_argument('--grasp-offset', type=float, default=82.0)
    ap.add_argument('--stack-height', type=float, default=100.0)
    ap.add_argument('--layer-rise', type=float, default=110.0)
    ap.add_argument('--crowd', type=int, default=6)
    ap.add_argument('--place-gap', type=float, default=20.0)
    ap.add_argument('--min-area', type=float, default=2000.0)
    ap.add_argument('--hold', default='none',
                    help='none=手上没东西；给了 "x,y" 就当成吸盘上挂着的那块，'
                         '选点时按这个位置剔掉')
    args = ap.parse_args()

    home = pp.parse_xyz(args.home)
    obs = pp.parse_xyz(args.observe)
    hold = pp.parse_xy(args.hold) if args.hold.strip().lower() not in ('', 'none') else None

    rospy.init_node('place_probe', anonymous=True)
    arm = pp.Arm()
    arm.attach()

    print('=== 1. 回初始位 -> 摆到最左 ===', flush=True)
    arm.goto(home[0], home[1], home[2], tag='初始位')
    z = args.lift if args.lift > 0 else obs[2]
    arm.goto(home[0], home[1], z, tag='升降到拍摄高度')
    obs_deg = np.degrees(np.arctan2(obs[1], obs[0]))
    obs_r = float(np.hypot(obs[0], obs[1]))
    arm.swing(obs_r, z, np.degrees(np.arctan2(home[1], home[0])), float(obs_deg))
    grip = (obs_r * np.cos(np.radians(obs_deg)), obs_r * np.sin(np.radians(obs_deg)))

    pp.wait_still()
    colour, depth = grab()
    cv2.imwrite('/tmp/place_probe.jpg', colour)
    np.save('/tmp/place_probe.npy', depth)

    print('=== 2. 扫桌面（avoid=%s）===' % str(hold or grip), flush=True)
    blocks = pp.scan_table(depth, args.block_z, args.stack_height, args.layer_rise,
                           avoid_xy=hold or grip, min_area=args.min_area)
    print('    框出 %d 个方块：' % len(blocks), flush=True)
    for i, b in enumerate(blocks):
        bx, by, bw, bh = b['box']
        print('      #%d 框=(%d,%d) %dx%d 面积=%.0f 层=%d 顶面z=%.0f 深度=%d '
              '-> 臂(%.0f,%.0f) r=%.0f'
              % (i, bx, by, bw, bh, b['area'], b['layer'], b['top_z'],
                 b['depth'], b['x'], b['y'], b['r']), flush=True)

    spot = None
    tgt = None
    pz = args.block_z + args.grasp_offset
    crowded = len(blocks) >= args.crowd
    if crowded:
        tgt = pp.stack_target(blocks)
        print('    方块数 %d >= %d，走叠第二层：%s'
              % (len(blocks), args.crowd,
                 '无可用目标' if tgt is None else '(%.0f,%.0f)'
                 % (tgt['x'], tgt['y'])), flush=True)
        if tgt is not None:
            spot = (tgt['x'], tgt['y'])
            pz += args.stack_height
    if spot is None:
        fs = pp.free_spot(blocks, args.place_gap)
        print('    找空地：%s' % ('无' if fs is None else
                             '(%.1f, %.1f) 到最近轮廓 %.1fmm' % fs), flush=True)
        if fs is not None:
            spot = (fs[0], fs[1])

    if spot is None:
        print('    一个落点都没算出来（真跑时会退回 --place-default）', flush=True)
    else:
        surface_z = pz - args.grasp_offset - args.stack_height
        print('    === 落点 (%.1f, %.1f, z=%.1f) 承放面 z=%.1f ==='
              % (spot[0], spot[1], pz, surface_z), flush=True)
        path = pp.draw_scan(colour, blocks, spot, surface_z, tgt,
                            '/tmp/place_probe_annot.jpg')
        print('    标注图: %s' % path, flush=True)
    print('    原图: /tmp/place_probe.jpg   深度: /tmp/place_probe.npy', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
