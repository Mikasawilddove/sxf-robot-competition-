#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""识别前先把机械臂挪开——D435 装在臂上，臂身不移开会挡住分类区最右边那个码。

实测（2026-11-19，机器人实拍 640x480）：臂身压掉第三个码的一角，外接矩形被拉长到
1.47、实心度掉到 0.62，三个码只能检出两个。人眼一看码是齐的，算法却少一个，
这类"少一个"的 bug 在赛场上就是配错区。

运动方式和 arm_pick_place.py 一致：发 /position_write_topic(swiftpro/position)，
先给 /swiftpro_status_topic 发 1 上力，再走"先抬到安全高度、再同一高度横移"，
避免扫过桌面上的方块。

用法（在机器人上，先 source 工作空间）：

    python3 zone_arm.py --read                     # 只读当前姿态，不动
    python3 zone_arm.py --park --dry-run           # 只打印计划，不动
    python3 zone_arm.py --park                     # 真的挪到最左
    python3 zone_arm.py --park --pose 60,170,150   # 换一侧
"""
import argparse
import math
import sys
import time

# 默认避让位：贴到 y 负方向最边上、抬高到 150mm。x 留 60 保证在可达范围内
# （ArmLimits: x 40~280, y -180~180, z -80~200）。
PARK_POSE = (60.0, -170.0, 150.0)
SAFE_Z = 150.0
POSITION_TOPIC = '/position_write_topic'
STATUS_TOPIC = '/swiftpro_status_topic'
STATE_TOPIC = '/SwiftproState_topic'

_PUB_CACHE = None          # (pos_pub, status_pub)，建一次复用


def _ros():
    """延迟导入 ROS：没装 ROS 的机器上，zone_assign 的离线路径照样能跑。"""
    import rospy
    from swiftpro.msg import position, status
    return rospy, position, status


def _node():
    rospy, _, _ = _ros()
    if not rospy.core.is_initialized():
        rospy.init_node('zone_arm', anonymous=True, disable_signals=True)
    return rospy


def read_pose(timeout=3.0):
    """读一次 /SwiftproState_topic 的 x,y,z；读不到返回 None。

    驱动只在收过指令后才持续发状态，所以没动过臂的时候这里多半是空的，
    不是故障，调用方必须能处理 None。
    """
    rospy, _, _ = _ros()
    from swiftpro.msg import SwiftproState
    _node()
    try:
        state = rospy.wait_for_message(STATE_TOPIC, SwiftproState, timeout=timeout)
    except rospy.ROSException:
        return None
    return (float(state.x), float(state.y), float(state.z))


def _publishers(settle=1.5):
    """建好两个发布者并等连接真正建立。

    rospy 的坑：Publisher 刚建就 publish，订阅端还没连上，消息直接丢，
    不报错也不阻塞。第一次实测就是这么静默失败的——姿态一直停在 200,0,150。
    arm_pick_place.py 的 Arm.__init__ 里那句 sleep(settle) 就是这个作用，
    这里照抄，并且额外查 get_num_connections()。
    """
    global _PUB_CACHE
    if _PUB_CACHE is not None:
        return _PUB_CACHE
    rospy, position, status = _ros()
    _node()
    pos_pub = rospy.Publisher(POSITION_TOPIC, position, queue_size=1)
    st_pub = rospy.Publisher(STATUS_TOPIC, status, queue_size=1)
    time.sleep(settle)
    deadline = time.time() + 5.0
    while time.time() < deadline and pos_pub.get_num_connections() == 0:
        time.sleep(0.1)
    if pos_pub.get_num_connections() == 0:
        print('  警告: %s 没有订阅者，指令会丢（检查 swiftpro_write_node 是否在跑）'
              % POSITION_TOPIC, file=sys.stderr)
    _PUB_CACHE = (pos_pub, st_pub)
    return _PUB_CACHE


def _attach(hold=0.8):
    _, st_pub = _publishers()
    status = _ros()[2]
    for _ in range(3):                 # 和 arm_pick_place 一样连发三次，发一次有时不生效
        st_pub.publish(status(1))
        time.sleep(0.2)
    time.sleep(hold)


def _send(x, y, z, dwell):
    pos_pub, _ = _publishers()
    position = _ros()[1]
    # 连发三次：单发丢过一次，多发的代价只是驱动按同一目标重算一遍
    for _ in range(3):
        pos_pub.publish(position(float(x), float(y), float(z)))
        time.sleep(0.2)
    print('    arm -> (%7.1f, %7.1f, %7.1f)  等 %.1fs' % (x, y, z, dwell), flush=True)
    time.sleep(dwell)


def _dwell(distance, speed=45.0, pad=1.2):
    return max(1.5, distance / speed + pad)


def plan(current, park_pose=PARK_POSE, safe_z=SAFE_Z):
    """算出"先抬再横移"的两个路点；已在高处就只走一步。"""
    park_pose = tuple(float(v) for v in park_pose)
    steps = []
    if current is None:
        steps.append(('未知当前高度，直接去避让位（若臂在低位，请先手动抬高）', park_pose))
        return steps
    cur_x, cur_y, cur_z = current
    if cur_z < safe_z - 5:
        steps.append(('原地抬到安全高度', (cur_x, cur_y, safe_z)))
    steps.append(('横移到避让位', park_pose))
    return steps


def park(park_pose=PARK_POSE, safe_z=SAFE_Z, dry_run=False, verify=True,
         timeout=3.0):
    """把机械臂挪到避让位。返回 True 表示已经到（或 dry-run 打完了计划）。"""
    park_pose = tuple(float(v) for v in park_pose)
    current = read_pose(timeout)
    steps = plan(current, park_pose, safe_z)

    print('当前姿态: %s' % ('%.1f, %.1f, %.1f' % current if current else '读不到'), flush=True)
    for note, point in steps:
        print('  计划: %-24s -> (%7.1f, %7.1f, %7.1f)'
              % (note, point[0], point[1], point[2]), flush=True)
    print('  避让位: (%7.1f, %7.1f, %7.1f)  y=%+.0f 贴到一侧最边'
          % (park_pose[0], park_pose[1], park_pose[2], park_pose[1]), flush=True)

    if dry_run:
        print('  dry-run：没有发任何运动指令', flush=True)
        return True

    _attach()
    if current is None:
        # 状态读不到就再试一次；还是读不到就按"已经在安全高度"处理，直接横移，
        # 但把这句显式打出来，出问题时能看到是哪一步没确认
        current = read_pose(timeout)
        print('  上力后读姿态: %s'
              % ('%.1f, %.1f, %.1f' % current if current else '仍然读不到'), flush=True)

    for note, point in plan(current, park_pose, safe_z):
        reference = current if current else point
        distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(point, reference)))
        _send(point[0], point[1], point[2], _dwell(distance))
        current = point

    time.sleep(0.4)
    if verify:
        after = read_pose(timeout)
        if after:
            offset = math.sqrt(sum((a - b) ** 2 for a, b in zip(after, park_pose)))
            print('到达姿态: (%.1f, %.1f, %.1f)  与目标差 %.1f mm'
                  % (after + (offset,)), flush=True)
            if offset > 25:
                print('  注意: 偏差偏大，可能被限位/力矩挡住', file=sys.stderr)
        else:
            print('到达姿态: 读不到（不影响后续识别，但建议肉眼确认', flush=True)
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description='识别前把机械臂挪到一侧，别挡相机')
    parser.add_argument('--read', action='store_true', help='只读当前姿态')
    parser.add_argument('--park', action='store_true', help='挪到避让位')
    parser.add_argument('--pose', default='', help='避让位 x,y,z，默认 %s' % (PARK_POSE,))
    parser.add_argument('--safe-z', type=float, default=SAFE_Z, help='抬升时的高度')
    parser.add_argument('--dry-run', action='store_true', help='只打印计划，不发指令')
    args = parser.parse_args(argv)

    pose = PARK_POSE
    if args.pose:
        pose = tuple(float(v) for v in args.pose.split(','))
        if len(pose) != 3:
            print('--pose 需要三个数：x,y,z', file=sys.stderr)
            return 2

    if args.read or not args.park:
        current = read_pose(3.0)
        print('当前姿态: %s' % ('%.1f, %.1f, %.1f' % current if current else '读不到'))
        if not args.park:
            return 0

    park(pose, args.safe_z, dry_run=args.dry_run)
    return 0


if __name__ == '__main__':
    sys.exit(main())
