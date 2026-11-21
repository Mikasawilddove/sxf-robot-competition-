#!/usr/bin/env python3
"""Stopped-base board check. Records fresh RGB evidence; sends no robot commands."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time


def read_formal_station(args):
    import math
    import rosgraph
    import rospy
    import yaml
    from nav_msgs.msg import Odometry
    from suat_mission.automation.serial_mission import SerialMissionRuntime, resolve_serial_config

    root = args.workspace_root
    config_path = root / 'src/suat_mission/config/mission.yaml'
    config = resolve_serial_config(yaml.safe_load(config_path.read_text(encoding='utf-8')), root)
    station = 'Classification_area__' + args.station
    if args.station not in {candidate['id'] for candidate in config.get('board_station_candidates', ())}:
        raise ValueError('station does not belong to the selected map: ' + args.station)
    args.output.mkdir(parents=True, exist_ok=False)
    config['run_directory'] = str(args.output)
    rospy.init_node('suat_formal_board_check', anonymous=True)
    graph = rosgraph.Master(rospy.get_name())

    def require_stopped():
        if dict(graph.getSystemState()[0]).get('/cmd_vel'):
            raise RuntimeError('base navigation output is still published')
        msg = rospy.wait_for_message('/odom', Odometry, timeout=1.)
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if not (0 <= age <= .4 and math.hypot(linear.x, linear.y) <= .01 and abs(angular.z) <= .03):
            raise RuntimeError('stop and fresh odometry required for formal board reading')

    runtime = SerialMissionRuntime(root, config, None, require_stopped)
    try:
        require_stopped()
        report = asdict(runtime.read_board(args.output.name, station))
    except Exception as exc:
        report = {'accepted': False, 'reason': str(exc)}
    report.update(station=station, motion_goal_sent=False, arm_commands_sent=False)
    (args.output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Board check:',report['accepted'],report['reason'],report.get('mapping',{}),args.output/'result.json')
    return 0 if report['accepted'] else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root',type=Path,default=Path.home()/'SUAT/spark')
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--geometry',type=Path)
    source.add_argument('--station',help='run the formal board reader for this recorded station ID')
    parser.add_argument('--output',type=Path,required=True,help='new evidence directory')
    args=parser.parse_args()
    sys.path.insert(0,str(args.workspace_root/'src/suat_mission/src'))
    if args.station:
        return read_formal_station(args)
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from suat_mission.board_recognition import BoardGeometry,BoardReader,DirectionEvidence
    from suat_mission.template_matcher import TemplateMatcher
    from suat_mission.map_identity import map_identity
    from suat_mission.navigation_map import load_binding

    config=json.loads(args.geometry.read_text())
    if config['map_identity']!=map_identity(load_binding(args.workspace_root)):
        raise ValueError('board geometry belongs to another map/layout')
    geometry=BoardGeometry(tuple(config['image_size']),config['corners'],
                            tuple(config['normalized_size']),config['geometry_id'])
    args.output.mkdir(parents=True,exist_ok=False)
    rospy.init_node('suat_board_camera_check',anonymous=True)
    run_id=args.output.name
    direction=DirectionEvidence(run_id,geometry.geometry_id,'fixed_reference',
                                config['direction_reference'],tuple(config['zones']),
                                config.get('direction_confirmed',False))
    reader=BoardReader(run_id,geometry,direction,started_ns=rospy.Time.now().to_nsec())
    templates={name:[cv2.imread(str(args.workspace_root/'src/suat_mission/assets/templates'/(name+'.png')))]
               for name in ('clock','teddybear','banana')}
    matcher=TemplateMatcher(templates,scales=[.4,.5,.6,.7,.8,.9,1.,1.1,1.2],threshold=.6,max_peaks=8)
    bridge=CvBridge();rows=[];deadline=time.monotonic()+20;last_stamp=reader.started_ns
    print('[相机] 图像话题：/camera/color/image_raw',flush=True)
    print('[识别] 方法：多尺度模板匹配（TM_CCOEFF_NORMED）',flush=True)
    print('[识别] 当前阈值：0.60',flush=True)
    try:
        while len(rows)<5 and time.monotonic()<deadline and not rospy.is_shutdown():
            odom=rospy.wait_for_message('/odom',Odometry,timeout=3)
            if abs(odom.twist.twist.linear.x)>.015 or abs(odom.twist.twist.angular.z)>.03:
                raise RuntimeError('base must be stopped for board reading')
            msg=rospy.wait_for_message('/camera/color/image_raw',Image,timeout=5)
            stamp=msg.header.stamp.to_nsec()
            if stamp<=last_stamp:continue
            last_stamp=stamp
            image=bridge.imgmsg_to_cv2(msg,'bgr8');normalized=geometry.normalize(image)
            print('[相机] 分辨率：%d × %d' % (msg.width,msg.height),flush=True)
            print('[相机] 图像状态：正常',flush=True)
            print('[相机] 最后一帧时间：%.6f s（ROS 时间）' % msg.header.stamp.to_sec(),flush=True)
            started=time.monotonic();matches=matcher.detect_board(normalized)
            elapsed=time.monotonic()-started;detections=matches.pop('detections')
            print('[识别] 检测时间：%.3f s' % elapsed,flush=True)
            print('[识别] 目标数量：%d' % len(detections),flush=True)
            for index,detection in enumerate(detections,1):
                print('[识别] 目标 %d：%s，中心像素=(%.1f, %.1f)，框=(%.1f × %.1f)，分数=%.3f' %
                      (index,detection.label,detection.x,detection.y,detection.width,
                       detection.height,detection.score),flush=True)
            name='frame_%02d.png'%len(rows)
            if not cv2.imwrite(str(args.output/name),image):raise RuntimeError('cannot save raw frame')
            decision=reader.observe(detections,coordinate_space='normalized_board',run_id=run_id,
                                    stamp_ns=stamp,now_ns=rospy.Time.now().to_nsec(),
                                    frame_id=msg.header.frame_id,image_id=name)
            rows.append({'image':name,'sha256':hashlib.sha256((args.output/name).read_bytes()).hexdigest(),
                         'stamp_ns':stamp,'frame_id':msg.header.frame_id,'detection_elapsed_s':elapsed,
                         'detections':[asdict(d) for d in detections],'matching':matches,
                         'decision':asdict(decision)})
        result=reader.result(now_ns=rospy.Time.now().to_nsec())
        print('[分类] 本次对应表：',flush=True)
        for label,zone in sorted(result.mapping.items()):
            print('       %s → %s' % (label,zone),flush=True)
        print('[分类] 对应表状态：%s' % ('完整' if result.accepted else '不完整 / 存在冲突'),flush=True)
        if not result.accepted:
            print('[分类] 状态：暂停，等待重新识别；原因：%s' % result.reason,flush=True)
        report={'status':'VERIFIED_SINGLE_SEQUENCE' if result.accepted else 'FAILED',
                'decision':asdict(result),'frames':rows,'geometry_config':config,
                'geometry_sha256':hashlib.sha256(args.geometry.read_bytes()).hexdigest(),
                'hardware_ready':False,'motion_goal_sent':False,'arm_commands_sent':False}
    except Exception as exc:
        print('[相机] 图像状态：未收到 / 画面过期 / 处理失败',flush=True)
        print('[分类] 状态：暂停，等待重新识别；原因：%s' % exc,flush=True)
        report={'status':'FAILED','error':str(exc),'frames':rows,'hardware_ready':False,
                'motion_goal_sent':False,'arm_commands_sent':False}
    (args.output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=True))
    return 0 if report['status']=='VERIFIED_SINGLE_SEQUENCE' else 2


if __name__=='__main__':raise SystemExit(main())
