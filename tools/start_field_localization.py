#!/usr/bin/env python3
"""Check the saved A start and exclusivity; optionally launch the minimal field stack."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/suat_mission/src'))
from suat_mission.navigation_map import load_binding
from suat_mission.map_identity import map_identity
from suat_mission.start_pose import read_start_pose,validate_start_pose
from suat_mission.navigation.field_alignment import static_transform_args


def open_rviz_when_ready(root, finished, display=':0', timeout=20.0):
    """Open the existing visualization after this launch starts map and AMCL."""
    config=root/'reports/navigation_20261115_46dd5d73a659/new_map_localization.rviz'
    if not config.is_file():
        print('[RViz] 配置文件不存在：%s；定位栈继续运行' % config, file=sys.stderr, flush=True)
        return
    master=xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'))
    deadline=time.monotonic()+timeout
    while not finished.is_set() and time.monotonic()<deadline:
        try:
            code,_,state=master.getSystemState('/suat_field_rviz_start')
            nodes={n for _,owners in state[0]+state[1]+state[2] for n in owners} if code==1 else set()
            if '/map_server' in nodes and '/amcl' in nodes:
                if '/suat_mapping_view' in nodes:
                    print('[RViz] 已有 /suat_mapping_view，继续使用现有窗口',flush=True)
                    return
                env=os.environ.copy()
                env.update(DISPLAY=display, XAUTHORITY='/home/spark/.Xauthority')
                try:
                    proc=subprocess.Popen(['rviz','-d',str(config),'__name:=suat_mapping_view'],
                        env=env,stdin=subprocess.DEVNULL,start_new_session=True)
                except OSError as exc:
                    print('[RViz] 启动失败：%s；定位栈继续运行' % exc,file=sys.stderr,flush=True)
                    return
                if finished.wait(.5):
                    return
                if proc.poll() is not None:
                    print('[RViz] 窗口进程已退出，检查桌面显示及授权；定位栈继续运行',
                          file=sys.stderr,flush=True)
                else:
                    print('[RViz] 已启动桌面 %s 的地图、激光、相机与路径窗口' % display,flush=True)
                return
        except (OSError,xmlrpc.client.Error):
            pass
        finished.wait(.25)
    if not finished.is_set():
        print('[RViz] 等待地图和 AMCL 超时，未自动打开；定位栈继续运行',file=sys.stderr,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',action='store_true')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--at-start-a',action='store_true',help='operator placed robot at saved A start facing D, stopped with arm stowed')
    mode.add_argument('--manual-initialization',action='store_true',help='new map without a saved Start_A; initialize pose before navigation')
    parser.add_argument('--workspace-root',type=Path,default=ROOT)
    parser.add_argument('--defer-settle',action='store_true',help='let the competition launcher perform stationary alignment')
    parser.add_argument('--no-rviz',action='store_true',help='do not automatically open RViz when starting localization')
    parser.add_argument('--rviz-display',default=':0',help='robot desktop X display for RViz (default: :0)')
    parser.add_argument('--describe',action='store_true',help='offline selected-map plan; Start_A/ROS/process/device checks are not performed')
    args=parser.parse_args()
    if args.describe and args.start:parser.error('--describe cannot start nodes')
    root=args.workspace_root.resolve()
    binding=load_binding(root)
    alignment=binding.get('field_alignment')
    launch=['roslaunch',str(root/'src/suat_mission/launch/field_localization.launch'),
            'workspace_root:='+str(root),'map_file:='+binding['map_yaml']]
    if alignment:
        launch+=['field_transform:='+' '.join(static_transform_args(alignment)),
                 'field_alignment_version:='+alignment['alignment_version']]
    settle=args.at_start_a and not args.defer_settle
    launch+=['settle_initialization:='+str(settle).lower()]
    if args.describe:
        print(json.dumps({'map_file':binding['map_yaml'],'map_identity':map_identity(binding),
              'field_alignment_version':alignment['alignment_version'] if alignment else None,
              'launch_command':launch,'saved_Start_A_status':'NOT_CHECKED_OFFLINE',
              'initialization_mode':'manual_initialization' if args.manual_initialization else 'saved_Start_A',
              'requires_pose_initialization_before_navigation':args.manual_initialization,
              'required_at_start':(['initialize actual pose before navigation'] if args.manual_initialization
                  else ['verified Start_A pose/covariance','current --at-start-a confirmation']),
              'robot_connected':False,'hardware_ready':False},indent=2))
        return 0
    if args.manual_initialization:
        x,y,yaw=0.,0.,0.
        covariance=[.25,.25,.06853891945]
    else:
        data=read_start_pose(root/'data/waypoints/start_pose_A.yaml')
        x,y,yaw=validate_start_pose(data,binding)
        covariance=data['initial_covariance_diagonal']
    socket.setdefaulttimeout(1)
    master_up=False
    conflicts=[]
    try:
        response=xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311')).getSystemState('/suat_field_preflight')
        if response[0]!=1:raise RuntimeError(response[1])
        master_up=True
        pubs,subs,services=response[2]
        nodes={n for _,ns in pubs+subs+services for n in ns}
        conflicts=sorted(nodes & {'/spark_base_server','/ydlidar_g2_node','/robot_state_publisher',
            '/joint_state_publisher','/map_server','/amcl','/move_base','/suat_navigation_gate'})
        if dict(pubs).get('/cmd_vel'):conflicts+=dict(pubs)['/cmd_vel']
    except (OSError,xmlrpc.client.ProtocolError):pass
    # Serial-owning orphan nodes may survive a terminated ROS master.
    orphans=[]
    own_pid=os.getpid()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==own_pid:continue
        try:
            raw=(proc/'cmdline').read_bytes().decode(errors='replace')
            if any(p in raw for p in ('/lib/spark_base/spark_base_node','/lib/ydlidar_g2/ydlidar_g2_node',
                '/lib/robot_state_publisher/robot_state_publisher','/lib/joint_state_publisher/joint_state_publisher',
                '/lib/map_server/map_server','/lib/amcl/amcl')):
                orphans.append(int(proc.name))
        except OSError:pass
    paths={name:Path('/dev/'+name).exists() for name in ('sparkBase','ydlidar')}
    serial_targets={str(Path('/dev/'+name).resolve()) for name in paths}
    serial_owners=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            for fd in (proc/'fd').iterdir():
                try:
                    device=os.readlink(str(fd))
                    if device in serial_targets:
                        serial_owners.append({'pid':int(proc.name),'device':device})
                except OSError:pass
        except OSError:pass
    command=launch+['initial_pose_x:='+str(x),'initial_pose_y:='+str(y),'initial_pose_a:='+str(yaw),
             'initial_cov_xx:='+str(covariance[0]),
             'initial_cov_yy:='+str(covariance[1]),
             'initial_cov_aa:='+str(covariance[2])]
    check=subprocess.run(command[:1]+['--nodes']+command[1:],capture_output=True,text=True,timeout=10)
    expected={'/spark_base_server','/ydlidar_g2_node','/robot_state_publisher','/joint_state_publisher','/map_server','/amcl'}
    if settle:expected.add('/suat_localization_settler')
    if alignment:expected.add('/suat_field_alignment')
    names=set(check.stdout.split())
    if check.returncode or names!=expected:
        raise RuntimeError('minimal launch inventory differs: '+check.stdout+' '+check.stderr)
    ready=not conflicts and not orphans and not serial_owners and all(paths.values())
    result={'ready_to_start':ready,'master_already_running':master_up,'conflicting_nodes':conflicts,
            'existing_driver_processes':orphans,'devices_present':paths,'launch_nodes':sorted(names),
            'existing_serial_owners':serial_owners,
            'saved_A_pose':None if args.manual_initialization else [x,y,yaw],
            'initialization_mode':'manual_initialization' if args.manual_initialization else 'saved_Start_A',
            'requires_pose_initialization_before_navigation':args.manual_initialization,
            'physical_A_confirmation_supplied':args.at_start_a,
            'starts_motion_source':False,'starts_arm_or_camera_driver':False,'starts_move_base_or_task':False}
    print(json.dumps(result,indent=2),flush=True)
    if not ready:return 2
    if args.start:
        if not (args.at_start_a or args.manual_initialization):
            raise ValueError('--start requires current --at-start-a confirmation or --manual-initialization')
        finished=threading.Event()
        if not args.no_rviz:
            threading.Thread(target=open_rviz_when_ready,args=(root,finished,args.rviz_display),
                             daemon=True).start()
        try:
            return subprocess.call(command)
        finally:
            finished.set()
    return 0


if __name__=='__main__':raise SystemExit(main())
