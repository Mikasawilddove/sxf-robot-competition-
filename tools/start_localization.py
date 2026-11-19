#!/usr/bin/env python3
"""Check map/localization ownership, then optionally launch the canonical recording stack."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/suat_mission/src'))
from suat_mission.navigation_map import load_binding
from suat_mission.map_identity import map_identity
from suat_mission.start_pose import read_start_pose,validate_start_pose

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',action='store_true',help='launch map_server + AMCL; never starts motors or a task')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--at-start-a',action='store_true',help='operator has placed the robot at the recorded A start, facing D')
    mode.add_argument('--manual-initialization',action='store_true',help='new map or changed physical position; initialize and verify the actual pose')
    parser.add_argument('--workspace-root',type=Path,default=ROOT)
    parser.add_argument('--defer-settle',action='store_true',help='let the competition launcher perform stationary alignment')
    parser.add_argument('--describe',action='store_true',help='offline selected-map plan; no ROS/process/device checks')
    args=parser.parse_args()
    if args.describe and args.start:parser.error('--describe cannot start nodes')
    if args.start and not (args.at_start_a or args.manual_initialization):
        parser.error('--start requires --at-start-a or explicit --manual-initialization')
    root=args.workspace_root.resolve()
    binding=load_binding(root)
    map_path=Path(binding['map_yaml'])
    launch_args=['workspace_root:='+str(root),'map_file:='+str(map_path)]
    launch_args+=['settle_initialization:='+str(args.at_start_a and not args.defer_settle).lower()]
    if args.describe:
        print(json.dumps({'map_file':str(map_path),'map_identity':map_identity(binding),
              'launch_command':['roslaunch',str(root/'src/suat_mission/launch/localization_record.launch')]+launch_args,
              'saved_Start_A_status':'NOT_CHECKED_OFFLINE','robot_connected':False,'hardware_ready':False},indent=2))
        return 0
    socket.setdefaulttimeout(3)
    master=xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'))
    code,message,state=master.getSystemState('/suat_localization_preflight')
    if code!=1:raise RuntimeError(message)
    pubs,subs,services=state
    nodes={n for _,ns in pubs+subs+services for n in ns}
    map_publishers=dict(pubs).get('/map',[])
    conflicts=sorted(n for n in nodes if 'gmapping' in n or 'cartographer' in n or n in ('/amcl','/map_server','/move_base'))
    start_info={'mode':'manual_initialization'}
    start_error=None
    if args.at_start_a:
        try:
            saved_path=root/'data/waypoints/start_pose_A.yaml'
            data=read_start_pose(saved_path)
            x,y,yaw=validate_start_pose(data,binding)
            launch_args+=['initial_pose_x:='+str(x),'initial_pose_y:='+str(y),'initial_pose_a:='+str(yaw)]
            launch_args+=['initial_cov_xx:='+str(data['initial_covariance_diagonal'][0]),
                          'initial_cov_yy:='+str(data['initial_covariance_diagonal'][1]),
                          'initial_cov_aa:='+str(data['initial_covariance_diagonal'][2])]
            start_info={'mode':'saved_Start_A','file':str(saved_path),'pose':[x,y,yaw],
                        'operator_reports_currently_at_A_facing_D':True}
        except (OSError,ValueError,KeyError) as exc:
            start_error=str(exc)
    result={'map_file':str(map_path),'map_publishers':map_publishers,'conflicting_nodes':conflicts,
            'ready_to_start':not conflicts and not map_publishers and map_path.is_file() and not start_error,
            'initial_pose':start_info,'start_pose_error':start_error,
            'motion_commands':False,'task_started':False}
    print(json.dumps(result,indent=2),flush=True)
    if not result['ready_to_start']:return 2
    if args.start:
        return subprocess.call(['roslaunch',str(root/'src/suat_mission/launch/localization_record.launch')]+launch_args)
    return 0

if __name__=='__main__':raise SystemExit(main())
