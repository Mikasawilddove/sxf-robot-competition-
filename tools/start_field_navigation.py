#!/usr/bin/env python3
"""Describe or start the field navigation stack, always with final output disabled."""
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
from suat_mission.navigation.navigation_profiles import (
    CONTROLLER_HZ, configured_global_planner, LOCAL_PLANNER_PLUGIN, PLANNER_MODES,
    local_planner_settings, speed_profile,
)
from suat_mission.navigation.navigation_map import load_binding
from suat_mission.navigation.pickup_keepout import KEEP_OUT_FILE, PickupKeepout


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed',choices=('slow','normal','fast'),default='slow')
    parser.add_argument('--planner',choices=(*PLANNER_MODES,'path'),default='rollout')
    parser.add_argument('--comparison',action='store_true',
                        help='use the shared move_base settings for the board-station comparison')
    parser.add_argument('--owner',choices=('field','mission'),default='field',help='one navigation owner for this stack')
    parser.add_argument('--keepout',default=KEEP_OUT_FILE)
    parser.add_argument('--start',action='store_true')
    parser.add_argument('--describe',action='store_true',help='offline description, no ROS connection')
    args=parser.parse_args();speed=speed_profile(args.speed)
    owner='/suat_mission' if args.owner=='mission' else '/suat_field_navigation'
    settings=local_planner_settings(args.planner)
    path_control=args.planner=='path'
    if path_control and args.speed!='fast':
        parser.error('the fixed path controller requires --speed fast')
    use_rollout_settings=args.comparison or args.planner in ('rollout','path')
    global_planner=configured_global_planner(ROOT,'rollout' if use_rollout_settings else args.planner)
    result={'mode':'field','speed':args.speed,'planner':args.planner,'limits':speed,'field_validated':False,'navigation_owner':owner,
            'global_planner':global_planner,'local_planner':'field_path' if path_control else LOCAL_PLANNER_PLUGIN,
            'controller_hz':CONTROLLER_HZ,'sampling':settings,
            'comparison':args.comparison,
            'output_initially':'DISARMED_NO_CMD_VEL_PUBLISHER','motion_goal_sent':False}
    if args.describe:
        if args.start:parser.error('--describe cannot start nodes')
        print(json.dumps(result,indent=2));return 0
    socket.setdefaulttimeout(3)
    code,msg,state=xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311')).getSystemState('/suat_field_preflight')
    if code!=1:raise RuntimeError(msg)
    pubs,subs,services=state;nodes={n for _,ns in pubs+subs+services for n in ns}
    conflicts=sorted(n for n in nodes if any(x in n.lower() for x in ('gmapping','cartographer','teleop'))
                     or (n=='/suat_mission' and args.owner!='mission')
                     or n in ('/move_base','/suat_navigation_gate','/suat_navigation_nodelet_manager','/suat_field_navigation'))
    packages=['move_base','dwa_local_planner',global_planner.split('/',1)[0],'suat_mission']
    if not path_control:
        packages.append('follower_velocity_smoother')
    missing=[name for name in packages
             if subprocess.run(['rospack','find',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode]
    ready=not conflicts and not missing and not dict(pubs).get('/cmd_vel') and dict(pubs).get('/map')==['/map_server'] and '/amcl' in nodes
    result.update(ready_to_start=ready,conflicts=conflicts,missing_packages=missing)
    print(json.dumps(result,indent=2),flush=True)
    if not ready:return 2
    if args.start:
        if args.owner=='mission':
            PickupKeepout.load(ROOT,load_binding(ROOT),args.keepout)
        return subprocess.call(['roslaunch',str(ROOT/'src/suat_mission/launch/navigation_field.launch'),
            'workspace_root:='+str(ROOT),'speed_profile:='+args.speed,
            'navigation_owner:='+owner,
            'mission_keepout:='+str(args.owner=='mission').lower(),
            'keepout_file:='+args.keepout,
            'rollout_candidate:='+str(use_rollout_settings).lower(),
            'planner_mode:='+args.planner,
            'path_control:='+str(path_control).lower(),
            'local_planner_settings:='+json.dumps(settings,separators=(',',':')),
            'controller_hz:='+str(CONTROLLER_HZ),
            'max_v:='+str(speed['linear']),'max_w:='+str(speed['angular']),
            'accel_v:='+str(speed['accel_linear']),'accel_w:='+str(speed['accel_angular'])])
    return 0

if __name__=='__main__':raise SystemExit(main())
