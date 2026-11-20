#!/usr/bin/env python3
"""One visible stop-only button on the robot desktop; never enables motion."""
import argparse
import os
from pathlib import Path
import subprocess
import rospy
from std_srvs.srv import Trigger

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout',type=int,default=900,help='stop automatically after 10..1800 seconds')
    args=parser.parse_args()
    if not 10<=args.timeout<=1800:parser.error('timeout must be 10..1800 seconds')
    env=dict(os.environ)
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if p.stat().st_uid!=os.getuid() or b'gnome-shell' not in (p/'cmdline').read_bytes():continue
            values=dict(x.split('=',1) for x in (p/'environ').read_bytes().decode(errors='replace').split('\0') if '=' in x)
            for key in ('DISPLAY','XAUTHORITY','DBUS_SESSION_BUS_ADDRESS','XDG_RUNTIME_DIR'):
                if key in values:env[key]=values[key]
            break
        except OSError:pass
    if not env.get('DISPLAY'):raise RuntimeError('robot desktop not available')
    rospy.init_node('suat_navigation_stop_panel',anonymous=True)
    try:
        subprocess.run(['zenity','--info','--title=导航测试停止','--text=短距离导航测试\n点击下面按钮或关闭本窗口，立即锁存停止。',
                        '--ok-label=立即停止','--width=360','--height=160'],env=env,timeout=args.timeout)
    except subprocess.TimeoutExpired:pass
    rospy.wait_for_service('/suat_navigation_gate/stop',timeout=2)
    result=rospy.ServiceProxy('/suat_navigation_gate/stop',Trigger)()
    print(result.message,flush=True)

if __name__=='__main__':main()
