#!/usr/bin/env python3
"""One Enter starts field bring-up, stationary alignment and the competition mission."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


CAMERA_TOPICS = {
    '/camera/color/image_raw': 'sensor_msgs/Image',
    '/camera/aligned_depth_to_color/image_raw': 'sensor_msgs/Image',
    '/camera/color/camera_info': 'sensor_msgs/CameraInfo',
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scope',choices=('cake-only','full'),nargs='?',default='cake-only')
    parser.add_argument('--speed',choices=('slow','normal','fast'),default='fast')
    parser.add_argument('--planner',choices=('rollout','dwa'),default='rollout')
    parser.add_argument('--at-start-a',action='store_true',
                        help='automatically align at the saved A placement after Enter, before motion')
    parser.add_argument('--geometry',type=Path,help='explicit existing measured/baseline geometry resource')
    parser.add_argument('--calibration',type=Path,help='optional factory coefficient profile, read once for this run')
    parser.add_argument('--grasp-source',choices=('dlt','official-planar'),default='dlt',
                        help='DLT remains default; official-planar requires a matching height model')
    parser.add_argument('--calibration-preview',action='store_true',
                        help='launch observations and return without preparing or starting arm/task execution')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    geometry=args.geometry or root/'data/calibration/field_20261117/pickup_geometry_visual_20261119.yaml'
    geometry=geometry.expanduser()
    if not geometry.is_absolute():geometry=root/geometry
    geometry=geometry.resolve()
    if not geometry.exists():
        raise RuntimeError('geometry resource missing: '+str(geometry)+'; select the actual file with --geometry')
    sys.path.insert(0,str(root/'src/suat_mission/src'))
    import yaml
    from suat_mission.automation.mission_console import discard_pending_input, main as run_console
    from suat_mission.official_calibration import bind_official_geometry
    profile=args.calibration.expanduser() if args.calibration is not None else None
    if profile is not None and not profile.is_absolute():profile=root/profile
    original_geometry=geometry
    values=bind_official_geometry(yaml.safe_load(geometry.read_text(encoding='utf-8')),profile,args.grasp_source)
    entered_at=None
    if not args.calibration_preview:
        if not sys.stdin.isatty():
            parser.error('one fresh Enter must come from the operator terminal')
        discard_pending_input()
        if input('Press Enter once to start the field stack and mission: ')!='':
            parser.error('press Enter on an empty line')
        entered_at=time.monotonic()
    logs=root/'reports/runs'/('competition_start_'+time.strftime('%Y%m%d_%H%M%S'))
    logs.mkdir(parents=True,exist_ok=False)
    alignment_report=logs/'start_localization.json' if args.at_start_a else None
    geometry=logs/'resolved_geometry.json'
    geometry.write_text(json.dumps(values,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (logs/'resource_selection.json').write_text(json.dumps({
        'base_geometry_file':str(original_geometry),'resolved_geometry_file':str(geometry),
        'official_profile':str(profile) if profile else None,'grasp_xy_source':args.grasp_source,
        'official_files':values.get('official_planar',{}).get('planes',[]),
        'operator_enter_monotonic':entered_at,
        'navigation_cruise_profile':args.speed,'navigation_planner_mode':args.planner,
        'start_localization_report':str(alignment_report) if alignment_report else None,
        'physical_accuracy':'NOT_TESTED_BY_LOADING'},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Grasp XY source: '+args.grasp_source+'; resources: '+str(logs/'resource_selection.json'),flush=True)
    def spawn(name,command,env=None):
        with (logs/(name+'.log')).open('w') as output:
            return subprocess.Popen(command,cwd=str(root),stdin=subprocess.DEVNULL,stdout=output,
                                    stderr=subprocess.STDOUT,start_new_session=True,env=env)

    def ready(check, process, seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if process is not None and process.poll() is not None:
                return False
            if check():
                return True
            time.sleep(.2)
        return False

    import rospy
    import rosnode
    import rosgraph
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    if not rosgraph.is_master_online():
        if args.calibration_preview:
            raise RuntimeError('calibration preview requires a running ROS master')
        print('Starting ROS master.',flush=True)
        master=spawn('roscore',['roscore'])
        if not ready(rosgraph.is_master_online,master,15.):
            raise RuntimeError('ROS master startup failed; see '+str(logs/'roscore.log'))
    rospy.init_node('suat_competition_launcher',anonymous=True,disable_signals=True)
    def message(topic,timeout=1.):
        try:return json.loads(rospy.wait_for_message(topic,String,timeout=timeout).data)
        except rospy.ROSException:return None
    state=message('/suat/mission/state',.5)
    if state and (state.get('worker_id') or state.get('preparing') or state.get('recovery_required')):
        raise RuntimeError('existing mission is active or retaining a possible payload; no second worker started')
    nav=message('/suat/navigation/state',1.)
    if nav and (nav.get('state')=='ARMED' or nav.get('active_goal')):
        raise RuntimeError('another navigation leg is active')
    if '/suat_mission' in rosnode.get_node_names():
        rosnode.kill_nodes(['/suat_mission'])
        time.sleep(.5)
    if not args.calibration_preview:
        def missing_camera():
            published=dict(rospy.get_published_topics())
            return [topic for topic,kind in CAMERA_TOPICS.items() if published.get(topic)!=kind]

        nodes=rosnode.get_node_names()
        topics=dict(rospy.get_published_topics())
        localization=None
        if '/map' not in topics and not {'/map_server','/amcl'}.intersection(nodes):
            if not args.at_start_a:
                raise RuntimeError('automatic localization startup requires the robot at the saved A pose and --at-start-a')
            have_scan,have_odom='/scan' in topics,'/odom' in topics
            if have_scan!=have_odom:
                raise RuntimeError('field drivers are partially running; see missing /scan or /odom')
            tool='start_localization.py' if have_scan else 'start_field_localization.py'
            print('Starting field localization.',flush=True)
            localization=spawn('localization',[sys.executable,'tools/'+tool,'--start','--at-start-a','--defer-settle'])

        camera=None
        if (missing_camera() and not any(name.startswith('/camera/') for name in nodes)
                and not any(topic.startswith('/camera/') for topic in topics)):
            print('Starting the D435 RGB-D camera.',flush=True)
            camera=spawn('camera',['roslaunch','realsense2_camera','rs_camera.launch',
                'enable_color:=true','enable_depth:=true','align_depth:=true','enable_infra:=false',
                'enable_infra1:=false','enable_infra2:=false','enable_pointcloud:=false','publish_tf:=false',
                'publish_odom_tf:=false','color_width:=640','color_height:=480','color_fps:=30',
                'depth_width:=640','depth_height:=480','depth_fps:=30'])
        field_topics={'/map','/scan','/odom'}
        if not ready(lambda: field_topics <= dict(rospy.get_published_topics()).keys()
                     and '/amcl' in rosnode.get_node_names(),localization,30.):
            raise RuntimeError('field localization requires /map, /scan, /odom and AMCL; logs: '+str(logs))
        if not ready(lambda: not missing_camera(),camera,20.):
            raise RuntimeError('RGB-D camera is missing '+', '.join(missing_camera())+'; logs: '+str(logs))

        gui=os.environ.copy()
        if not gui.get('DISPLAY') and Path('/tmp/.X11-unix/X0').is_socket():
            gui['DISPLAY']=':0'
            authority=Path.home()/'.Xauthority'
            if authority.is_file():gui.setdefault('XAUTHORITY',str(authority))
        if gui.get('DISPLAY'):
            nodes=set(rosnode.get_node_names())
            views=(('camera_view',['rosrun','image_view','image_view','image:=/camera/color/image_raw',
                                   '_autosize:=true','_window_name:=Spark-I Camera',
                                   '__name:=suat_board_camera_view'],
                    {'/suat_board_camera_view','/suat_four_sides_camera'}),
                   ('rviz',['rviz','-d',str(root/'src/suat_mission/config/mission_monitor.rviz'),
                            '__name:=suat_mission_rviz'],
                    {'/rviz','/suat_mapping_view','/suat_localization_view','/suat_four_sides_rviz','/suat_mission_rviz'}))
            for name,command,existing in views:
                if nodes.isdisjoint(existing):
                    try:
                        spawn(name,command,env=gui)
                    except OSError as exc:
                        print('Optional '+name+' window failed to launch: '+str(exc),file=sys.stderr,flush=True)
        else:
            print('No desktop display found; continuing with the RGB-D driver and run logs.',flush=True)
    if alignment_report:
        if any(topic=='/cmd_vel' for topic,_ in rospy.get_published_topics()):
            raise RuntimeError('cannot restore Start_A while a base command source is running')
        from suat_mission.navigation.localization_settler import restore_start_a
        print('Automatically aligning Start_A while stopped.',flush=True)
        alignment=restore_start_a(root,alignment_report)
        print('Start_A localization '+alignment['status']+': '+str(alignment_report),flush=True)
    def desired_navigation(state):
        return (state and state.get('owner')=='/suat_mission' and state.get('speed_profile')==args.speed
                and state.get('mission_keepout') is True
                and {'lease_linear_cap_mps','lease_angular_cap_radps'}<=state.keys()
                and rospy.get_param('/move_base/DWAPlannerROS/use_dwa',None) is (args.planner=='dwa'))

    if nav and not desired_navigation(nav):
        rospy.ServiceProxy('/suat_navigation_gate/stop',Trigger)()
        rosnode.kill_nodes([n for n in rosnode.get_node_names() if n in
            ('/move_base','/suat_navigation_gate','/suat_navigation_nodelet_manager','/suat_navigation_smoother')])
        rospy.sleep(1.);nav=None
    if nav is None:
        proc=spawn('navigation',[sys.executable,'tools/start_field_navigation.py','--start','--speed',args.speed,
                                 '--planner',args.planner,'--owner','mission'])
        deadline=time.monotonic()+20.
        while time.monotonic()<deadline:
            if proc.poll() is not None:raise RuntimeError('navigation startup failed: '+str(logs/'navigation.log'))
            nav=message('/suat/navigation/state',1.)
            if desired_navigation(nav) and nav.get('health') is None:break
        else:raise RuntimeError('navigation did not become ready; see '+str(logs))
    old_scene='/suat_mission_scene'
    if old_scene in rosnode.get_node_names():rosnode.kill_nodes([old_scene]);rospy.sleep(.3)
    process=spawn('mission',['roslaunch','suat_mission','mission.launch','navigation_speed:='+args.speed,
                            'geometry_file:='+str(geometry)])
    deadline=time.monotonic()+15.
    while time.monotonic()<deadline:
        if process.poll() is not None:raise RuntimeError('formal launch failed: '+str(logs/'mission.log'))
        launched=message('/suat/mission/state',.7)
        if launched and launched.get('core_delivery_id')=='20261117_official_bridge_C3':break
    else:raise RuntimeError('formal node did not start: '+str(logs/'mission.log'))
    print('Formal startup logs: '+str(logs),flush=True)
    if args.calibration_preview:
        print('Observation preview only: mission was not prepared or started; no arm task was opened.',flush=True)
        return 0
    result=run_console(['--scope',args.scope],entered_at=entered_at)
    print('No per-stage commands are needed. Result code: '+str(result),flush=True)
    if result:print('Mission process retained for real fault/recovery information: '+str(logs/'mission.log'),flush=True)
    return result


if __name__=='__main__':
    try:raise SystemExit(main())
    except KeyboardInterrupt:raise SystemExit(130)
    except (EOFError,OSError,RuntimeError,ValueError) as exc:print(str(exc),file=sys.stderr);raise SystemExit(2)
