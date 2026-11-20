#!/usr/bin/env python3
"""One Enter starts field bring-up, stationary alignment and the competition mission."""
import argparse
import atexit
from functools import partial
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

AMCL_COMPARISON = {
    'baseline': {'update_min_d': .25, 'update_min_a': .20},
    'fine': {'update_min_d': .05, 'update_min_a': .05},
}


def spawn_logged_component(root, logs, name, command, env=None):
    with (logs / (name + '.log')).open('w') as output:
        return subprocess.Popen(command, cwd=str(root), stdin=subprocess.DEVNULL, stdout=output,
                                stderr=subprocess.STDOUT, start_new_session=True, env=env)


def ready(check, process, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            return False
        if check():
            return True
        time.sleep(.2)
    return False


def ensure_ros_master(spawn, allow_start=True):
    import rosgraph

    if rosgraph.is_master_online():
        return
    if not allow_start:
        raise RuntimeError('calibration preview requires a running ROS master')
    print('Starting ROS master.', flush=True)
    master = spawn('roscore', ['roscore'])
    if not ready(rosgraph.is_master_online, master, 15.):
        raise RuntimeError('ROS master startup failed; see roscore.log in the run directory')


def missing_camera_topics(published, color_only=False):
    return [topic for topic, kind in CAMERA_TOPICS.items() if published.get(topic) != kind
            and (not color_only or topic == '/camera/color/image_raw')]


def start_rgbd_camera(spawn, nodes, topics, color_only=False):
    if (not missing_camera_topics(topics, color_only)
            or any(name.startswith('/camera/') for name in nodes)
            or any(topic.startswith('/camera/') for topic in topics)):
        return None
    print('Starting the D435 RGB-D camera.', flush=True)
    return spawn('camera', ['roslaunch', 'realsense2_camera', 'rs_camera.launch',
        'enable_color:=true', 'enable_depth:=true', 'align_depth:=true', 'enable_infra:=false',
        'enable_infra1:=false', 'enable_infra2:=false', 'enable_pointcloud:=false', 'publish_tf:=false',
        'publish_odom_tf:=false', 'color_width:=640', 'color_height:=480', 'color_fps:=30',
        'depth_width:=640', 'depth_height:=480', 'depth_fps:=30'])


def open_observation_windows(root, spawn, nodes):
    gui = os.environ.copy()
    if not gui.get('DISPLAY') and Path('/tmp/.X11-unix/X0').is_socket():
        gui['DISPLAY'] = ':0'
        authority = Path.home() / '.Xauthority'
        if authority.is_file():
            gui.setdefault('XAUTHORITY', str(authority))
    if not gui.get('DISPLAY'):
        print('No desktop display found; continuing with the RGB-D driver and run logs.', flush=True)
        return
    views = (
        ('camera_view', [sys.executable, 'tools/mission_camera_view.py'],
         {'/suat_mission_camera_view'}),
        ('rviz', ['rviz', '-d', str(root / 'src/suat_mission/config/mission_monitor.rviz'),
                  '__name:=suat_mission_rviz'],
         {'/rviz', '/suat_mapping_view', '/suat_localization_view', '/suat_four_sides_rviz', '/suat_mission_rviz'}),
    )
    for name, command, existing in views:
        if set(nodes).isdisjoint(existing):
            try:
                spawn(name, command, env=gui)
            except OSError as exc:
                print('Optional ' + name + ' window failed to launch: ' + str(exc), file=sys.stderr, flush=True)


def set_amcl_comparison(profile, report_path):
    from dynamic_reconfigure.client import Client
    client = Client('/amcl', timeout=3.)
    requested = AMCL_COMPARISON[profile]
    config = client.get_configuration(timeout=3.)
    before = {name:config[name] for name in requested}
    report = {'profile':profile, 'before':before, 'requested':requested}

    def save():
        report_path.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')

    if before != requested:
        def restore():
            try:
                restored = client.update_configuration(before)
                report['restored'] = {name:restored[name] for name in before}
                if report['restored'] != before:
                    raise RuntimeError('AMCL readback differs from the original thresholds')
            except Exception as exc:
                report['restore_error'] = str(exc)
                print('AMCL comparison cleanup: '+str(exc),file=sys.stderr,flush=True)
            save()

        atexit.register(restore)
        config = client.update_configuration(requested)
    report['applied'] = {name:config[name] for name in requested}
    save()
    if report['applied'] != requested:
        raise RuntimeError('AMCL comparison thresholds were not applied')


def missing_base_command(nodes, topics):
    """Restore only the missing base on an otherwise retained localization stack."""
    if (not {'/amcl','/map_server'} <= set(nodes) or '/scan' not in topics
            or '/odom' in topics or '/spark_base_server' in nodes):
        return None
    if '/cmd_vel' in topics:
        raise RuntimeError('cannot start the missing base driver while /cmd_vel has a publisher')
    device='/dev/sparkBase'
    if not Path(device).exists():
        raise RuntimeError('base driver missing and '+device+' is absent')
    owner=subprocess.run(['fuser',device],capture_output=True)
    if owner.returncode!=1 or owner.stdout.strip() or owner.stderr.strip():
        raise RuntimeError('base driver missing; exclusive access to '+device+' is unavailable')
    description={'/robot_state_publisher','/joint_state_publisher'}.intersection(nodes)
    if not description:
        return ['roslaunch','spark_bringup','driver_bringup.launch','start_base:=true',
                'start_camera:=false','enable_arm_tel:=false','arm_type_tel:=uarm',
                'camera_type_tel:=d435','lidar_type_tel:=ydlidar_g2']
    return ['roslaunch','spark_base','spark_base.launch','serialport:='+device]


def navigate_to_board(root, configuration, keepout_file, logs, planner, station_id=None, *, entered_at):
    """Run the normal navigation leg without starting the mission node."""
    from suat_mission.contracts import ActionContext
    from suat_mission.navigation.board_stations import board_station_candidates
    from suat_mission.navigation.navigation_budget import NavigationBudget
    from suat_mission.navigation.navigation_ros import NavigationPlanError, Navigator
    from suat_mission.navigation.stations import mission_stations

    nav = None
    started = time.monotonic()
    outcome = {'passed': False, 'run_id': logs.name, 'scope': 'to-board',
               'speed': 'slow', 'planner': planner, 'requested_station': station_id,
               'station_attempts': []}
    try:
        nav = Navigator(root, 'slow', owner='/suat_mission', planner_mode=planner,
                        keepout_file=keepout_file, comparison=True)
        nav.connect_control()
        station = mission_stations(root,configuration,nav.binding)['Classification_area']['pose']
        if configuration.get('board_station_records_file'):
            records = json.loads((root / configuration['board_station_records_file']).read_text(encoding='utf-8'))
            candidates = board_station_candidates(station, nav.binding, records)
        else:
            candidates = ({'id': 'active_route', 'pose': station},)
        if station_id is not None:
            candidates = tuple(item for item in candidates if item['id'] == station_id)
            if not candidates:
                raise ValueError('board station not present in this map: ' + station_id)
        budget = NavigationBudget(float(configuration.get('navigation_timeout', 60.)))
        competition_deadline = (entered_at + float(configuration.get('mission_timeout', 600.))
                                - float(configuration.get('startup_delay', 0.))
                                - float(configuration.get('stop_margin', 0.)))
        deadline = budget.deadline(started, competition_deadline)
        outcome.update(budget_s=vars(budget), competition_deadline=competition_deadline,
                       navigation_deadline=deadline)
        for candidate in candidates:
            if time.monotonic() + nav.executor_stop_budget() >= deadline:
                raise TimeoutError('navigation budget exhausted before the next board station')
            planned_at = time.monotonic()
            name = 'Classification_area__' + candidate['id']
            pose = list(candidate['pose'])
            attempt = dict(candidate)
            outcome['station_attempts'].append(attempt)
            start = nav.pose()
            outcome['actual_pose'] = start
            try:
                plan = nav.plan(start, pose, nav.grid())
            except NavigationPlanError as exc:
                attempt.update(error=str(exc), rejected_plan=exc.plan, first_failure_stage='planning',
                               elapsed_s=time.monotonic()-planned_at, actual_pose=start)
                outcome.setdefault('first_failure_stage','planning')
                if str(exc) not in NavigationPlanError.STATION_ERRORS:
                    raise
                continue
            outcome.update(station=name, start_pose=start, goal_pose=pose)
            context = ActionContext(logs.name, name, 'navigate', '', 0, deadline)
            print('Navigating to ' + name, flush=True)
            result = nav.execute_leg({'name': name, 'pose': pose}, plan,
                                     context=context, station_approach=True, budget=budget)
            outcome.update(passed=result['passed'], navigation=result)
            outcome['actual_pose'] = result.get('actual_pose', start)
            if not result['passed']:
                outcome.setdefault('first_failure_stage',result.get('first_failure_stage','navigation'))
            attempt['navigation'] = result
            if not outcome['passed'] and nav.can_try_next_station(result, context):
                nav.connect_control()
                continue
            if not outcome['passed']:
                outcome['error'] = result.get('error', 'arrival and stop were not confirmed')
            break
        else:
            outcome['error'] = (outcome.get('navigation', {}).get('error')
                                or 'no classification station had a clear route from Start_A')
    except (Exception, KeyboardInterrupt) as exc:
        outcome['error'] = str(exc) or type(exc).__name__
        outcome.setdefault('first_failure_stage','navigation_prepare')
    finally:
        outcome['elapsed_s'] = time.monotonic()-started
        outcome.setdefault('actual_pose',None)
        phases = []
        for attempt in outcome['station_attempts']:
            leg = attempt.get('navigation')
            if not leg:
                continue
            if leg.get('initial_contact_prefix'):
                phases.append(leg['initial_contact_prefix'])
            if leg.get('position_leg') and leg['phase'] != leg['position_leg']['phase']:
                phases.append(leg['position_leg'])
            phases.append(leg)
        outcome['comparison_metrics'] = {
            'elapsed_s': outcome['elapsed_s'],
            'planned_distance_m': sum((phase.get('plan') or {}).get('path_length_m',0.) for phase in phases),
            'odom_distance_m': sum(phase.get('travel_m',phase.get('distance_m',0.)) or 0. for phase in phases),
            'timeouts': sum(bool(phase.get('timed_out','deadline' in (phase.get('error') or ''))) for phase in phases),
            'odom_turn_reversals': sum(phase.get('odom_turn_reversals',0) for phase in phases),
            'max_observed_cmd_vel_gap_s': max(
                (phase['output_max_gap_s'] for phase in phases if phase.get('output_max_gap_s') is not None),default=None),
            'output_messages': sum(phase.get('output_messages',0) for phase in phases),
        }
        try:
            if nav is not None:
                nav.close()
        finally:
            (logs/'result.json').write_text(json.dumps(outcome,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    message = 'ARRIVED AND STOPPED' if outcome['passed'] else 'NAVIGATION DID NOT PASS: '+outcome['error']
    print(message + ' | ' + str(logs/'result.json'), flush=True)
    return 0 if outcome['passed'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scope',choices=('cake-only','full'),nargs='?',default='cake-only')
    parser.add_argument('--speed',choices=('slow','normal','fast'),default='fast')
    parser.add_argument('--planner',choices=('rollout','dwa'),default='rollout')
    parser.add_argument('--at-start-a',action='store_true',
                        help='automatically align at the saved A placement after Enter, before motion')
    parser.add_argument('--to-board',action='store_true',
                        help='one Enter for Start_A to the board station at low speed')
    parser.add_argument('--board-station', help='fix the recorded station ID for a --to-board comparison')
    parser.add_argument('--amcl-profile', choices=tuple(AMCL_COMPARISON),
                        help='set and record the AMCL thresholds for this A-start run')
    parser.add_argument('--mission-config',type=Path,
                        help='select an alternate mission configuration, including its geometry')
    parser.add_argument('--geometry',type=Path,help='explicit existing measured/baseline geometry resource')
    parser.add_argument('--calibration',type=Path,help='optional factory coefficient profile, read once for this run')
    parser.add_argument('--grasp-source',choices=('dlt','official-planar'),default='dlt',
                        help='DLT remains default; official-planar requires a matching height model')
    parser.add_argument('--calibration-preview',action='store_true',
                        help='launch observations and return without preparing or starting arm/task execution')
    args=parser.parse_args()
    if args.to_board and (not args.at_start_a or args.speed!='slow' or args.calibration_preview):
        parser.error('--to-board requires --at-start-a --speed slow')
    if args.board_station and not args.to_board:
        parser.error('--board-station requires --to-board')
    if args.amcl_profile and not args.at_start_a:
        parser.error('--amcl-profile requires --at-start-a')
    root=Path(__file__).resolve().parents[1]
    import yaml
    mission_config=(root/(args.mission_config or Path('src/suat_mission/config/mission.yaml')).expanduser()).resolve()
    if not mission_config.is_file():
        parser.error('mission configuration missing: '+str(mission_config))
    configuration=yaml.safe_load(mission_config.read_text(encoding='utf-8'))
    configured_geometry=configuration.get('pickup_geometry_file')
    geometry=args.geometry or Path(configured_geometry or
        'data/calibration/field_20261117/pickup_geometry_visual_20261119.yaml')
    geometry=geometry.expanduser()
    if not geometry.is_absolute():geometry=root/geometry
    geometry=geometry.resolve()
    if not args.to_board and not geometry.exists():
        raise RuntimeError('geometry resource missing: '+str(geometry)+'; select the actual file with --geometry')
    sys.path.insert(0,str(root/'src/suat_mission/src'))
    from suat_mission.automation.mission_console import discard_pending_input, main as run_console
    from suat_mission.navigation.navigation_profiles import (
        CONTROLLER_HZ, configured_global_planner, LOCAL_PLANNER_PLUGIN, PLANNER_NAMESPACE,
        local_planner_settings, planner_profile_matches,
    )
    from suat_mission.official_calibration import bind_official_geometry
    from suat_mission.navigation.pickup_keepout import KEEP_OUT_FILE
    keepout_file=configuration.get('navigation_keepout_file',KEEP_OUT_FILE)
    profile=args.calibration.expanduser() if args.calibration is not None else None
    if profile is not None and not profile.is_absolute():profile=root/profile
    original_geometry=geometry
    values=None if args.to_board else bind_official_geometry(
        yaml.safe_load(geometry.read_text(encoding='utf-8')),profile,args.grasp_source)
    entered_at=None
    if not args.calibration_preview:
        if not sys.stdin.isatty():
            parser.error('one fresh Enter must come from the operator terminal')
        discard_pending_input()
        prompt=('Press Enter once to align at A and navigate to the board station: ' if args.to_board
                else 'Press Enter once to start the field stack and mission: ')
        if input(prompt)!='':
            parser.error('press Enter on an empty line')
        entered_at=time.monotonic()
    prefix='a_to_board_acceptance_' if args.to_board else 'competition_start_'
    logs=root/'reports/runs'/(prefix+time.strftime('%Y%m%d_%H%M%S'))
    logs.mkdir(parents=True,exist_ok=False)
    alignment_report=logs/'start_localization.json' if args.at_start_a else None
    geometry=logs/'resolved_geometry.json'
    if values is not None:
        geometry.write_text(json.dumps(values,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (logs/'resource_selection.json').write_text(json.dumps({
        'scope':'to-board' if args.to_board else args.scope,
        'base_geometry_file':str(original_geometry) if values is not None else None,
        'resolved_geometry_file':str(geometry) if values is not None else None,
        'mission_config_file':str(mission_config),
        'navigation_keepout_file':keepout_file,
        'official_profile':str(profile) if profile else None,'grasp_xy_source':args.grasp_source,
        'official_files':(values or {}).get('official_planar',{}).get('planes',[]),
        'operator_enter_monotonic':entered_at,
        'navigation_cruise_profile':args.speed,'navigation_planner_mode':args.planner,
        'requested_board_station':args.board_station,
        'navigation_cruise_sampling':local_planner_settings(args.planner,comparison=args.to_board),
        'navigation_approach_sampling':local_planner_settings(args.planner,approach=True,comparison=args.to_board),
        'start_localization_report':str(alignment_report) if alignment_report else None,
        'physical_accuracy':'NOT_TESTED_BY_LOADING'},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Run logs: '+str(logs),flush=True)
    spawn = partial(spawn_logged_component, root, logs)

    import rospy
    import rosnode
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    ensure_ros_master(spawn, allow_start=not args.calibration_preview)
    if args.to_board and '/suat_mission' in rosnode.get_node_names():
        raise RuntimeError('an existing mission node owns the official navigation name')
    rospy.init_node('suat_mission' if args.to_board else 'suat_competition_launcher',
                    anonymous=not args.to_board,disable_signals=True)
    def message(topic,timeout=1.):
        try:return json.loads(rospy.wait_for_message(topic,String,timeout=timeout).data)
        except rospy.ROSException:return None
    state=message('/suat/mission/state',.5)
    mission_exists='/suat_mission' in rosnode.get_node_names()
    if not args.to_board and mission_exists and state is None:
        raise RuntimeError('existing mission state is unavailable; refusing to replace it')
    if state and any(state.get(key) for key in ('worker_id','preparing','prepared','recovery_required')):
        raise RuntimeError('existing mission is prepared, active or requires recovery; no second worker started')
    nav=message('/suat/navigation/state',1.)
    if nav and (nav.get('state')=='ARMED' or nav.get('active_goal')):
        raise RuntimeError('another navigation leg is active')
    if not args.to_board and mission_exists:
        rosnode.kill_nodes(['/suat_mission'])
        time.sleep(.5)
    if not args.calibration_preview:
        def missing_camera():
            return missing_camera_topics(dict(rospy.get_published_topics()), color_only=args.to_board)

        nodes=rosnode.get_node_names()
        topics=dict(rospy.get_published_topics())
        localization=None
        base_command=missing_base_command(nodes,topics)
        if base_command:
            print('Restoring the missing Spark base driver for the running localization stack.',flush=True)
            localization=spawn('base_recovery',base_command)
        if '/map' not in topics and not {'/map_server','/amcl'}.intersection(nodes):
            if not args.at_start_a:
                raise RuntimeError('automatic localization startup requires the robot at the saved A pose and --at-start-a')
            have_scan,have_odom='/scan' in topics,'/odom' in topics
            if have_scan!=have_odom:
                raise RuntimeError('field drivers are partially running; see missing /scan or /odom')
            tool='start_localization.py' if have_scan else 'start_field_localization.py'
            print('Starting field localization.',flush=True)
            localization=spawn('localization',[sys.executable,'tools/'+tool,'--start','--at-start-a','--defer-settle'])

        camera=start_rgbd_camera(spawn,nodes,topics,color_only=args.to_board)
        field_topics={'/map','/scan','/odom'}
        if not ready(lambda: field_topics <= dict(rospy.get_published_topics()).keys()
                     and '/amcl' in rosnode.get_node_names(),localization,30.):
            missing=sorted(field_topics-dict(rospy.get_published_topics()).keys())
            raise RuntimeError('field localization unavailable; missing topics=%s, AMCL=%s; logs: %s'
                               % (missing,'/amcl' in rosnode.get_node_names(),logs))
        if not ready(lambda: not missing_camera(),camera,20.):
            raise RuntimeError('RGB-D camera is missing '+', '.join(missing_camera())+'; logs: '+str(logs))

        open_observation_windows(root,spawn,rosnode.get_node_names())
    if alignment_report:
        if any(topic=='/cmd_vel' for topic,_ in rospy.get_published_topics()):
            raise RuntimeError('cannot restore Start_A while a base command source is running')
        if args.amcl_profile:
            set_amcl_comparison(args.amcl_profile,logs/'amcl_profile.json')
        from suat_mission.navigation.localization_settler import restore_start_a
        print('Automatically aligning Start_A while stopped.',flush=True)
        alignment=restore_start_a(root,alignment_report)
        print('Start_A localization '+alignment['status']+': '+str(alignment_report),flush=True)
    expected_global=configured_global_planner(root,'rollout' if args.to_board else args.planner)
    config_dir = root/'src/suat_mission/config'
    expected_grid_path = None
    if expected_global == 'global_planner/GlobalPlanner':
        planner_config = yaml.safe_load((config_dir/'navigation_test.yaml').read_text(encoding='utf-8'))
        expected_grid_path = planner_config['GlobalPlanner']['use_grid_path']
    goal_handling = {}
    if args.to_board or args.planner == 'rollout':
        rollout_config = yaml.safe_load((config_dir/'navigation_rollout_candidate.yaml').read_text(encoding='utf-8'))
        goal_handling = {
            '/move_base/planner_frequency': rollout_config['planner_frequency'],
            PLANNER_NAMESPACE+'/latch_xy_goal_tolerance': rollout_config['DWAPlannerROS']['latch_xy_goal_tolerance'],
        }

    def desired_navigation(state):
        return (state and state.get('owner')=='/suat_mission' and state.get('speed_profile')==args.speed
                and state.get('mission_keepout') is True
                and state.get('keepout_file',KEEP_OUT_FILE)==keepout_file
                and {'lease_linear_cap_mps','lease_angular_cap_radps'}<=state.keys()
                and rospy.get_param('/move_base/base_global_planner',None)==expected_global
                and (expected_grid_path is None or
                     rospy.get_param('/move_base/GlobalPlanner/use_grid_path',None)==expected_grid_path)
                and all(rospy.get_param(name, None)==value for name,value in goal_handling.items())
                and rospy.get_param('/move_base/base_local_planner',None)==LOCAL_PLANNER_PLUGIN
                and rospy.get_param('/move_base/controller_frequency',None)==CONTROLLER_HZ
                and planner_profile_matches(rospy.get_param(PLANNER_NAMESPACE,{}),args.planner,
                                            comparison=args.to_board))

    nav=message('/suat/navigation/state',1.)
    if nav and (nav.get('state')=='ARMED' or nav.get('active_goal')):
        raise RuntimeError('another navigation leg is active')
    if nav and not desired_navigation(nav):
        rospy.ServiceProxy('/suat_navigation_gate/stop',Trigger)()
        rosnode.kill_nodes([n for n in rosnode.get_node_names() if n in
            ('/move_base','/suat_navigation_gate','/suat_navigation_nodelet_manager','/suat_navigation_smoother')])
        rospy.sleep(1.);nav=None
    proc=None
    if nav is None:
        command=[sys.executable,'tools/start_field_navigation.py','--start','--speed',args.speed,
                 '--planner',args.planner,'--owner','mission','--keepout',keepout_file]
        if args.to_board:
            command.append('--comparison')
        proc=spawn('navigation',command)
    deadline=time.monotonic()+20.
    while time.monotonic()<deadline:
        if proc is not None and proc.poll() is not None:
            raise RuntimeError('navigation startup failed: '+str(logs/'navigation.log'))
        nav=message('/suat/navigation/state',1.)
        if nav and (nav.get('state')=='ARMED' or nav.get('active_goal')):
            raise RuntimeError('another navigation leg is active')
        if desired_navigation(nav) and nav.get('health') is None:break
    else:
        actual=rospy.get_param('/move_base/base_global_planner',None)
        raise RuntimeError('navigation did not become ready; global planner expected=%s actual=%s; '
                           'gate=%s; see %s' % (expected_global,actual,json.dumps(nav),logs))
    if args.to_board:
        return navigate_to_board(root,configuration,keepout_file,logs,args.planner,args.board_station,
                                 entered_at=entered_at)
    old_scene='/suat_mission_scene'
    if old_scene in rosnode.get_node_names():rosnode.kill_nodes([old_scene]);rospy.sleep(.3)
    mission_command=['roslaunch','suat_mission','mission.launch','workspace_root:='+str(root),
                     'navigation_speed:='+args.speed,
                     'navigation_planner_mode:='+args.planner,'geometry_file:='+str(geometry),
                     'config:='+str(mission_config),
                     'observation_target:='+('cake' if args.scope=='cake-only' else 'clock')]
    process=spawn('mission',mission_command)
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
