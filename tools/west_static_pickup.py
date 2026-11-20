#!/usr/bin/env python3
"""Validate the formal cake grip at a manually positioned, stationary west station."""
import argparse
from datetime import datetime
from functools import partial
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time


# Position measured during the prior central empty-arm recovery.
UNFOLDED_POSE_MM = (191.74, -0.30, 124.09)
FOLD_FEED_LIMIT = 1000.


def check_stationary_sources(graph, observer_name=None):
    import rospy

    publishers, _, _ = graph.getSystemState()
    sources = dict(publishers)
    if sources.get('/cmd_vel') or sources.get('/suat/mission/state'):
        raise RuntimeError('base velocity source or formal mission still running')
    expected = {'/suat/perception/grasp_instances': observer_name,
                '/suat/manipulation/active': rospy.get_name()}
    for topic, owner in expected.items():
        if set(sources.get(topic, ())) - {owner}:
            raise RuntimeError('another producer is using ' + topic)


def start_stationary_stack(root, run):
    import rospy
    import rosgraph
    import rosnode
    from sensor_msgs.msg import CameraInfo, Image
    from run_competition import (ensure_ros_master, missing_base_command, missing_camera_topics,
                                 open_observation_windows, ready, spawn_logged_component, start_rgbd_camera)

    spawn = partial(spawn_logged_component, root, run)
    ensure_ros_master(spawn)
    rospy.init_node('suat_west_static_pickup', anonymous=True, disable_signals=True)
    graph = rosgraph.Master(rospy.get_name())
    check_stationary_sources(graph)
    nodes = rosnode.get_node_names()
    topics = dict(rospy.get_published_topics())
    camera = start_rgbd_camera(spawn, nodes, topics)
    open_observation_windows(root, spawn, nodes)

    base = None
    base_command = missing_base_command(nodes, topics)
    if base_command:
        base = spawn('base_recovery', base_command)
    elif '/odom' not in topics and not {'/map', '/scan'}.intersection(topics) and not {
            '/spark_base_server', '/ydlidar_g2_node', '/map_server', '/amcl',
            '/robot_state_publisher', '/joint_state_publisher'}.intersection(nodes):
        print('Starting field sensors for the stationary test.', flush=True)
        base = spawn('localization', [sys.executable, 'tools/start_field_localization.py',
                                      '--start', '--manual-initialization', '--no-rviz'])
    if not ready(lambda: '/odom' in dict(rospy.get_published_topics()), base, 30.):
        raise RuntimeError('no odometry from the field base; see localization or base_recovery log')

    def missing_camera():
        return missing_camera_topics(dict(rospy.get_published_topics()))

    if not ready(lambda: not missing_camera(), camera, 20.):
        raise RuntimeError('D435 startup did not provide ' + ', '.join(missing_camera()) + '; see camera.log')
    for topic, message_type in (
            ('/camera/color/image_raw', Image), ('/camera/aligned_depth_to_color/image_raw', Image),
            ('/camera/color/camera_info', CameraInfo)):
        try:
            rospy.wait_for_message(topic, message_type, timeout=4.)
        except rospy.ROSException as exc:
            raise RuntimeError('no D435 frames on ' + topic + '; see camera.log') from exc
    print('Odometry and RGB-D streams are ready.', flush=True)
    return graph


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission-config', type=Path,
                        default=Path('src/suat_mission/config/mission_full_pickup_39_candidate_20261119.yaml'))
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('use the operator terminal to start this stationary pickup')

    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root/'src/suat_mission/src'))
    from suat_mission.automation.mission_console import discard_pending_input

    discard_pending_input()
    if input('Press Enter once to open RViz and camera and run the stationary cake test: ') != '':
        parser.error('press Enter on an empty line')

    run = root/'reports/runs'/datetime.now().strftime('west_static_pickup_%Y%m%d_%H%M%S_%f')
    run.mkdir(parents=True, exist_ok=False)
    result = {'passed':False, 'scope':'west-static', 'phase':'starting_stack',
              'grasp_confirmed':False, 'returned_to_support':False}
    print('Run logs: ' + str(run), flush=True)
    try:
        run_pickup(root, args, run, result)
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or type(exc).__name__
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    (run/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return 0 if result['passed'] else 1


def run_pickup(root, args, run, result):
    graph = start_stationary_stack(root, run)

    import yaml
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String

    from suat_mission.automation.serial_mission import SerialMissionRuntime, resource
    from suat_mission.contracts import ActionContext, Outcome
    from suat_mission.observations import decode_scene
    from run_competition import spawn_logged_component

    config = yaml.safe_load(resource(root, args.mission_config).read_text(encoding='utf-8'))
    config['pickup_geometry_file'] = str(resource(root, config['pickup_geometry_file']))
    config['max_base_alignments'] = 0
    config['run_directory'] = str(run)
    result.update(geometry=config['pickup_geometry_file'], phase='stationary_observation')
    condition = threading.Condition()
    stopped = threading.Event()
    latest_odom = latest_scene = None
    graph_checked = 0.
    last_camera_error = ''
    observer_name = rospy.get_name() + '_scene'
    observer = None
    scene_counts = {'received':0, 'decoded':0}
    result['scene_messages'] = scene_counts

    def odometry(msg):
        nonlocal latest_odom
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        with condition:
            latest_odom = time.monotonic(), math.hypot(linear.x, linear.y), angular.z
            condition.notify_all()

    def scene_message(msg):
        nonlocal latest_scene, last_camera_error
        if (msg._connection_header or {}).get('callerid') != observer_name:
            return
        with condition:
            scene_counts['received'] += 1
        try:
            scene = decode_scene(msg.data, time.monotonic(), rospy.Time.now().to_sec(),
                                 float(config.get('observation_max_age', 2.)), runtime.geometry.calibration_id,
                                 int(config.get('target_min_frames', 3)))
        except (ValueError, KeyError, TypeError) as exc:
            with condition:
                last_camera_error = str(exc)
            return
        with condition:
            last_camera_error = ''
            scene_counts['decoded'] += 1
            if latest_scene is None or (scene.version > latest_scene.version
                                        and scene.source_stamp > latest_scene.source_stamp):
                latest_scene = scene
                condition.notify_all()

    def require_stopped():
        nonlocal graph_checked
        if stopped.is_set() or rospy.is_shutdown():
            raise RuntimeError('stop requested')
        now = time.monotonic()
        if now - graph_checked > .25:
            check_stationary_sources(graph, observer_name)
            graph_checked = now
        sample = latest_odom
        if (sample is None or now - sample[0] > .4 or not all(map(math.isfinite, sample))
                or abs(sample[1]) > .01 or abs(sample[2]) > .03):
            raise RuntimeError('fresh stationary odometry is required')

    def next_scene(version, deadline):
        while time.monotonic() < deadline:
            require_stopped()
            if observer is not None and observer.poll() is not None:
                raise RuntimeError('grasp observation exited; see ' + str(run/'grasp_scene.log'))
            with condition:
                if (latest_scene is not None and latest_scene.version > version
                        and time.monotonic()-latest_scene.observed_at <= float(config.get('observation_max_age', 2.))):
                    return latest_scene
                condition.wait(min(.1, max(0., deadline-time.monotonic())))
        raise TimeoutError('no fresh RGB-D scene'+(': '+last_camera_error if last_camera_error else ''))

    def reject_base_alignment(*_args):
        raise RuntimeError('the stationary pickup script has no base alignment executor')

    runtime = SerialMissionRuntime(root, config, next_scene, require_stopped, reject_base_alignment)
    try:
        check_stationary_sources(graph)
        odom_sub = rospy.Subscriber('/odom', Odometry, odometry, queue_size=1)
        stop_sub = rospy.Subscriber('/suat/mission/stop', Bool,
                                   lambda msg: stopped.set() if msg.data else None, queue_size=1)
        end = time.monotonic()+3.
        with condition:
            while latest_odom is None and time.monotonic()<end:
                condition.wait(.1)
        for _ in range(5):
            require_stopped()
            time.sleep(.1)
        scene_sub = rospy.Subscriber('/suat/perception/grasp_instances', String, scene_message, queue_size=1)
        observer = spawn_logged_component(root, run, 'grasp_scene', [
            'rosrun', 'suat_mission', 'mission_scene_node.py',
            '__name:=' + observer_name.lstrip('/'), '_workspace_root:=' + str(root),
            '_geometry_file:=' + config['pickup_geometry_file'],
            '_target_label:=cake', '_stationary_camera_tracking:=true',
        ])
        runtime.fresh_scene(time.monotonic()+12.)
        action_timeout = float(config.get('action_timeout', 60.))
        preparation_deadline = time.monotonic() + action_timeout
        result['phase'] = 'arm_workpose'
        print('Recovering the empty arm through the centered workpose.', flush=True)
        runtime.prepare_empty_observation(UNFOLDED_POSE_MM, preparation_deadline, fold_feed_limit=FOLD_FEED_LIMIT)
        runtime.bootstrap(preparation_deadline)
        result['phase'] = 'side_observation'
        runtime.observation('objects', preparation_deadline)
        deadline = time.monotonic() + action_timeout
        observe_until = min(deadline, time.monotonic()+float(config.get('observation_timeout', 5.)))
        result['phase'] = 'target_selection'
        selection = {'fresh_frames':0, 'last_cake_candidates':0}
        result['target_selection'] = selection
        while True:
            try:
                scene = runtime.fresh_scene(observe_until)
            except TimeoutError as exc:
                if selection['fresh_frames']:
                    raise RuntimeError('no graspable cake in %d fresh side-view scenes' %
                                       selection['fresh_frames']) from exc
                raise
            selection['fresh_frames'] += 1
            selection['last_cake_candidates'] = sum(item.label=='cake' for item in scene.detections)
            cakes = [item for item in scene.detections
                     if item.label=='cake' and item.graspable is True and not item.occluded_by]
            if len(cakes)>1:
                raise RuntimeError('more than one fresh graspable cake instance')
            if cakes:
                break
        target = cakes[0]
        result['phase'] = 'pick_and_return'
        context = ActionContext(run.name, 'west-static-cake', 'pick', target.object_id, scene.version, deadline)
        pick = runtime.pick(target, 'cake', context, return_to_support=True)
        result.update(passed=pick.outcome==Outcome.CONFIRMED, outcome=pick.outcome.value, evidence=pick.evidence)
    finally:
        result['grasp_confirmed'] = any(event['phase']=='holding_confirmed' for event in runtime.events)
        result['returned_to_support'] = any(event['phase']=='returned_to_support' for event in runtime.events)
        try:
            result['arm_close'] = runtime.close()
        except Exception as exc:
            result.update(passed=False, arm_close_error=str(exc))
        finally:
            if observer is not None and observer.poll() is None:
                observer.terminate()
                try:
                    observer.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    observer.kill()
                    observer.wait()


if __name__ == '__main__':
    sys.exit(main())
