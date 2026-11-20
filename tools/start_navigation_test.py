#!/usr/bin/env python3
"""Check the existing localization, then optionally start a disarmed planning/test stack."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', action='store_true')
    parser.add_argument('--rollout-candidate', action='store_true', help='load the unvalidated low-speed rollout candidate; output still starts disabled')
    args = parser.parse_args()
    socket.setdefaulttimeout(3)
    root = Path(__file__).resolve().parents[1]
    master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
    code, message, state = master.getSystemState('/suat_navigation_preflight')
    if code != 1:
        raise RuntimeError(message)
    pubs, subs, services = state
    nodes = {n for _, ns in pubs+subs+services for n in ns}
    conflicts = sorted(n for n in nodes if any(x in n for x in ('gmapping', 'cartographer', 'suat_mission'))
                       or n in ('/move_base', '/suat_navigation_gate', '/suat_navigation_nodelet_manager'))
    missing = []
    for name in ('move_base', 'dwa_local_planner', 'navfn', 'follower_velocity_smoother', 'suat_mission'):
        if subprocess.run(['rospack', 'find', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
            missing.append(name)
    ready = not conflicts and not missing and dict(pubs).get('/map', []) == ['/map_server'] and '/amcl' in nodes
    result = {'ready_to_start_disarmed': ready, 'conflicting_nodes': conflicts, 'missing_packages': missing,
              'existing_cmd_vel_publishers': dict(pubs).get('/cmd_vel', []),
              'output_initially': 'DISARMED_NO_CMD_VEL_PUBLISHER',
              'motion_goal_sent': False, 'starts_arm_or_mission': False}
    result['profile'] = 'rollout_candidate_not_field_validated' if args.rollout_candidate else 'dwa_baseline_first_trial_failed'
    print(json.dumps(result, indent=2), flush=True)
    if not ready:
        return 2
    if args.start:
        return subprocess.call(['roslaunch', str(root/'src/suat_mission/launch/navigation_test.launch'),
                                'workspace_root:='+str(root),
                                'rollout_candidate:='+('true' if args.rollout_candidate else 'false')])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
