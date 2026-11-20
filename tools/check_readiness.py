#!/usr/bin/env python3
"""Read files only; never imports rospy or publishes robot commands."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/suat_mission/src'))
from suat_mission.readiness import inspect_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/suat_mission/config/mission.yaml')
    parser.add_argument('--recorded-waypoints', default='data/waypoints/suat_waypoints.yaml')
    parser.add_argument('--workspace-root', type=Path, default=ROOT,
                        help='workspace holding the selected map and field resources')
    args = parser.parse_args()
    import yaml  # available on the Noetic robot; do not install anything automatically
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if config.get('execution_backend') == 'serial_rgbd':
        from suat_mission.automation.serial_mission import resolve_serial_config
        from suat_mission.navigation.navigation_map import load_binding
        from suat_mission.navigation.stations import load_start

        try:
            root = args.workspace_root.resolve()
            binding = load_binding(root)
            resolved = resolve_serial_config(config, root)
            result = inspect_config(resolved, binding=binding)
            if result['configuration_complete']:
                result['start_a'] = load_start(root, binding)['pose']
                result['stations'] = resolved['waypoints']
                result['board_candidates'] = resolved.get('board_station_candidates', ())
                result['pickup_geometry_file'] = resolved['pickup_geometry_file']
                result['board_geometry_file'] = resolved['board_geometry_file']
                result['calibration_id'] = resolved['calibration_id']
                result['max_base_alignments'] = resolved.get('max_base_alignments', 2)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            result = {'configuration_complete': False, 'motion_authorized': False,
                      'issues': [{'code': 'serial_resources', 'detail': str(exc)}]}
    else:
        with open(args.recorded_waypoints, encoding='utf-8') as stream:
            recorded = yaml.safe_load(stream)
        result = inspect_config(config, recorded)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['configuration_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
