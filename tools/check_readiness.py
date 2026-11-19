#!/usr/bin/env python3
"""Read files only; never imports rospy or publishes robot commands."""
import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'suat_mission', 'src')))
from suat_mission.readiness import inspect_config
from suat_mission.navigation_map import load_binding


def read_mapping(path):
    with open(path, encoding='utf-8-sig') as stream:
        if Path(path).suffix.lower() == '.json':
            value = json.load(stream)
        else:
            try:
                import yaml
            except ImportError as exc:
                raise ValueError('YAML reader unavailable; use JSON for offline diagnostics; nothing was installed') from exc
            value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError('configuration must be a mapping')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/suat_mission/config/mission.yaml')
    parser.add_argument('--recorded-waypoints', default='data/waypoints/suat_waypoints.yaml')
    parser.add_argument('--scope', choices=('cake-only', 'full'), help='override only the selected diagnostic scope')
    args = parser.parse_args()
    try:
        config = read_mapping(args.config)
        if args.scope:
            config['mission_scope'] = args.scope
        try:
            recorded = read_mapping(args.recorded_waypoints)
            recorded_error = None
        except (OSError, ValueError) as exc:
            recorded, recorded_error = {}, str(exc)
        result = inspect_config(config, recorded, binding=load_binding(Path(__file__).resolve().parents[1]))
        result['recorded_file_error'] = recorded_error
    except (OSError, ValueError, TypeError) as exc:
        result = {'mode': 'read_only', 'motion_authorized': False, 'configuration_complete': False,
                  'issues': [{'code': 'file.config', 'detail': str(exc)}]}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['configuration_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
