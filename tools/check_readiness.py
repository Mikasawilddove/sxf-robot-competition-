#!/usr/bin/env python3
"""Read files only; never imports rospy or publishes robot commands."""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'suat_mission', 'src')))
from suat_mission.readiness import inspect_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/suat_mission/config/mission.yaml')
    parser.add_argument('--recorded-waypoints', default='data/waypoints/suat_waypoints.yaml')
    args = parser.parse_args()
    import yaml  # available on the Noetic robot; do not install anything automatically
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    with open(args.recorded_waypoints, encoding='utf-8') as stream:
        recorded = yaml.safe_load(stream)
    result = inspect_config(config, recorded)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['configuration_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
