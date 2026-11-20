#!/usr/bin/env python3
"""Pure-Python regression runner. Never imports the real ROS adapter as a node."""
import argparse
import json
from dataclasses import asdict
from itertools import permutations
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'suat_mission' / 'src'))
from suat_mission.mission import Mission
from suat_mission.perception import OBJECTS
from suat_mission.simulator import SimRobot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--include-vision', action='store_true', help='requires existing OpenCV/numpy; installs nothing')
    args = parser.parse_args()
    suite = unittest.TestSuite()
    patterns = ['test_algorithms.py', 'test_alignment.py', 'test_m1_contracts.py', 'test_ros_adapter_contract.py',
                'test_maintenance.py', 'test_navigation_gate.py', 'test_start_pose.py', 'test_navigation_test_geometry.py',
                'test_field_navigation.py', 'test_field_navigation_adapter.py']
    if args.include_vision:
        patterns += ['test_template_matcher.py', 'test_real_board_photo.py', 'test_real_cake_photo.py', 'test_field_image.py',
                     'test_experimental_vision.py', 'test_surface_geometry.py', 'test_navigation_map.py']
    loader = unittest.TestLoader()
    for pattern in patterns:
        suite.addTests(loader.discover(str(ROOT / 'tests'), pattern=pattern))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    scenarios = []
    for order in permutations(OBJECTS):
        scenarios.append({'fixture': 'synthetic_board_' + ','.join(order),
                          'report': asdict(Mission(SimRobot(order)).run())})
    for failure in ('navigate:Classification_area', 'pick:cake', 'navigate:Collection_A', 'drop:cake'):
        scenarios.append({'fixture': 'synthetic_failure_' + failure,
                          'report': asdict(Mission(SimRobot(fail_at=failure)).run())})
    report = {'validation_type': 'offline_synthetic_and_recorded_images' if args.include_vision else 'offline_synthetic',
              'real_motion_validation': 'NOT_RUN', 'tests_run': result.testsRun,
              'failures': len(result.failures), 'errors': len(result.errors),
              'skipped': len(result.skipped),
              'successful': result.wasSuccessful(), 'scenarios': scenarios}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'scenarios'}, indent=2))
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
