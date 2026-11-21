#!/usr/bin/env python3
"""Detect cake and optionally move the empty arm above it; no contact or pump."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--execute', action='store_true')
parser.add_argument('--site-ready', action='store_true')
parser.add_argument('--offset', nargs=2, type=float, metavar=('X_MM', 'Y_MM'),
                    help='measured camera-to-arm correction in arm coordinates')
args = parser.parse_args()
if args.execute and not args.site_ready:
    parser.error('--execute requires --site-ready')
if args.execute and args.offset is None:
    parser.error('actual cup was visibly misaligned; provide measured --offset X_MM Y_MM before motion')
run = root / 'reports/runs' / ('cake_approach_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
run.mkdir(parents=True, exist_ok=False)
detect = subprocess.run([sys.executable, str(root / 'src/suat_mission/scripts/arm_pick_place.py'),
                         '--target', 'cake', '--observe', 'same'], cwd=str(root),
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=35)
(run / 'detection.log').write_text(detect.stdout, encoding='utf-8')
report = {'detection_returncode': detect.returncode, 'motion_sent': False, 'pump_sent': False,
          'report_dir': str(run), 'success': False}
try:
    if detect.returncode:
        raise ValueError('cake detection failed; see detection.log')
    match = re.search(r'臂坐标 x=([-+\d.]+) y=([-+\d.]+) 顶面 z=([-+\d.]+)', detect.stdout)
    if not match:
        raise ValueError('cake coordinates missing from detection output')
    x, y, top_z = map(float, match.groups())
    # Existing teammate geometry: suction offset 82 mm, 40 mm pre-contact clearance.
    offset = args.offset or (0., 0.)
    target = [x + offset[0], y + offset[1], max(70., top_z + 82. + 40.)]
    report.update(target_xyz=target, estimated_top_z=top_z,
                  note='Candidate from teammate calibration; confirm real cup alignment before contact.')
    if args.execute:
        report['motion_sent'] = True
        move = subprocess.run([sys.executable, str(root / 'tools/arm_workpose.py'), '--target',
                               *map(str, target), '--execute', '--site-ready'], cwd=str(root),
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=100)
        (run / 'arm_workpose.log').write_text(move.stdout, encoding='utf-8')
        report['motion_returncode'] = move.returncode
        if move.returncode:
            raise ValueError('arm did not confirm arrival; see arm_workpose.log')
        report['success'] = True
except (Exception, KeyboardInterrupt) as exc:
    report['error'] = str(exc) or type(exc).__name__
(run / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
raise SystemExit(1 if 'error' in report else 0)
