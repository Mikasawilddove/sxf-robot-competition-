#!/usr/bin/env python3
"""Resolve the selected map and any fitted field transform offline."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/suat_mission/src'))
from suat_mission.navigation_map import load_binding
from suat_mission.map_identity import map_identity


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root',type=Path,default=ROOT)
    parser.add_argument('--path',action='store_true',help='print only the resolved map YAML path')
    args=parser.parse_args()
    try:
        binding=load_binding(args.workspace_root)
        field=binding.get('field_alignment')
        print(binding['map_yaml'] if args.path else json.dumps({'map_file':binding['map_yaml'],
              'alignment_version':field['alignment_version'] if field else binding['alignment_version'],
              'field_alignment_file':binding.get('field_alignment_file'),
              'field_to_map':({k:field[k] for k in ('theta_rad','translation_m','check_rms_m')}
                              if field else None),
              'map_identity':map_identity(binding),'robot_connected':False},indent=2))
        return 0
    except (ValueError,OSError,KeyError) as exc:
        parser.exit(2,'selected map refused: '+str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
