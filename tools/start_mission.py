#!/usr/bin/env python3
"""Open the scoped single-Enter ROS console. --help never imports ROS."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'suat_mission' / 'src'))

if __name__ == '__main__':
    runpy.run_path(str(ROOT / 'src' / 'suat_mission' / 'scripts' / 'mission_console.py'), run_name='__main__')
