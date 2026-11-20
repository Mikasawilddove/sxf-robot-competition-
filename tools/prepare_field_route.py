#!/usr/bin/env python3
"""Offline saved-map route preparation. Never connects to ROS or the robot."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/suat_mission/src'))
from suat_mission.saved_grid import load_saved_grid
from suat_mission.field_routes import loop_candidate,validate_route,StaticPlanner
from suat_mission.start_pose import read_start_pose,validate_start_pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root',type=Path,default=ROOT)
    parser.add_argument('--anchor',nargs=3,type=float,metavar=('X','Y','YAW'))
    parser.add_argument('--near',type=float,default=.45)
    parser.add_argument('--far',type=float,default=2.15)
    parser.add_argument('--check',type=Path,help='validate an existing map-bound JSON route')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    grid=load_saved_grid(args.workspace_root)
    if args.check:
        route=json.loads(args.check.read_text(encoding='utf-8'))
        points=validate_route(route,grid);planner=StaticPlanner(grid)
        for a,b in zip(points,points[1:]):planner.plan(a['pose'],b['pose'])
    else:
        if args.anchor is None:
            reference=json.loads((args.workspace_root/'data/navigation/arena_reference.json').read_text(encoding='utf-8'))
            source=(args.workspace_root/reference['source_file']).resolve()
            source.relative_to(args.workspace_root.resolve())
            if hashlib.sha256(source.read_bytes()).hexdigest()!=reference['source_sha256']:
                raise ValueError('saved A reference changed; regenerate the reference')
            try:
                anchor=validate_start_pose(read_start_pose(source),grid)
            except ValueError as exc:
                raise ValueError('Start_A reference is not valid for the selected map: '+str(exc)) from exc
            if list(anchor) != reference['pose']:
                raise ValueError('Start_A reference pose differs from its bound source record')
        else:anchor=args.anchor
        route=loop_candidate(grid,anchor,args.near,args.far)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(route,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in route.items() if k not in ('static_paths',)},ensure_ascii=True,indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
