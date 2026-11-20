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
from suat_mission.navigation.stations import STATIONS, body_radius, load_candidate, load_start, map_image_digest, mission_stations
from suat_mission.navigation.pickup_keepout import KEEP_OUT_FILE, PickupKeepout
from suat_mission.navigation.station_coverage import coverage, inspect_station
from suat_mission.navigation.map_identity import map_identity
from suat_mission.navigation.board_stations import board_station_candidates
from suat_mission.navigation.stations import read_json


def coverage_report(root, grid, configuration_file, candidates=None):
    import yaml
    configs = root/'src/suat_mission/config'
    configuration_file = configuration_file or configs/'mission.yaml'
    config = yaml.safe_load(configuration_file.read_text(encoding='utf-8'))
    nav_config = yaml.safe_load((configs/'navigation_test.yaml').read_text(encoding='utf-8'))
    stations = mission_stations(root,config,grid)
    if (root/'data/waypoints/start_pose_A.yaml').is_file():
        stations['Start_A'] = load_start(root,grid)
    digest = map_image_digest(grid)
    if candidates:
        for name in STATIONS:
            path = candidates/(name+'.json')
            if path.is_file():
                stations[name] = load_candidate(path,name,grid,digest)
    keepout_file=config.get('navigation_keepout_file',KEEP_OUT_FILE)
    keepout = PickupKeepout.load(root,grid,keepout_file)
    radius = body_radius(nav_config)
    report = coverage(grid,stations,radius,keepout)
    if config.get('board_station_records_file') and 'Classification_area' in stations:
        source = root/config['board_station_records_file']
        options = board_station_candidates(stations['Classification_area']['pose'],grid,read_json(source))
        report['board_station_candidates'] = [
            dict(option, **inspect_station('Classification_area',option['pose'],grid,radius,keepout))
            for option in options]
    report.update(map_identity=map_identity(grid), map_image_sha256=digest,
                  map_yaml=str(grid['map_yaml']), mission_config=str(configuration_file),
                  keepout_file=keepout_file)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root',type=Path,default=ROOT)
    parser.add_argument('--anchor',nargs=3,type=float,metavar=('X','Y','YAW'))
    parser.add_argument('--near',type=float,default=.45)
    parser.add_argument('--far',type=float,default=2.15)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check',type=Path,help='validate an existing map-bound JSON route')
    mode.add_argument('--coverage',action='store_true',help='report all 20 semantic routes on the selected map')
    parser.add_argument('--mission-config',type=Path,help='select the same station resources as the mission')
    parser.add_argument('--candidate-directory',type=Path,help='evaluate recorded candidates without enabling them')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    grid=load_saved_grid(args.workspace_root)
    if args.coverage:
        route=coverage_report(args.workspace_root,grid,args.mission_config,args.candidate_directory)
    elif args.check:
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
