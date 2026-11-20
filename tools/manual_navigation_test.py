#!/usr/bin/env python3
"""One-station field navigation test; only --execute commands motion."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/suat_mission/src'))
from suat_mission.saved_grid import load_saved_grid, flat_map_metadata
from suat_mission.map_identity import validate_map_identity

STATIONS = {
    'Classification_area': 'data/navigation/recorded_stations_20261115.json',
    'Sorting_W': 'data/navigation/recorded_stations_20261115.json',
    'Collection_A': 'data/navigation/recorded_stations_20261115.json',
    'Sorting_N': 'data/waypoints/collection_bcd_candidates_20261117.json',
    'Collection_B': 'data/waypoints/collection_bcd_candidates_20261117.json',
    'Collection_C': 'data/waypoints/collection_bcd_candidates_20261117.json',
    'Collection_D': 'data/waypoints/collection_bcd_candidates_20261117.json',
}

def resolve_station(root, station):
    binding = load_saved_grid(root)
    path = root / STATIONS[station]
    record = json.loads(path.read_text(encoding='utf-8'))
    if record.get('schema') != 1 or record.get('frame_id') != 'map':
        raise ValueError('\u70b9\u4f4d\u6587\u4ef6\u683c\u5f0f\u6216\u5750\u6807\u7cfb\u4e0d\u6b63\u786e')
    validate_map_identity(record, binding)
    map_yaml = Path(binding['map_yaml']).resolve()
    map_yaml.relative_to(root.resolve())
    meta = flat_map_metadata(map_yaml)
    image = (map_yaml.parent / meta['image']).resolve()
    image.relative_to(root.resolve())
    image_digest = hashlib.sha256(image.read_bytes()).hexdigest()
    if not binding.get('image_sha256') or image_digest != binding['image_sha256']:
        raise ValueError('\u5b9e\u9645\u5730\u56fe\u6587\u4ef6\u4e0e\u6240\u9009\u5730\u56fe\u8eab\u4efd\u4e0d\u4e00\u81f4')
    if record.get('map_image_sha256') != image_digest:
        raise ValueError('\u70b9\u4f4d\u8bb0\u5f55\u4e0e\u5f53\u524d\u5730\u56fe\u6587\u4ef6\u4e0d\u5339\u914d')
    points = record.get('waypoints')
    if isinstance(points, list):
        matches = [item.get('pose') for item in points
                   if isinstance(item, dict) and item.get('name') == station]
        if len(matches) != 1:
            raise ValueError('\u70b9\u4f4d\u4e0d\u5b58\u5728\u6216\u540d\u79f0\u91cd\u590d')
        point = matches[0]
    elif isinstance(points, dict):
        point = points.get(station)
    else:
        raise ValueError('\u70b9\u4f4d\u6587\u4ef6\u5185\u5bb9\u4e0d\u6b63\u786e')
    if (not isinstance(point, (list, tuple)) or len(point) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in point)):
        raise ValueError('\u70b9\u4f4d\u5fc5\u987b\u5305\u542b\u6709\u9650\u6570\u503c x\u3001y\u3001yaw')
    return [float(v) for v in point], str(path)

def run_stage(root, station_pose, report_path, log_path, execute):
    command = [sys.executable, str(root / 'src/suat_mission/scripts/field_navigation.py'),
               '--goal'] + [repr(value) for value in station_pose] + [
               '--speed', 'slow', '--align-path', '--output', str(report_path)]
    if execute:
        command += ['--execute', '--site-ready']
    with log_path.open('w', encoding='utf-8') as output:
        process = subprocess.run(command, cwd=str(root), stdout=output,
                                 stderr=subprocess.STDOUT, check=False)
    try:
        report = json.loads(report_path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        report = {}
    return process.returncode, report


def print_navigation_result(station, point, result):
    legs = result.get('legs', [])
    leg = legs[-1] if legs else {}
    state = '成功' if leg.get('passed') else '失败 / 超时'
    print('[导航] 目标点：%s' % station, flush=True)
    print('[导航] 目标位姿：x=%.3f m，y=%.3f m，朝向=%.1f°' %
          (point[0], point[1], math.degrees(point[2])), flush=True)
    print('[导航] 状态：%s' % state, flush=True)
    if isinstance(leg.get('elapsed_s'), (int, float)):
        print('[导航] 耗时：%.2f s' % leg['elapsed_s'], flush=True)
    actual = leg.get('actual_pose')
    if isinstance(actual, list) and len(actual) >= 3:
        print('[导航] 到达时估计位姿：x=%.3f m，y=%.3f m，朝向=%.1f°' %
              (actual[0], actual[1], math.degrees(actual[2])), flush=True)
    reason = result.get('error') or leg.get('error')
    if reason:
        print('[导航] 原因：%s' % reason, flush=True)

def main(argv=None):
    parser = argparse.ArgumentParser(description='\u5355\u70b9\u5bfc\u822a\u6d4b\u8bd5\uff1a\u9ed8\u8ba4\u53ea\u89c4\u5212\uff1b\u52a0 --execute \u4f1a\u8ba9\u5e95\u76d8\u79fb\u52a8\u3002')
    parser.add_argument('--station', choices=sorted(STATIONS), default='Classification_area',
                        help='\u76ee\u6807\u70b9\u540d\u79f0\uff0c\u9ed8\u8ba4 Classification_area')
    parser.add_argument('--execute', action='store_true',
                        help='\u5148\u68c0\u67e5\u89c4\u5212\uff0c\u6210\u529f\u540e\u7acb\u5373\u6267\u884c\u771f\u5b9e\u5e95\u76d8\u5bfc\u822a')
    args = parser.parse_args(argv)
    try:
        point, source = resolve_station(ROOT, args.station)
    except (OSError, ValueError, KeyError) as exc:
        print('\u70b9\u4f4d\u68c0\u67e5\u5931\u8d25\uff1a%s' % exc, file=sys.stderr, flush=True)
        return 2
    runs = ROOT / 'reports/runs'
    runs.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix='manual_navigation_', dir=str(runs)))
    print('\u76ee\u6807\u70b9\uff1a%s  \u4f4d\u59ff\uff1a%s' % (args.station, point), flush=True)
    print('[导航] 出发点：当前估计位置（由执行器在发送前读取）', flush=True)
    print('[导航] 测试请求时间：%s；规划模式不会发送运动目标' % time.strftime('%Y-%m-%d %H:%M:%S'), flush=True)
    print('\u70b9\u4f4d\u6587\u4ef6\uff1a%s' % source, flush=True)
    print('\u8bb0\u5f55\u76ee\u5f55\uff1a%s' % run_dir, flush=True)
    print('\u6b63\u5728\u53ea\u89c4\u5212\u68c0\u67e5\uff0c\u5e95\u76d8\u4e0d\u79fb\u52a8\u2026\u2026', flush=True)
    try:
        code, plan = run_stage(ROOT, point, run_dir / 'plan.json', run_dir / 'plan.log', False)
        if (code != 0 or plan.get('successful') is not True
                or plan.get('validation_type') != 'LIVE_PLAN_ONLY'
                or plan.get('motion_goal_sent') is not False):
            print('\u89c4\u5212\u5931\u8d25\u6216\u62a5\u544a\u5f02\u5e38\uff0c\u672a\u542f\u52a8\u771f\u5b9e\u5bfc\u822a\u3002\u539f\u56e0\uff1a%s' % plan.get('error', '\u8bf7\u67e5\u770b plan.log'), flush=True)
            return 2
        plans = plan.get('plans', [])
        if len(plans) != 1 or not isinstance(plans[0].get('path_length_m'), (int, float)):
            print('\u89c4\u5212\u62a5\u544a\u4e0d\u5b8c\u6574\uff0c\u672a\u542f\u52a8\u771f\u5b9e\u5bfc\u822a\u3002', flush=True)
            return 2
        print('\u89c4\u5212\u901a\u8fc7\uff0c\u8def\u7ebf\u957f\u5ea6\u7ea6 %.2f \u7c73\u3002' % plans[0]['path_length_m'], flush=True)
        if not args.execute:
            print('\u53ea\u89c4\u5212\u5b8c\u6210\uff0c\u672a\u53d1\u9001\u8fd0\u52a8\u76ee\u6807\u3002\u771f\u673a\u6267\u884c\u65f6\u8ffd\u52a0 --execute\u3002', flush=True)
            return 0
        print('\u5f00\u59cb\u771f\u673a\u5bfc\u822a\uff1a\u5e95\u76d8\u5373\u5c06\u79fb\u52a8\uff1b\u8bf7\u73b0\u573a\u89c2\u5bdf\uff0c\u5f02\u5e38\u65f6\u4f7f\u7528\u7269\u7406\u6025\u505c\u3002', flush=True)
        print('[导航] 执行请求时间：%s；实际 goal 在门禁放行后发送' % time.strftime('%Y-%m-%d %H:%M:%S'),flush=True)
        code, result = run_stage(ROOT, point, run_dir / 'execute.json', run_dir / 'execute.log', True)
        print_navigation_result(args.station, point, result)
        legs = result.get('legs', [])
        if (code == 0 and result.get('successful') is True
                and result.get('route_completed') is True and len(legs) == 1
                and legs[0].get('passed') is True
                and legs[0].get('arrival_confirmed') is True
                and legs[0].get('output_released') is True):
            print('\u7a0b\u5e8f\u62a5\u544a\uff1a\u5230\u70b9\u3001\u505c\u8f66\u68c0\u67e5\u53ca\u63a7\u5236\u91ca\u653e\u901a\u8fc7\u3002\u8bf7\u73b0\u573a\u6838\u5bf9\u5b9e\u7269\u4f4d\u7f6e\u548c\u671d\u5411\u3002', flush=True)
            return 0
        detail = result.get('error') or (legs[-1].get('error') if legs else None) or '\u8bf7\u67e5\u770b execute.log'
        print('\u5bfc\u822a\u672a\u901a\u8fc7\uff1a%s\u3002\u8bf7\u67e5\u770b\u62a5\u544a\u5e76\u786e\u8ba4\u673a\u5668\u4eba\u5df2\u505c\u7a33\u3002' % detail, flush=True)
        return 2
    except KeyboardInterrupt:
        print('\u5df2\u8bf7\u6c42\u4e2d\u65ad\uff1b\u8bf7\u7acb\u5373\u786e\u8ba4\u5e95\u76d8\u505c\u7a33\u3002\u5fc5\u8981\u65f6\u4f7f\u7528\u7269\u7406\u6025\u505c\u3002', file=sys.stderr, flush=True)
        return 130

if __name__ == '__main__':
    raise SystemExit(main())
