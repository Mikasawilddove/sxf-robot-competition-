#!/usr/bin/env python3
"""Map-bound field CLI; execution is shared with the mission ROS adapter."""
import argparse
import json
import math
from pathlib import Path
import rospy

from suat_mission.navigation.navigation_ros import Navigator
from suat_mission.navigation.navigation_profiles import speed_profile
from suat_mission.navigation.navigation_gate import GOAL_PREFILTER_RADIUS_M
from suat_mission.navigation.field_routes import validate_route
from suat_mission.navigation.route_runner import run_sequence
from suat_mission.navigation.map_identity import map_identity


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root',type=Path,default=Path.home()/'SUAT/spark')
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--route',type=Path)
    group.add_argument('--goal',nargs=3,type=float,metavar=('X','Y','YAW'))
    parser.add_argument('--speed',choices=('slow','normal','fast'),default='slow')
    parser.add_argument('--execute',action='store_true');parser.add_argument('--site-ready',action='store_true')
    parser.add_argument('--xy-tolerance',type=float,default=.05,help='requested stopped map-position tolerance in metres')
    parser.add_argument('--yaw-tolerance',type=float,default=.08,help='requested stopped yaw tolerance in radians')
    parser.add_argument('--align-path',action='store_true',help='turn toward the live path before travel, within the same leg budget')
    parser.add_argument('--a-egress',action='store_true',help='legacy alias; initial contact recovery is automatic on the goal path')
    parser.add_argument('--cancel-distance',type=float,help='cancel first leg after .03..0.50 m, then stop the route')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(rospy.myargv()[1:])
    if args.align_path and args.cancel_distance is not None:
        parser.error('entry alignment and distance cancellation require separate trials')
    if args.execute and not args.site_ready:parser.error('--execute requires --site-ready')
    if not math.isfinite(args.xy_tolerance) or not .02<=args.xy_tolerance<=.08:parser.error('xy-tolerance must be .02..0.08 m')
    if not math.isfinite(args.yaw_tolerance) or not .04<=args.yaw_tolerance<=.12:parser.error('yaw-tolerance must be .04..0.12 rad')
    if args.cancel_distance is not None and not .03<=args.cancel_distance<=.50:parser.error('cancel-distance must be .03..0.50 m')
    rospy.init_node('suat_field_navigation',anonymous=False)
    report={'validation_type':'LIVE_PLAN_ONLY','motion_goal_sent':False,'legs':[],'successful':False,
            'route_completed':False,'cancel_test_passed':False}
    navigator=None
    try:
        report['start_overlap_strategy']='automatic_live_goal_path_prefix'
        navigator=Navigator(args.workspace_root,args.speed);grid=navigator.grid()
        if args.route:route=json.loads(args.route.read_text(encoding='utf-8'))
        else:route={'schema':1,'frame_id':'map','map_image_sha256':navigator.binding['image_sha256'],
                    'map_grid_sha256':navigator.binding['grid_sha256'],
                    'map_identity':map_identity(navigator.binding),
                    'waypoints':[{'name':'target','pose':args.goal}]}
        # The grid verifier returns geometry/cells; identity is already verified.
        grid.update(image_sha256=navigator.binding['image_sha256'],grid_sha256=navigator.binding['grid_sha256'])
        points=validate_route(route,grid,radius=GOAL_PREFILTER_RADIUS_M)
        start=navigator.pose();plans=[]
        if args.cancel_distance is not None and math.dist(start[:2],points[0]['pose'][:2])<args.cancel_distance+.16:
            raise ValueError('cancel-test first target is too close; choose a distant point such as D_inside')
        for item in points:
            plans.append(navigator.plan(start,item['pose'],grid));start=item['pose']
        report.update(speed=args.speed,limits=speed_profile(args.speed),route=route,plans=plans)
        print('[导航] 请求停稳容差：位置 %.3f m，朝向 %.3f rad；实际精度需现场测量' % (args.xy_tolerance,args.yaw_tolerance),flush=True)
        if args.execute:
            navigator.connect_control();report['validation_type']='LIVE_FIELD_NAVIGATION'
            route_index=[0];previous=['无（第一段）'];departure=['当前估计位置']
            def current_plan(item):
                return navigator.plan(navigator.pose(),item['pose'],navigator.grid())
            def execute(item,plan,cancel):
                route_index[0]+=1
                print('[路线] 第 %d 段：%s → %s，开始；上一段结果：%s' %
                      (route_index[0],departure[0],item['name'],previous[0]),flush=True)
                leg=navigator.execute_leg(item,plan,args.cancel_distance if cancel else None,align_path=args.align_path,
                    xy_tolerance=args.xy_tolerance,yaw_tolerance=args.yaw_tolerance)
                report['motion_goal_sent']|=leg['motion_goal_sent']
                outcome='成功' if leg.get('passed') else '失败 / 超时'
                print('[路线] 第 %d 段：%s → %s，%s' %
                      (route_index[0],departure[0],item['name'],outcome),flush=True)
                if leg.get('error'):
                    print('[路线] 失败或超时原因：%s' % leg['error'],flush=True)
                previous[0]=outcome;departure[0]=item['name']
                return leg
            report.update(run_sequence(points,current_plan,execute,report['legs'],args.cancel_distance is not None))
            print('[路线] 当前状态：%s' % ('完成' if report.get('route_completed') else '停止，等待处理'),flush=True)
        report['successful']=True
        return 0
    except Exception as exc:
        report['error']=str(exc);return 2
    finally:
        if navigator is not None:navigator.close()
        report['physical_validation']='requires on-site observation' if args.execute else 'NOT_RUN'
        destination=args.output or args.workspace_root/'reports/field_navigation/last_run.json'
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=True,indent=2),flush=True)


if __name__=='__main__':raise SystemExit(main())
