#!/usr/bin/env python3
"""Map-bound field CLI; execution is shared with the mission ROS adapter."""
import argparse
import json
import math
from pathlib import Path
import sys

from suat_mission.navigation.navigation_profiles import speed_profile
from suat_mission.navigation.field_routes import StaticPlanner, validate_route
from suat_mission.navigation.route_runner import run_sequence
from suat_mission.navigation.map_identity import map_identity
from suat_mission.navigation.saved_grid import load_saved_grid
from suat_mission.navigation.stations import STATIONS, body_radius, destination_radius, load_start
from suat_mission.navigation.pickup_keepout import KEEP_OUT_FILE, PickupKeepout
from suat_mission.navigation.station_coverage import static_leg


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root',type=Path,default=Path.home()/'SUAT/spark')
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--route',type=Path)
    group.add_argument('--goal',nargs=3,type=float,metavar=('X','Y','YAW'))
    parser.add_argument('--station-name', choices=STATIONS, help='semantic destination for a single goal')
    offline = parser.add_mutually_exclusive_group()
    offline.add_argument('--offline-start',nargs=3,type=float,metavar=('X','Y','YAW'))
    offline.add_argument('--offline-start-a',action='store_true')
    parser.add_argument('--keepout',default=KEEP_OUT_FILE,help='map-bound central keepout for offline planning')
    parser.add_argument('--station-budget',type=float,metavar='CRUISE_SECONDS')
    parser.add_argument('--speed',choices=('slow','normal','fast'),default='slow')
    parser.add_argument('--execute',action='store_true');parser.add_argument('--site-ready',action='store_true')
    parser.add_argument('--xy-tolerance',type=float,default=.05,help='requested stopped map-position tolerance in metres')
    parser.add_argument('--yaw-tolerance',type=float,default=.08,help='requested stopped yaw tolerance in radians')
    parser.add_argument('--align-path',action='store_true',help='turn toward the live path before travel, within the same leg budget')
    parser.add_argument('--a-egress',action='store_true',help='legacy alias; initial contact recovery is automatic on the goal path')
    parser.add_argument('--cancel-distance',type=float,help='cancel first leg after .03..0.50 m, then stop the route')
    parser.add_argument('--output',type=Path)
    # ROS private remappings are handled by rospy.init_node in live mode.
    args=parser.parse_args([a for a in (sys.argv[1:] if argv is None else argv) if ':=' not in a])
    offline_mode = args.offline_start is not None or args.offline_start_a
    if offline_mode and (args.execute or not all(map(math.isfinite,args.offline_start or ()))):
        parser.error('--offline-start requires a finite pose and planning-only mode')
    if args.station_budget is not None and (args.align_path or args.cancel_distance is not None):
        parser.error('--station-budget cannot be combined with path alignment or distance cancellation')
    if args.align_path and args.cancel_distance is not None:
        parser.error('entry alignment and distance cancellation require separate trials')
    if args.execute and not args.site_ready:parser.error('--execute requires --site-ready')
    if not math.isfinite(args.xy_tolerance) or not .02<=args.xy_tolerance<=.08:parser.error('xy-tolerance must be .02..0.08 m')
    if not math.isfinite(args.yaw_tolerance) or not .04<=args.yaw_tolerance<=.12:parser.error('yaw-tolerance must be .04..0.12 rad')
    if args.cancel_distance is not None and not .03<=args.cancel_distance<=.50:parser.error('cancel-distance must be .03..0.50 m')
    report={'validation_type':'LIVE_PLAN_ONLY','motion_goal_sent':False,'legs':[],'successful':False,
            'route_completed':False,'cancel_test_passed':False}
    navigator=None
    try:
        report['start_overlap_strategy']='automatic_live_goal_path_prefix'
        keepout=None
        if offline_mode:
            import yaml
            grid=load_saved_grid(args.workspace_root)
            keepout=PickupKeepout.load(args.workspace_root,grid,args.keepout)
            config_path=args.workspace_root/'src/suat_mission/config/navigation_test.yaml'
            radius=body_radius(yaml.safe_load(config_path.read_text(encoding='utf-8')))
        else:
            import rospy
            from suat_mission.navigation.navigation_ros import Navigator
            rospy.init_node('suat_field_navigation',anonymous=False)
            navigator=Navigator(args.workspace_root,args.speed)
            grid=navigator.grid()
            keepout, radius=navigator.keepout, navigator.radius
        if args.route:route=json.loads(args.route.read_text(encoding='utf-8'))
        else:route={'schema':1,'frame_id':'map','map_image_sha256':grid['image_sha256'],
                    'map_grid_sha256':grid['grid_sha256'],'map_identity':map_identity(grid),
                    'waypoints':[{'name':args.station_name or 'target','pose':args.goal}]}
        points=validate_route(route,grid,radius_for=lambda name:destination_radius(name,radius,keepout))
        if offline_mode:
            report['validation_type']='STATIC_MAP_ONLY'
            planner=StaticPlanner(keepout.mask(grid),radius,keepout)
            start=load_start(args.workspace_root,grid)['pose'] if args.offline_start_a else args.offline_start
            for index,item in enumerate(points):
                start_name = ('Start_A' if args.offline_start_a else 'offline_start') if index==0 else points[index-1]['name']
                leg=static_leg(start_name,
                               item['name'],start,item['pose'],grid,planner,keepout)
                report['legs'].append(leg)
                if not leg['static_plan'] or leg['status']=='STATIC_MARGIN_UNRESOLVED':
                    raise ValueError(leg.get('reason',leg['status']))
                start=item['pose']
            report['successful']=True
            return 0
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
                if args.station_budget is not None:
                    import time
                    from suat_mission.contracts import ActionContext
                    from suat_mission.navigation.navigation_budget import NavigationBudget
                    budget=NavigationBudget(args.station_budget)
                    at=time.monotonic()
                    context=ActionContext('manual-field',str(at),'navigate','',0,budget.deadline(at,float('inf')))
                    leg=navigator.execute_leg(item,plan,context=context,station_approach=True,budget=budget)
                else:
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
