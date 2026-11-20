"""Offline route and floor-clearance evidence for the semantic stations."""
import math

from suat_mission.navigation.corner_egress import contact_handoffs
from suat_mission.navigation.field_routes import StaticPlanner
from suat_mission.navigation.navigation_gate import goal_is_free
from suat_mission.navigation.stations import BOARD_STATION, STATIONS, board_pairs, destination_radius, route_groups


# Offline review floor only. Uses the existing 1 cm precise pickup goal
# clearance as a screen along the whole route. Field margin is measured later.
FIELD_REVIEW_FLOOR_M = .01


def inspect_station(name, pose, grid, radius, keepout):
    semantic_name = BOARD_STATION if name.startswith(BOARD_STATION+'__') else name
    target_radius = radius if semantic_name == 'Start_A' else destination_radius(semantic_name,radius,keepout)
    return {'required_radius_m': target_radius,
            'footprint_clear': goal_is_free(grid,*pose[:2],target_radius),
            'keepout_clear': keepout.distance(pose)>target_radius}


def conservative_clearance(path, grid, keepout, body_radius):
    # The exact clearance is 1-Lipschitz along the path. Sample within 1 cm
    # and subtract half an interval to bound clearance between the samples.
    step = min(grid['resolution']/2,.01)
    minimum = float('inf')
    for start, end in zip(path,path[1:]):
        count = max(1, math.ceil(math.dist(start[:2],end[:2])/step))
        for i in range(count+1):
            fraction = i/count
            point = [a+(b-a)*fraction for a,b in zip(start[:2],end[:2])]
            left, right = 0., min(.5,keepout.distance(point))
            while right-left>.002:
                mid=(left+right)/2
                if mid>0 and goal_is_free(grid,*point,mid):
                    left=mid
                else:
                    right=mid
            minimum=min(minimum,left)
    return minimum-body_radius-step/2


def static_leg(start_name, goal_name, start, goal, grid, planner, keepout):
    start_check = inspect_station(start_name,start,grid,planner.radius,keepout)
    goal_check = inspect_station(goal_name,goal,grid,planner.radius,keepout)
    row = {'start': start_name, 'goal': goal_name, 'start_yaw_rad': start[2], 'goal_yaw_rad': goal[2],
           'static_plan': False, 'footprint': 'BLOCKED', 'keepout': 'CLEAR',
           'status': 'BLOCKED', 'path_length_m': None, 'min_clearance_lower_bound_m': None,
           'final_alignment': 'UNVERIFIED', 'departure': None}
    if not start_check['keepout_clear'] or not goal_check['keepout_clear']:
        return dict(row,keepout='BLOCKED',reason='station encroaches on central keepout')
    if not goal_check['footprint_clear'] or (not start_check['footprint_clear'] and start_name!='Start_A'):
        return dict(row,reason='station footprint overlaps occupied/unknown space')
    row.update(footprint='CLEAR',final_alignment='STATIC_CIRCULAR_TURN_CLEAR')
    starts = [(0.,start,None)]
    if not start_check['footprint_clear']:
        starts = contact_handoffs(grid,start,goal,planner.radius)
        row['footprint']='START_CONTACT'
    for _, origin, prefix in starts:
        if prefix and not keepout.segment_clear(start,origin,planner.radius):
            continue
        try:
            path = planner.plan(origin,goal)
        except ValueError as exc:
            row['reason']=str(exc)
            continue
        if not all(keepout.segment_clear(a,b,planner.radius) for a,b in zip(path,path[1:])):
            continue
        length=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
        if prefix:
            length+=math.dist(start[:2],origin[:2])
        if length>8.:
            row['reason']='one leg exceeds 8 m'
            continue
        row.pop('reason',None)
        margin=conservative_clearance(path,grid,keepout,planner.radius)
        status='STATIC_WITH_DEPARTURE' if prefix else 'STATIC_PASS'
        if margin<=0:
            status='STATIC_MARGIN_UNRESOLVED'
        return dict(row, static_plan=True, status=status,
                    path_length_m=length, departure=prefix, handoff_pose=origin if prefix else None,
                    min_clearance_lower_bound_m=margin,
                    clearance_excludes_initial_contact=bool(prefix))
    row.setdefault('reason','no clear path including any required A contact departure')
    row['keepout']='ENFORCED_NO_PATH'
    return row


def coverage(grid, stations, radius, keepout, *, board_candidates=()):
    planner=StaticPlanner(keepout.mask(grid), radius, keepout)
    endpoints = {name: (dict(stations[name], **inspect_station(name,stations[name]['pose'],grid,radius,keepout))
                        if name in stations else {'validation_status':'MISSING_RECORD'})
                 for name in STATIONS}

    def check(pairs, points):
        rows = []
        for start,goal in pairs:
            missing = [name for name in (start,goal) if name not in points]
            if missing:
                rows.append({'start':start,'goal':goal,'static_plan':False,'footprint':'UNKNOWN',
                             'keepout':'NOT_EVALUATED','status':'MISSING_RECORD','missing_stations':missing})
            else:
                row = static_leg(start,goal,points[start]['pose'],points[goal]['pose'],grid,planner,keepout)
                row['static_status'] = row['status']
                if row['static_plan'] and row['min_clearance_lower_bound_m'] < FIELD_REVIEW_FLOOR_M:
                    row.update(status='FIELD_REVIEW_REQUIRED',
                               review_reason='clearance lower bound below offline review floor')
                rows.append(row)
        return rows

    groups = {group:check(pairs,stations) for group,pairs in route_groups().items()}
    alternatives = []
    for candidate in board_candidates:
        name = BOARD_STATION+'__'+candidate['id']
        points = dict(stations)
        points[name] = candidate
        alternatives.append(dict(candidate, name=name,
            **inspect_station(name,candidate['pose'],grid,radius,keepout),
            routes=check(board_pairs(name),points)))
    return {'validation_type':'STATIC_MAP_ONLY','physical_navigation':'NOT_RUN',
            'body_radius_m':radius,'field_review_floor_m':FIELD_REVIEW_FLOOR_M,
            'stations':endpoints,'board_station_candidates':alternatives,**groups}
