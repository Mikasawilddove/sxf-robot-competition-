"""A-corner straight departure, using the unchanged circular collision radius.

The starting footprint may already overlap the wall in the coarse map.  Only
components touching the footprint initially may overlap during the departure;
their closest distance must not decrease, unknown cells are never entered, and
the complete laser-observed forward swept footprint must stay clear.
"""
import math
from collections import deque

DISTANCE = .55
SPEED = .04
REAR_ASSOCIATION_M = .010
REAR_CLOSING_TOLERANCE_M = .005
MAX_RECHECK_NEIGHBOR_RAD = math.radians(3)
MAX_RECHECK_GAP_M = .020
CONTACT_WALL_BAND_M = .020


class SweepBlocked(ValueError):
    def __init__(self, side, point, beam_count):
        super().__init__({
            'forward': '激光检测到起步扫掠范围有障碍',
            'rear': '起步时后侧检测到未匹配的近障碍',
        }[side])
        self.side = side
        self.point = point
        self.beam_count = beam_count


def anchor_check(pose, anchor):
    if len(pose) != 3 or len(anchor) != 3 or not all(math.isfinite(v) for v in (*pose, *anchor)):
        raise ValueError('A起点定位无效')
    yaw_error = abs(math.atan2(math.sin(pose[2] - anchor[2]), math.cos(pose[2] - anchor[2])))
    if math.dist(pose[:2], anchor[:2]) > .15 or yaw_error > .30:
        raise ValueError('当前位置或朝向不符合已记录的 A 起点')


def _nearby(grid, x, y, radius):
    r = grid['resolution']; ox, oy = grid['origin']
    cx = math.floor((x-ox)/r); cy = math.floor((y-oy)/r)
    extent = int(math.ceil(radius/r)) + 1
    for j in range(cy-extent, cy+extent+1):
        for i in range(cx-extent, cx+extent+1):
            dx=max(ox+i*r-x,0.,x-(ox+(i+1)*r))
            dy=max(oy+j*r-y,0.,y-(oy+(j+1)*r))
            distance=math.hypot(dx,dy)
            if distance>radius:
                continue
            if not (0<=i<grid['width'] and 0<=j<grid['height']):
                raise ValueError('起步范围超出已知地图')
            yield (i,j), grid['data'][j*grid['width']+i], distance


def _reachable_free(grid, pose):
    r=grid['resolution']; ox,oy=grid['origin']; w,h=grid['width'],grid['height']
    seed=(math.floor((pose[0]-ox)/r),math.floor((pose[1]-oy)/r))
    if not (0<=seed[0]<w and 0<=seed[1]<h) or grid['data'][seed[1]*w+seed[0]]!=0:
        raise ValueError('A起点中心不在已知自由地图内')
    reached={seed}; pending=deque([seed])
    while pending:
        i,j=pending.popleft()
        for di,dj in ((-1,0),(1,0),(0,-1),(0,1)):
            x,y=i+di,j+dj
            if 0<=x<w and 0<=y<h and (x,y) not in reached and grid['data'][y*w+x]==0:
                reached.add((x,y)); pending.append((x,y))
    return reached


def _unknown_enterable(cell, reached):
    i,j=cell
    return any((i+di,j+dj) in reached for di in (-1,0,1) for dj in (-1,0,1))


def map_departure(grid, pose, radius, distance=DISTANCE):
    if not math.isfinite(radius) or not .10 <= radius <= .50 or not 0 < distance <= .60:
        raise ValueError('起步几何参数无效')
    reached=_reachable_free(grid,pose)
    start=list(_nearby(grid,pose[0],pose[1],radius))
    if any(v<0 and _unknown_enterable(c,reached) for c,v,_ in start):
        raise ValueError('A起点车身范围包含未知地图')
    contacts={cell for cell,value,_ in start if value>0}
    if not contacts:
        raise ValueError('起点没有已知墙角重叠，无需特殊驶离')
    initial_min=min(d for _,v,d in start if v>0)

    # 8-connected walls from the initial overlaps: never silently exempt a
    # separate obstacle or unknown space that the robot may encounter later.
    wall=set(contacts); pending=deque(contacts); width=grid['width']; height=grid['height']
    while pending:
        i,j=pending.popleft()
        for di,dj in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
            x,y=i+di,j+dj
            if (0<=x<width and 0<=y<height and (x,y) not in wall
                    and grid['data'][y*width+x]>0):
                wall.add((x,y));pending.append((x,y))

    map_wall_tolerance=math.sqrt(2.)*grid['resolution']
    worst=initial_min
    for step in range(1,math.ceil(distance/.02)+1):
        d=min(distance,step*.02)
        x=pose[0]+d*math.cos(pose[2]); y=pose[1]+d*math.sin(pose[2])
        cells=list(_nearby(grid,x,y,radius))
        if any(v<0 and _unknown_enterable(c,reached) for c,v,_ in cells):
            raise ValueError('起步扫掠范围包含未知地图')
        if any(v>0 and c not in wall for c,v,_ in cells):
            raise ValueError('起步扫掠范围存在新增地图障碍')
        wall_distances=[dist for c,v,dist in cells if v>0 and c in wall]
        if wall_distances:
            nearest=min(wall_distances)
            worst=min(worst,nearest)
            if nearest < initial_min-map_wall_tolerance:
                raise ValueError('起步路径进一步接近初始墙面')
    endpoint=[pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2]),pose[2]]
    endpoint_cells=list(_nearby(grid,endpoint[0],endpoint[1],radius))
    if any((v>0 and c not in wall) or (v<0 and _unknown_enterable(c,reached)) for c,v,_ in endpoint_cells):
        raise ValueError('起步终点包含新增障碍或可进入的未知地图')
    return {'start':list(pose), 'end':endpoint, 'distance_m':distance,
            'radius_m':radius, 'initial_wall_distance_m':initial_min,
            'worst_wall_distance_m':worst, 'initial_map_overlap':True,
            'map_wall_raster_tolerance_m':map_wall_tolerance,
            'endpoint_initial_wall_overlap':any(v>0 for _,v,_ in endpoint_cells),
            'unknown_policy':'only_unreachable_cells_behind_known_walls_excluded'}


def swept_clearance(x, y, distance):
    return math.hypot(x - min(distance, max(0., x)), y)


def contact_path(grid, pose, points, radius, wall_reference=None, _cache=None, static_contact=False):
    """Permit only the original connected-wall overlap until the path escapes it."""
    reached=(_cache['reached'] if _cache and 'reached' in _cache else _reachable_free(grid,pose))
    start=list(_nearby(grid,pose[0],pose[1],radius))
    contacts={cell for cell,value,_ in start if value>0}
    if any(value<0 and _unknown_enterable(cell,reached) for cell,value,_ in start):
        raise ValueError('initial contact includes reachable unknown space')
    walls=set(_cache['walls']) if _cache and 'walls' in _cache else set(contacts)
    pending=deque() if _cache and 'walls' in _cache else deque(contacts)
    w,h=grid['width'],grid['height']
    while pending:
        i,j=pending.popleft()
        for di,dj in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
            x,y=i+di,j+dj
            if 0<=x<w and 0<=y<h and (x,y) not in walls and grid['data'][y*w+x]>0:
                walls.add((x,y));pending.append((x,y))
    if _cache is not None:_cache.update(reached=reached,walls=walls)
    initial=min((d for _,v,d in start if v>0),default=radius)
    tolerance=math.sqrt(2.)*grid['resolution']
    escaped=not static_contact and not any(v!=0 for _,v,_ in start)
    r=grid['resolution'];ox,oy=grid['origin'];travel=0.
    previous=tuple(pose[:2]);escape_at=0. if escaped else None
    for endpoint in points:
        endpoint=tuple(endpoint[:2]);length=math.dist(previous,endpoint)
        count=max(1,math.ceil(length/.02))
        for step in range(1,count+1):
            x=previous[0]+(endpoint[0]-previous[0])*step/count
            y=previous[1]+(endpoint[1]-previous[1])*step/count
            i,j=math.floor((x-ox)/r),math.floor((y-oy)/r)
            if not (0<=i<w and 0<=j<h) or grid['data'][j*w+i]!=0:
                raise ValueError('recovery path centre enters occupied or unknown map')
            cells=list(_nearby(grid,x,y,radius))
            for cell,value,d in cells:
                reference_match=False
                if value>0 and wall_reference is not None:
                    i,j=cell;wx,wy=ox+(i+.5)*r,oy+(j+.5)*r
                    rx,ry,rr,reference=wall_reference
                    ri,rj=math.floor((wx-rx)/rr),math.floor((wy-ry)/rr)
                    reference_match=any((ri+di,rj+dj) in reference for di in (-1,0,1) for dj in (-1,0,1))
                if value<0 and _unknown_enterable(cell,reached):
                    raise ValueError('recovery path reaches an unknown frontier')
                if value>0 and (escaped or (cell not in walls and not (static_contact and reference_match))):
                    raise ValueError('recovery path enters a new map obstacle')
                if value>0:
                    i,j=cell
                    initial_distance=math.hypot(max(ox+i*r-pose[0],0.,pose[0]-(ox+(i+1)*r)),
                                                max(oy+j*r-pose[1],0.,pose[1]-(oy+(j+1)*r)))
                    if d<min(radius,initial_distance)-tolerance:
                        raise ValueError('recovery path approaches the initial wall beyond raster uncertainty')
                    if wall_reference is not None:
                        wx,wy=ox+(i+.5)*r,oy+(j+.5)*r
                        if not reference_match:
                            raise ValueError('new live obstacle appeared on the departure path')
            if not static_contact and not escaped and all(value==0 for _,value,_ in cells):
                escaped=True;escape_at=travel+length*step/count
        travel+=length;previous=endpoint
    return {'initial_wall_overlap':bool(contacts),'escaped_initial_overlap':escaped,
            'escape_distance_m':escape_at,'path_length_m':travel,
            'raster_uncertainty_m':tolerance,'radius_m':radius}


def contact_handoffs(grid,pose,goal,radius):
    """Find short, non-closing escape connections before asking the live planner.

    make_plan may reject a blocked start before it returns any path. Planning
    from an already clear handoff solves that ordering problem without editing
    a costmap or reducing the physical footprint.
    """
    from suat_mission.navigation.navigation_gate import goal_is_free
    heading=math.atan2(goal[1]-pose[1],goal[0]-pose[0])
    candidates=[];cache={}
    for delta in (0.,-.25,.25,-.5,.5,-.8,.8,-1.2,1.2,-1.57,1.57):
        angle=heading+delta
        for distance in (.20,.30,.40,.50,.60):
            point=[pose[0]+distance*math.cos(angle),pose[1]+distance*math.sin(angle),pose[2]]
            if not goal_is_free(grid,*point[:2],radius=radius+.03):continue
            try:proof=contact_path(grid,pose,[point[:2]],radius,_cache=cache)
            except ValueError:continue
            if not proof['escaped_initial_overlap']:continue
            turn=abs(math.atan2(math.sin(angle-pose[2]),math.cos(angle-pose[2])))
            cost=distance+math.dist(point[:2],goal[:2])+.08*turn
            candidates.append((cost,point,proof))
            break
    return sorted(candidates,key=lambda value:value[0])


def _contact_walls(scan, radius, laser_pose):
    # Fit only surfaces already beside the starting disc, with visible support.
    tx,ty,turn=laser_pose; groups={}
    for i,r in enumerate(scan.ranges):
        if not math.isfinite(r) or not scan.range_min<r<scan.range_max: continue
        a=scan.angle_min+i*scan.angle_increment+turn
        x,y=tx+r*math.cos(a),ty+r*math.sin(a)
        if math.hypot(x,y)>radius+.025: continue
        key=('x',x>0) if abs(x)>abs(y) else ('y',y>0)
        groups.setdefault(key,[]).append((x,y))
    walls=[]
    for points in groups.values():
        for _ in range(2):
            if len(points)<6: break
            mx=sum(x for x,y in points)/len(points); my=sum(y for x,y in points)/len(points)
            xx=sum((x-mx)**2 for x,y in points); yy=sum((y-my)**2 for x,y in points)
            xy=sum((x-mx)*(y-my) for x,y in points)
            angle=.5*math.atan2(2*xy,xx-yy); nx,ny=-math.sin(angle),math.cos(angle)
            offset=nx*mx+ny*my
            if offset<0: nx,ny,offset=-nx,-ny,-offset
            points=[(x,y) for x,y in points if abs(nx*x+ny*y-offset)<=CONTACT_WALL_BAND_M]
        if len(points)<6: continue
        tangent=[-ny*x+nx*y for x,y in points]
        if max(tangent)-min(tangent)<.08 or not .10<offset<=radius: continue
        walls.append({'normal':[nx,ny],'offset_m':offset,'support_beams':len(points)})
    return walls


def _project_walls(walls, reference_pose, current_pose):
    if reference_pose is None or current_pose is None:
        return [(w['normal'][0],w['normal'][1],w['offset_m'],w['offset_m']) for w in walls]
    x0,y0,a0=reference_pose; x1,y1,a1=current_pose
    c,s=math.cos(a1),math.sin(a1); tx=c*(x0-x1)+s*(y0-y1); ty=-s*(x0-x1)+c*(y0-y1)
    c,s=math.cos(a0-a1),math.sin(a0-a1); projected=[]
    for wall in walls:
        nx,ny=wall['normal']; px,py=c*nx-s*ny,s*nx+c*ny
        d0=wall['offset_m']; projected.append((px,py,d0+px*tx+py*ty,d0))
    return projected


def _existing_wall_point(x,y,distance,walls):
    return any(abs(nx*x+ny*y-d)<=CONTACT_WALL_BAND_M
               and min(d,d-nx*distance)>=initial-REAR_CLOSING_TOLERANCE_M
               for nx,ny,d,initial in walls)


def _beam_blocks(side, points):
    groups=[]
    for point in points:
        if (groups and point['beam']==groups[-1][-1]['beam']+1
                and math.dist(point['base_xy_m'],groups[-1][-1]['base_xy_m'])<=.04):
            groups[-1].append(point)
        else: groups.append([point])
    return [SweepBlocked(side,min(g,key=lambda p:p['clearance_m']),len(g)) for g in groups]


def scan_departure(scan, radius, distance, laser_xy_yaw=(0.,0.,0.), *, report_block=False,
                   initial_walls=None, reference_pose=None, current_pose=None):
    """Check new swept obstacles; retain, but never approach, initial wall contacts."""
    tx,ty,turn=laser_xy_yaw
    if not all(math.isfinite(v) for v in (tx,ty,turn,radius,distance)) or not 0<distance<=.60:
        raise ValueError('激光或起步参数无效')
    if not math.isfinite(scan.angle_increment) or scan.angle_increment==0:
        raise ValueError('激光角度无效')
    if initial_walls is None: initial_walls=_contact_walls(scan,radius,laser_xy_yaw)
    walls=_project_walls(initial_walls,reference_pose,current_pose)
    valid_angles=[]; rear_points=[]; rear_beams=[]; blocked=[]; count=0; contacts=0; nearest=float('inf')
    for i,r in enumerate(scan.ranges):
        if not math.isfinite(r) or not scan.range_min<r<scan.range_max: continue
        bearing=scan.angle_min+i*scan.angle_increment; a=bearing+turn
        a=math.atan2(math.sin(a),math.cos(a)); x,y=tx+r*math.cos(a),ty+r*math.sin(a)
        count+=1
        if -math.pi/2<=a<=math.pi/2: valid_angles.append(a)
        if _existing_wall_point(x,y,distance,walls):
            contacts+=1; continue
        separation=math.hypot(x,y) if x<=0 else swept_clearance(x,y,distance)
        point={'beam':i,'bearing_rad':bearing,'range_m':r,'base_xy_m':[x,y],'clearance_m':separation}
        if x<=0:
            if separation<radius: rear_points.append((x,y)); rear_beams.append(point)
        else:
            nearest=min(nearest,separation)
            if separation<=radius: blocked.append(point)
    forward_blocks=_beam_blocks('forward',blocked); forward_block=forward_blocks[0] if forward_blocks else None
    if forward_block is not None and not report_block: raise forward_block
    valid_angles.sort()
    if count<60 or not valid_angles: raise ValueError('激光有效数据不足')
    angles=[-math.pi/2]+valid_angles+[math.pi/2]
    if max(b-a for a,b in zip(angles,angles[1:]))>math.radians(12):
        raise ValueError('起步方向激光盲区过大')
    return {'nearest_forward_m':nearest if math.isfinite(nearest) else None,
            'rear_points':rear_points,'rear_beams':rear_beams,'valid_beams':count,
            'forward_block':forward_block,'forward_blocks':forward_blocks,
            'contact_walls':initial_walls,'existing_wall_beams':contacts}


def sweep_observation_gap(scan, bearings, radius, distance, laser_pose, *, initial_walls=None, reference_pose=None, current_pose=None):
    """Bound the unobserved width left between valid rays over the swept body."""
    tx, ty, laser_yaw = laser_pose
    sensor_offset = math.hypot(tx, ty)
    if sensor_offset >= radius:
        return None

    walls=_project_walls(initial_walls or [],reference_pose,current_pose)
    observed = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or not scan.range_min < r < scan.range_max:
            continue
        bearing = scan.angle_min + i * scan.angle_increment
        angle = bearing + laser_yaw
        x, y = tx + r * math.cos(angle), ty + r * math.sin(angle)
        if swept_clearance(x, y, distance) > radius or _existing_wall_point(x,y,distance,walls):
            observed.append(bearing)

    largest_gap = 0.
    for bearing in bearings:
        neighbors = [math.remainder(b - bearing, math.tau) for b in observed]
        if any(abs(d) <= 1e-6 for d in neighbors):
            continue
        right = max((d for d in neighbors if -MAX_RECHECK_NEIGHBOR_RAD <= d < 0.), default=None)
        left = min((d for d in neighbors if 0. < d <= MAX_RECHECK_NEIGHBOR_RAD), default=None)
        if right is None or left is None:
            return None

        max_range = distance + radius + sensor_offset
        right_angle, left_angle = bearing + laser_yaw + right, bearing + laser_yaw + left
        sy_right, sy_left = math.sin(right_angle), math.sin(left_angle)
        if sy_right * sy_left > 0.:
            # Within this narrow interval, the minimum |sin| is at an endpoint.
            # The swept body stays within |y| <= radius.
            max_range = min(max_range, (radius + abs(ty)) / min(abs(sy_right), abs(sy_left)))
        gap = 2. * max_range * math.sin((left - right) / 2.)
        if gap > MAX_RECHECK_GAP_M:
            return None
        largest_gap = max(largest_gap, gap)
    return largest_gap


def check_rear_clearance(initial_points, current_points, initial_pose, current_pose, radius,
                         *, rear_beams=None):
    """Only allow pre-existing rear contacts that have not become closer.

    Project the initial scan into the current base frame using odometry.
    Angular beam indices alone do not identify the same physical feature.
    """
    if not all(math.isfinite(v) for v in (*initial_pose, *current_pose, radius)):
        raise ValueError('起步扫描里程计位姿无效')
    x0, y0, yaw0 = initial_pose
    x1, y1, yaw1 = current_pose
    cos_yaw, sin_yaw = math.cos(yaw1), math.sin(yaw1)
    tx = cos_yaw*(x0-x1) + sin_yaw*(y0-y1)
    ty = -sin_yaw*(x0-x1) + cos_yaw*(y0-y1)
    cos_turn, sin_turn = math.cos(yaw0-yaw1), math.sin(yaw0-yaw1)
    projected = []
    for x, y in initial_points:
        initial_distance = math.hypot(x, y)
        if initial_distance < radius:
            expected_x = tx + cos_turn*x - sin_turn*y
            expected_y = ty + sin_turn*x + cos_turn*y
            minimum = max(initial_distance, math.hypot(expected_x, expected_y))
            projected.append((expected_x, expected_y, minimum))

    unmatched = []
    for i, (x, y) in enumerate(current_points):
        distance = math.hypot(x, y)
        if distance >= radius:
            continue
        matches = [minimum for px, py, minimum in projected
                   if math.hypot(x-px, y-py) <= REAR_ASSOCIATION_M]
        if not matches:
            unmatched.append(i)
            continue
        if distance < min(matches) - REAR_CLOSING_TOLERANCE_M:
            raise ValueError('起步时后侧障碍距离缩短')
    if unmatched:
        point = rear_beams[unmatched[0]] if rear_beams is not None else None
        blocks=_beam_blocks('rear',[rear_beams[i] for i in unmatched]) if rear_beams is not None else [SweepBlocked('rear',point,len(unmatched))]
        blocks[0].blocks=blocks
        raise blocks[0]
