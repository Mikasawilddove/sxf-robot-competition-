"""Map-bound routes and whole-body offline A* validation, independent of ROS."""
import heapq
import math
from suat_mission.navigation.navigation_gate import goal_is_free
from suat_mission.navigation.map_identity import map_identity, validate_map_identity


def validate_route(route, grid, radius=.25):
    if route.get('schema') != 1 or route.get('frame_id') != 'map':
        raise ValueError('route requires schema 1 and map frame')
    validate_map_identity(route, grid)
    points = route.get('waypoints')
    if not isinstance(points, list) or not 1 <= len(points) <= 32:
        raise ValueError('route requires 1..32 waypoints')
    for item in points:
        pose = item.get('pose') if isinstance(item, dict) else None
        if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not all(type(v) in (int,float) and math.isfinite(v) for v in pose):
            raise ValueError('waypoint requires finite [x,y,yaw]')
        if not goal_is_free(grid, pose[0], pose[1], radius=radius):
            raise ValueError('waypoint footprint blocked: '+str(item.get('name')))
    return points


class StaticPlanner:
    def __init__(self, grid, radius=.25):
        self.grid, self.radius, self.cache = grid, radius, {}

    def xy(self, cell):
        g = self.grid
        return (g['origin'][0]+(cell[0]+.5)*g['resolution'], g['origin'][1]+(cell[1]+.5)*g['resolution'])

    def cell(self, point):
        g = self.grid
        return (math.floor((point[0]-g['origin'][0])/g['resolution']), math.floor((point[1]-g['origin'][1])/g['resolution']))

    def free(self, cell):
        if cell not in self.cache:
            x,y=self.xy(cell)
            # Extra half-diagonal covers travel between adjacent grid centers.
            self.cache[cell]=goal_is_free(self.grid,x,y,self.radius+self.grid['resolution']/math.sqrt(2))
        return self.cache[cell]

    def segment_free(self, a, b):
        count=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/(self.grid['resolution']/2)))
        return all(goal_is_free(self.grid,a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count,self.radius) for i in range(count+1))

    def plan(self, start, goal):
        if not goal_is_free(self.grid,*start[:2],self.radius) or not goal_is_free(self.grid,*goal[:2],self.radius):
            raise ValueError('start or goal body footprint is blocked')
        first,last=self.cell(start),self.cell(goal)
        if not self.free(first) or not self.free(last):
            raise ValueError('insufficient clearance around endpoint grid cell')
        queue=[(0.,0.,first)]; distance={first:0.}; parent={}
        while queue:
            _,cost,cell=heapq.heappop(queue)
            if cost != distance.get(cell):continue
            if cell==last:break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(cell[0]+dx,cell[1]+dy)
                if not self.free(nxt):continue
                if dx and dy and (not self.free((cell[0]+dx,cell[1])) or not self.free((cell[0],cell[1]+dy))):continue
                new=cost+math.hypot(dx,dy)
                if new < distance.get(nxt,float('inf')):
                    distance[nxt]=new;parent[nxt]=cell
                    heapq.heappush(queue,(new+math.hypot(nxt[0]-last[0],nxt[1]-last[1]),new,nxt))
        else:raise ValueError('no whole-body path on saved map')
        cells=[last]
        while cells[-1]!=first:cells.append(parent[cells[-1]])
        path=[tuple(start[:2])]+[self.xy(c) for c in reversed(cells)]+[tuple(goal[:2])]
        if not all(self.segment_free(a,b) for a,b in zip(path,path[1:])):
            raise ValueError('path connects through occupied or unknown space')
        return path


def loop_candidate(grid, anchor, near=.45, far=2.15):
    """A->D->C->B->A interior loop; anchor heading is the measured A->D axis."""
    if not all(math.isfinite(v) for v in (*anchor,near,far)) or not .3 <= near < far <= 2.5:
        raise ValueError('invalid loop geometry')
    x,y,yaw=anchor
    points=[]
    for name,f,l,turn in [('A_inside',near,near,0),('D_inside',far,near,math.pi/2),
                           ('C_inside',far,far,math.pi),('B_inside',near,far,-math.pi/2),('A_return',near,near,0)]:
        points.append({'name':name,'pose':[x+f*math.cos(yaw)-l*math.sin(yaw),y+f*math.sin(yaw)+l*math.cos(yaw),
                                         math.atan2(math.sin(yaw+turn),math.cos(yaw+turn))]})
    route={'schema':1,'frame_id':'map','map_image_sha256':grid['image_sha256'], 'map_grid_sha256':grid['grid_sha256'],
           'map_identity':map_identity(grid),
           'status':'CANDIDATE_NOT_FIELD_VALIDATED','purpose':'base_only_interior_loop','waypoints':points,
           'anchor_pose':list(anchor),'near_m':near,'far_m':far}
    validate_route(route,grid)
    planner=StaticPlanner(grid)
    paths=[planner.plan(a['pose'],b['pose']) for a,b in zip(points,points[1:])]
    route['static_paths']=paths
    route['static_length_m']=sum(math.dist(a,b) for path in paths for a,b in zip(path,path[1:]))
    return route
