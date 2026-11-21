"""Map-bound exclusion for the block array, independent of laser returns."""
import json
import math
from pathlib import Path

from suat_mission.navigation.map_identity import validate_map_identity


PLANNING_MAP_TOPIC = '/suat/navigation/planning_map'
KEEP_OUT_FILE = 'data/navigation/central_pickup_keepout.json'


def is_pickup_station(keepout, name):
    return (keepout is not None and keepout.precise_pickup
            and name in ('Sorting_W','Sorting_S','Sorting_E','Sorting_N'))


class PickupKeepout:
    def __init__(self, record, binding):
        validate_map_identity(record, binding)
        xy = record['center_map_m']
        angle, size, padding = (record[k] for k in ('yaw_map_rad', 'side_m', 'boundary_padding_m'))
        if (record.get('schema') != 1 or len(xy) != 2
                or not all(type(v) in (int, float) and math.isfinite(v) for v in (*xy, angle, size, padding))
                or size <= 0 or padding < 0):
            raise ValueError('invalid central pickup keepout geometry')
        self.center = tuple(xy)
        self.c, self.s = math.cos(angle), math.sin(angle)
        self.half = size / 2 + padding
        self.precise_pickup = record.get('rasterize_brick_array_only') is True
        self.raster_half = size / 2 if self.precise_pickup else self.half

    @classmethod
    def load(cls, root, binding, filename=KEEP_OUT_FILE):
        return cls(json.loads((Path(root)/filename).read_text(encoding='utf-8')), binding)

    def relative(self, point):
        x, y = point[0]-self.center[0], point[1]-self.center[1]
        return self.c*x+self.s*y, self.c*y-self.s*x

    def distance(self, point):
        x, y = self.relative(point)
        return math.hypot(max(abs(x)-self.half, 0.), max(abs(y)-self.half, 0.))

    def segment_clear(self, start, end, radius):
        if min(self.distance(start), self.distance(end)) <= radius:
            return False
        ax, ay = self.relative(start)
        bx, by = self.relative(end)
        dx, dy = bx-ax, by-ay
        squared = dx*dx+dy*dy
        if squared == 0:
            return True
        h = self.half
        # A segment with outside endpoints either crosses an edge, or its
        # minimum distance to the rectangle occurs at an endpoint or a corner.
        for fixed, other, dv, du in ((ax, ay, dx, dy), (ay, ax, dy, dx)):
            if dv:
                for edge in (-h, h):
                    t = (edge-fixed)/dv
                    if 0 <= t <= 1 and abs(other+t*du) <= h:
                        return False
        for x in (-h, h):
            for y in (-h, h):
                t = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/squared))
                if math.hypot(x-ax-t*dx, y-ay-t*dy) <= radius:
                    return False
        return True

    def mask(self, grid):
        ox, oy = grid['origin']
        r, width = grid['resolution'], grid['width']
        values = list(grid['data'])
        extent = self.raster_half*(abs(self.c)+abs(self.s))
        x0, x1 = (math.floor((self.center[0]+s*extent-ox)/r) for s in (-1.,1.))
        y0, y1 = (math.floor((self.center[1]+s*extent-oy)/r) for s in (-1.,1.))
        for j in range(max(0,y0),min(grid['height']-1,y1)+1):
            for i in range(max(0,x0),min(width-1,x1)+1):
                x,y = self.relative((ox+(i+.5)*r,oy+(j+.5)*r))
                if max(abs(x),abs(y)) <= self.raster_half:
                    values[j*width+i] = 100
        return dict(grid,data=values)
