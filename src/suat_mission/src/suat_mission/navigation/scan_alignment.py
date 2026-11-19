"""Bounded scan-to-map refinement for the saved A start. No ROS or motion I/O."""
import math
from statistics import median


def scan_endpoints(ranges, angle_min, angle_increment, range_min, range_max, laser_pose):
    x, y, yaw = laser_pose
    points = []
    for beam, distance in enumerate(ranges):
        if distance is None or not math.isfinite(distance) or not range_min < distance < min(range_max, 4.0):
            continue
        angle = angle_min + beam * angle_increment + yaw
        points.append((x + distance * math.cos(angle), y + distance * math.sin(angle)))
    return points[::max(1, math.ceil(len(points) / 240))]


def yaw_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ScanMap:
    """Distance to observed wall edges, without changing map occupancy."""

    def __init__(self, grid):
        self.x, self.y = grid['origin']
        self.resolution = grid['resolution']
        self.width, self.height = grid['width'], grid['height']
        self.data = grid['data']
        self.limit = max(.15, 3 * self.resolution)
        cells = math.ceil(self.limit / self.resolution)
        offsets = []
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                distance = math.hypot(dx, dy) * self.resolution
                if distance < self.limit:
                    offsets.append((dx, dy, distance))
        self.distances = {}
        for index, value in enumerate(self.data):
            if value != 100:
                continue
            row, column = divmod(index, self.width)
            if not any(0 <= i < self.width and 0 <= j < self.height and self.data[j * self.width + i] == 0
                       for i, j in ((column - 1, row), (column + 1, row), (column, row - 1), (column, row + 1))):
                continue
            for dx, dy, distance in offsets:
                cell = column + dx, row + dy
                if distance < self.distances.get(cell, self.limit):
                    self.distances[cell] = distance

    def free_center(self, pose):
        i = math.floor((pose[0] - self.x) / self.resolution)
        j = math.floor((pose[1] - self.y) / self.resolution)
        return 0 <= i < self.width and 0 <= j < self.height and self.data[j * self.width + i] == 0

    def residuals(self, points, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        gx = (x - self.x) / self.resolution - .5
        gy = (y - self.y) / self.resolution - .5
        c, s = c / self.resolution, s / self.resolution
        distances = self.distances.get
        limit = self.limit
        for px, py in points:
            u, v = gx + c * px - s * py, gy + s * px + c * py
            i, j = math.floor(u), math.floor(v)
            if not (-.5 <= u < self.width - .5 and -.5 <= v < self.height - .5):
                yield limit
                continue
            fx, fy = u - i, v - j
            low = (1 - fx) * distances((i, j), limit) + fx * distances((i + 1, j), limit)
            high = (1 - fx) * distances((i, j + 1), limit) + fx * distances((i + 1, j + 1), limit)
            yield (1 - fy) * low + fy * high

    def score(self, points, pose):
        if len(points) < 40:
            raise ValueError('not enough valid laser returns to align Start_A')
        residuals = list(self.residuals(points, pose))
        return {'beams': len(points), 'capped_mean_m': sum(residuals) / len(residuals),
                'median_m': median(residuals), 'within_0_10m': sum(d <= .10 for d in residuals) / len(residuals)}

    def refine(self, reference, pose, validation, validation_pose):
        """Fit one scan, then accept an improvement only on a separate scan."""
        before = [self.score(reference, pose), self.score(validation, validation_pose)]
        bounds = (.15, .15, math.radians(12))
        coarse = tuple(bound / 3 for bound in bounds)
        fine = tuple(step / 4 for step in coarse)

        def candidates(center, steps, count):
            for dx in range(-count, count + 1):
                for dy in range(-count, count + 1):
                    for da in range(-count, count + 1):
                        offset = tuple(v + i * step for v, i, step in zip(center, (dx, dy, da), steps))
                        if (math.hypot(*offset[:2]) > bounds[0] + 1e-9
                                or abs(offset[2]) > bounds[2] + 1e-9):
                            continue
                        candidate = tuple(v + d for v, d in zip(pose, offset))
                        if self.free_center(candidate):
                            cost = sum(self.residuals(reference, candidate))
                            yield cost, offset

        best = min(candidates((0., 0., 0.), coarse, 3), default=None)
        if best is None:
            return {'status': 'no_free_center', 'before': before}
        _, offset = min(candidates(best[1], fine, 3))
        candidate = tuple(v + d for v, d in zip(pose, offset))
        checked = tuple(v + d for v, d in zip(validation_pose, offset))
        after = [self.score(reference, candidate), self.score(validation, checked)]
        report = {'status': 'candidate', 'before': before, 'after': after,
                  'offset_xy_yaw': offset, 'candidate_pose': checked}
        if math.hypot(*offset[:2]) >= bounds[0] - fine[0] or abs(offset[2]) >= bounds[2] - fine[2]:
            report['status'] = 'search_boundary'
        elif not self.free_center(checked) or any(s['within_0_10m'] < .80 for s in after):
            report['status'] = 'insufficient_map_support'
        elif any(a['capped_mean_m'] - b['capped_mean_m'] < .05 * self.resolution
                 for a, b in zip(before, after)):
            report['status'] = 'no_repeatable_improvement'
        return report
