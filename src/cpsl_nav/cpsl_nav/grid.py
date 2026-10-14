"""
Occupancy-grid helpers for frontier_explore and wander: numpy, no ROS.

A grid is a ``nav_msgs/OccupancyGrid`` as an array: ``data[row, col]`` with
row = y and col = x, -1 unknown, 0..100 occupancy. Cell (r, c) covers
``origin + (c, r) * resolution`` to one cell further; the map's origin yaw is
assumed 0 (slam_toolbox and map_server both publish it so).

Everything here is deterministic given its inputs (sampling takes the
caller's ``numpy.random.Generator``), so it is unit-tested on synthetic grids.
"""

from collections import namedtuple
import math

import numpy as np

UNKNOWN = -1
OCCUPIED_THRESHOLD = 50     # a cell >= this is an obstacle; 0 <= value < it is free

Frontier = namedtuple('Frontier', 'size centroid goal cells')
Frontier.__doc__ = """One frontier: `size` cells, `centroid` and `goal` (x, y) in the map
frame (the goal is the cluster's cell nearest the centroid, among those with
clearance if any), `cells` an (N, 2) array of (row, col)."""


class Grid:
    """An occupancy grid with its geometry."""

    def __init__(self, data, resolution, origin_x=0.0, origin_y=0.0):
        self.data = np.asarray(data, dtype=np.int16)
        if self.data.ndim != 2:
            raise ValueError('grid data must be 2-D (rows = y, cols = x)')
        self.resolution = float(resolution)
        self.origin = (float(origin_x), float(origin_y))

    @classmethod
    def from_msg(cls, msg):
        """Build from a nav_msgs/OccupancyGrid (origin yaw ignored)."""
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return cls(data, info.resolution, info.origin.position.x, info.origin.position.y)

    @property
    def shape(self):
        return self.data.shape

    def to_cell(self, x, y):
        """(row, col) of the cell holding (x, y); may lie outside the grid."""
        return (int(math.floor((y - self.origin[1]) / self.resolution)),
                int(math.floor((x - self.origin[0]) / self.resolution)))

    def to_world(self, row, col):
        """(x, y) of a cell's centre."""
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def inside(self, row, col):
        return 0 <= row < self.data.shape[0] and 0 <= col < self.data.shape[1]

    def value_at(self, x, y, default=UNKNOWN):
        r, c = self.to_cell(x, y)
        return int(self.data[r, c]) if self.inside(r, c) else default

    def known_area(self):
        """Area of known (free or occupied) cells, m^2."""
        return float(np.count_nonzero(self.data >= 0)) * self.resolution ** 2


def free_mask(data, threshold=OCCUPIED_THRESHOLD):
    data = np.asarray(data)
    return (data >= 0) & (data < threshold)


def occupied_mask(data, threshold=OCCUPIED_THRESHOLD):
    return np.asarray(data) >= threshold


def unknown_mask(data):
    return np.asarray(data) < 0


def _shifted(mask, dr, dc):
    """`mask` moved by (dr, dc), False where nothing moved in."""
    out = np.zeros_like(mask)
    h, w = mask.shape
    src = mask[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
    out[max(0, dr):max(0, dr) + src.shape[0], max(0, dc):max(0, dc) + src.shape[1]] = src
    return out


def _offsets(radius, eight=True):
    if radius == 1:
        return [(-1, 0), (1, 0), (0, -1), (0, 1)] + (
            [(-1, -1), (-1, 1), (1, -1), (1, 1)] if eight else [])
    return [(dr, dc) for dr in range(-radius, radius + 1) for dc in range(-radius, radius + 1)
            if (dr or dc) and dr * dr + dc * dc <= radius * radius]


def dilate(mask, radius_cells):
    """Grow `mask` by a disc of `radius_cells` (0 = unchanged)."""
    mask = np.asarray(mask, dtype=bool)
    radius = int(math.ceil(radius_cells))
    if radius <= 0:
        return mask.copy()
    out = mask.copy()
    for dr, dc in _offsets(radius):
        out |= _shifted(mask, dr, dc)
    return out


def frontier_mask(data, threshold=OCCUPIED_THRESHOLD):
    """Free cells with an unknown 4-neighbour."""
    unknown = unknown_mask(data)
    near_unknown = np.zeros_like(unknown)
    for dr, dc in _offsets(1, eight=False):
        near_unknown |= _shifted(unknown, dr, dc)
    return free_mask(data, threshold) & near_unknown


def clusters(mask):
    """8-connected components of `mask`, as (N, 2) arrays of (row, col), row-major order."""
    mask = np.asarray(mask, dtype=bool)
    seen = np.zeros_like(mask)
    h, w = mask.shape
    out = []
    for r0, c0 in np.argwhere(mask):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        stack, cells = [(r0, c0)], []
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr, dc in _offsets(1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    stack.append((rr, cc))
        out.append(np.array(sorted(cells), dtype=int))
    return out


def frontiers(grid, min_size=5, clearance=0.0, threshold=OCCUPIED_THRESHOLD):
    """
    Frontier clusters of `grid` with at least `min_size` cells, largest first.

    A goal cell is preferred `clearance` metres from any obstacle; if no cell
    of the cluster has that, the cell nearest the centroid is used anyway (the
    planner's tolerance and the goal checker do the rest).
    """
    mask = frontier_mask(grid.data, threshold)
    near_obstacle = dilate(occupied_mask(grid.data, threshold), clearance / grid.resolution)
    out = []
    for cells in clusters(mask):
        if len(cells) < min_size:
            continue
        rows, cols = cells[:, 0], cells[:, 1]
        cx = grid.origin[0] + (cols.mean() + 0.5) * grid.resolution
        cy = grid.origin[1] + (rows.mean() + 0.5) * grid.resolution
        pool = cells[~near_obstacle[rows, cols]]
        if not len(pool):
            pool = cells
        d2 = (pool[:, 0] - (rows.mean())) ** 2 + (pool[:, 1] - (cols.mean())) ** 2
        r, c = pool[int(np.argmin(d2))]
        out.append(Frontier(len(cells), (cx, cy), grid.to_world(r, c), cells))
    out.sort(key=lambda f: -f.size)
    return out


def near_any(point, points, radius):
    """Tell whether `point` is within `radius` of any of `points`."""
    return any(math.hypot(point[0] - p[0], point[1] - p[1]) <= radius for p in points)


def choose(found, robot_xy, resolution, blacklist=(), blacklist_radius=0.5,
           min_distance=0.0, blocked=None, size_weight=0.5):
    """
    Pick the frontier to go to, or None.

    Cost = distance from the robot to the goal - `size_weight` * frontier
    length in metres (size * resolution): near and large wins. Frontiers whose
    goal is within `blacklist_radius` of a blacklisted point, nearer than
    `min_distance`, or for which ``blocked(x, y)`` is true (e.g. in a keepout
    zone) are skipped.
    """
    best, best_cost = None, math.inf
    for f in found:
        gx, gy = f.goal
        dist = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        if dist < min_distance or near_any(f.goal, blacklist, blacklist_radius):
            continue
        if blocked is not None and blocked(gx, gy):
            continue
        cost = dist - size_weight * f.size * resolution
        if cost < best_cost:
            best, best_cost = f, cost
    return best


def frontier_near(grid, xy, radius, threshold=OCCUPIED_THRESHOLD):
    """Tell whether any frontier cell lies within `radius` metres of `xy`."""
    r0, c0 = grid.to_cell(*xy)
    n = int(math.ceil(radius / grid.resolution))
    h, w = grid.shape
    r1, r2, c1, c2 = max(0, r0 - n), min(h, r0 + n + 1), max(0, c0 - n), min(w, c0 + n + 1)
    if r1 >= r2 or c1 >= c2:
        return False
    # pad by one so the window's own edge cells see their outside neighbours
    pr1, pr2, pc1, pc2 = max(0, r1 - 1), min(h, r2 + 1), max(0, c1 - 1), min(w, c2 + 1)
    window = frontier_mask(grid.data[pr1:pr2, pc1:pc2], threshold)
    return bool(window[r1 - pr1:r2 - pr1, c1 - pc1:c2 - pc1].any())


def blocked_by_mask(mask_grid, x, y, margin=0.0):
    """
    Tell whether a keepout mask marks (x, y), or any cell within `margin` of it.

    `mask_grid` is the keepout filter's mask (a Grid; > 0 = keepout, as
    filter_mask_server publishes an occupied cell), or None for no mask.
    """
    if mask_grid is None:
        return False
    r0, c0 = mask_grid.to_cell(x, y)
    n = int(math.ceil(margin / mask_grid.resolution))
    h, w = mask_grid.shape
    r1, r2, c1, c2 = max(0, r0 - n), min(h, r0 + n + 1), max(0, c0 - n), min(w, c0 + n + 1)
    if r1 >= r2 or c1 >= c2:
        return False
    return bool((mask_grid.data[r1:r2, c1:c2] > 0).any())


def free_goal_mask(grid, clearance=0.3, threshold=OCCUPIED_THRESHOLD):
    """Free cells at least `clearance` metres from any obstacle or unknown cell."""
    bad = occupied_mask(grid.data, threshold) | unknown_mask(grid.data)
    return free_mask(grid.data, threshold) & ~dilate(bad, clearance / grid.resolution)


def sample_goals(grid, robot_xy, rng, count=10, min_distance=1.0, max_distance=math.inf,
                 clearance=0.3, blocked=None, threshold=OCCUPIED_THRESHOLD):
    """
    Up to `count` random free goals (x, y), `min_distance`..`max_distance` from the robot.

    Goals are cell centres with `clearance` to obstacles and unknown space,
    drawn without replacement by `rng` (a numpy Generator); ``blocked(x, y)``
    true skips a goal (e.g. a keepout zone).
    """
    cells = np.argwhere(free_goal_mask(grid, clearance, threshold))
    if not len(cells):
        return []
    xs = grid.origin[0] + (cells[:, 1] + 0.5) * grid.resolution
    ys = grid.origin[1] + (cells[:, 0] + 0.5) * grid.resolution
    d = np.hypot(xs - robot_xy[0], ys - robot_xy[1])
    ok = np.flatnonzero((d >= min_distance) & (d <= max_distance))
    out = []
    for i in rng.permutation(ok):
        x, y = float(xs[i]), float(ys[i])
        if blocked is not None and blocked(x, y):
            continue
        out.append((x, y))
        if len(out) >= count:
            break
    return out
