"""
planner.py - global path planning with A* on the inflated cost-map.

A* = Dijkstra + a heuristic that "points" toward the goal, so it explores far
fewer cells.  We use 8-connected moves, octile distance as the heuristic and
prevent diagonal corner-cutting.  A line-of-sight smoother then removes the
zig-zag staircase typical of grid paths.
"""
import heapq
import math
import numpy as np
from . import config as C

SQ2 = math.sqrt(2.0)
_N8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]


def world_to_cell(x, y):
    ix = min(max(int(x // C.RES), 0), C.GRID_W - 1)
    iy = min(max(int(y // C.RES), 0), C.GRID_H - 1)
    return ix, iy


def cell_to_world(ix, iy):
    return (ix + 0.5) * C.RES, (iy + 0.5) * C.RES


def _octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (SQ2 - 2.0) * min(dx, dy)


def _line_free(grid, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 2
    xs = np.linspace(a[0], b[0], n).round().astype(int)
    ys = np.linspace(a[1], b[1], n).round().astype(int)
    return not grid[ys, xs].any()


def _smooth(cells, grid):
    if len(cells) < 3:
        return cells
    out, i = [cells[0]], 0
    while i < len(cells) - 1:
        j = i + 1
        while j + 1 < len(cells) and _line_free(grid, cells[i], cells[j + 1]):
            j += 1
        out.append(cells[j])
        i = j
    return out


def _start_zone(raw, s, r=3):
    z = np.zeros_like(raw, dtype=bool)
    z[max(s[1] - r, 0):s[1] + r + 1, max(s[0] - r, 0):s[0] + r + 1] = True
    return z


def astar(blocked, raw, start_xy, goal_xy, cost=None, smooth_grid=None, smooth=True):
    """
    blocked : inflated obstacle grid (True = forbidden)
    raw     : un-inflated obstacle grid (used to free the cells around the robot)
    cost    : optional per-cell cost (>=1) - makes the path keep away from obstacles
    smooth_grid : optional stricter grid used only for path shortcutting
    Returns a list of (x, y) world points, or None if no path exists.
    """
    s = world_to_cell(*start_xy)
    g = world_to_cell(*goal_xy)
    grid = blocked.copy()
    # The robot may already stand inside the safety margin of an obstacle:
    # keep real obstacles blocked but free the margin around the start.
    r = 3
    y0, y1 = max(s[1] - r, 0), min(s[1] + r + 1, C.GRID_H)
    x0, x1 = max(s[0] - r, 0), min(s[0] + r + 1, C.GRID_W)
    grid[y0:y1, x0:x1] = raw[y0:y1, x0:x1]
    grid[s[1], s[0]] = False
    if grid[g[1], g[0]]:
        return None

    open_heap = [(_octile(s, g), 0.0, s)]
    best = {s: 0.0}
    came = {}
    closed = set()
    while open_heap:
        _, gc, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == g:
            cells = [cur]
            while cur in came:
                cur = came[cur]
                cells.append(cur)
            cells.reverse()
            if smooth:
                sg = grid if smooth_grid is None else (smooth_grid | blocked) & ~_start_zone(raw, s)
                cells = _smooth(cells, sg)
            pts = [cell_to_world(*c) for c in cells]
            pts[0] = (float(start_xy[0]), float(start_xy[1]))
            pts[-1] = (float(goal_xy[0]), float(goal_xy[1]))
            return pts
        cx, cy = cur
        for dx, dy, step in _N8:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < C.GRID_W and 0 <= ny < C.GRID_H) or grid[ny, nx]:
                continue
            if dx and dy and (grid[cy, nx] or grid[ny, cx]):     # no corner cutting
                continue
            ng = gc + step * (1.0 if cost is None else float(cost[ny, nx]))
            if ng < best.get((nx, ny), 1e18):
                best[(nx, ny)] = ng
                came[(nx, ny)] = cur
                heapq.heappush(open_heap, (ng + _octile((nx, ny), g), ng, (nx, ny)))
    return None
