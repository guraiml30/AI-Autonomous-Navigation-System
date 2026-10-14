"""
world.py - the virtual environment (ground truth).

The robot is NOT allowed to read this directly.  It only sees the world through
the simulated LiDAR in sensors.py - exactly like a real robot.
"""
from dataclasses import dataclass
import numpy as np
from . import config as C


@dataclass
class Rect:
    x: float
    y: float
    w: float
    h: float


class DynamicObstacle:
    """A circular obstacle that patrols back and forth between two points."""

    def __init__(self, p0, p1, speed, radius):
        self.p0 = np.array(p0, float)
        self.p1 = np.array(p1, float)
        self.speed = speed
        self.radius = radius
        self.length = float(np.linalg.norm(self.p1 - self.p0))
        self.t = 0.0

    @property
    def pos(self):
        s = self.t % 2.0
        if s > 1.0:
            s = 2.0 - s
        return self.p0 + s * (self.p1 - self.p0)

    def step(self, dt):
        self.t += self.speed * dt / self.length


class World:
    def __init__(self, name, rects, dynamics, start, goal):
        self.name = name
        self.rects = rects
        self.dynamics = dynamics
        self.start = start          # (x, y, heading)
        self.goal = goal            # (x, y)
        self.static_grid = self._rasterize()

    # ---- building the static occupancy grid -------------------------------
    def _rasterize(self):
        g = np.zeros((C.GRID_H, C.GRID_W), dtype=bool)
        for r in self.rects:
            i0, i1 = int(round(r.x / C.RES)), int(round((r.x + r.w) / C.RES))
            j0, j1 = int(round(r.y / C.RES)), int(round((r.y + r.h) / C.RES))
            g[j0:j1, i0:i1] = True
        return g

    # ---- queries -----------------------------------------------------------
    def static_blocked(self, pts):
        ix = np.floor(pts[..., 0] / C.RES).astype(int)
        iy = np.floor(pts[..., 1] / C.RES).astype(int)
        out = (ix < 0) | (ix >= C.GRID_W) | (iy < 0) | (iy >= C.GRID_H)
        ixc = np.clip(ix, 0, C.GRID_W - 1)
        iyc = np.clip(iy, 0, C.GRID_H - 1)
        return out | self.static_grid[iyc, ixc]

    def dynamic_blocked(self, pts):
        res = np.zeros(pts.shape[:-1], dtype=bool)
        for d in self.dynamics:
            p = d.pos
            res |= np.hypot(pts[..., 0] - p[0], pts[..., 1] - p[1]) <= d.radius
        return res

    def is_blocked(self, pts):
        return self.static_blocked(pts) | self.dynamic_blocked(pts)

    def raycast(self, x, y, angles, max_range, step=0.1):
        """Vectorised ray marching. Returns (ranges, hit_flags)."""
        d = np.arange(0.2, max_range + 1e-9, step)
        dirs = np.stack([np.cos(angles), np.sin(angles)], axis=1)         # (R,2)
        pts = np.array([x, y]) + d[None, :, None] * dirs[:, None, :]      # (R,S,2)
        hit = self.is_blocked(pts)
        any_hit = hit.any(axis=1)
        first = hit.argmax(axis=1)
        ranges = np.where(any_hit, d[first], max_range)
        return ranges, any_hit

    def collision(self, x, y):
        """True if a robot disc centred at (x, y) overlaps anything."""
        a = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        ring1 = np.stack([x + C.ROBOT_RADIUS * np.cos(a), y + C.ROBOT_RADIUS * np.sin(a)], 1)
        ring2 = np.stack([x + 0.5 * C.ROBOT_RADIUS * np.cos(a), y + 0.5 * C.ROBOT_RADIUS * np.sin(a)], 1)
        pts = np.vstack([ring1, ring2, [[x, y]]])
        return bool(self.is_blocked(pts).any())

    def step(self, dt):
        for d in self.dynamics:
            d.step(dt)


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------
SCENARIOS = {
    "easy": "Open field with a few blocks - the 'hello world' run",
    "warehouse": "Serpentine warehouse aisles with crates - industrial AGV style",
    "dynamic": "Static blocks + 3 moving obstacles patrolling across the route",
}


def make_world(name: str) -> World:
    if name == "easy":
        rects = [Rect(14, 8, 5, 14), Rect(28, 22, 6, 12), Rect(40, 6, 5, 14),
                 Rect(18, 28, 8, 3), Rect(48, 26, 4, 8)]
        return World(name, rects, [], (4.0, 4.0, 0.6), (56.0, 36.0))

    if name == "warehouse":
        shelves = [Rect(12, 0, 2, 28), Rect(22, 12, 2, 28), Rect(32, 0, 2, 28), Rect(42, 12, 2, 28)]
        crates = [Rect(17.5, 18, 3, 3), Rect(27, 18, 3, 3), Rect(37, 22, 3, 3),
                  Rect(16, 31, 2, 2), Rect(47, 6, 3, 3)]
        return World(name, shelves + crates, [], (4.0, 20.0, 0.0), (56.0, 20.0))

    if name == "dynamic":
        rects = [Rect(10, 14, 8, 6), Rect(26, 16, 4, 12), Rect(40, 22, 8, 4)]
        dyn = [DynamicObstacle((22, 4), (22, 36), 1.1, 0.8),
               DynamicObstacle((34, 36), (34, 6), 1.0, 0.8),
               DynamicObstacle((48, 8), (48, 32), 0.9, 0.8)]
        return World(name, rects, dyn, (4.0, 4.0, 0.6), (56.0, 36.0))

    raise ValueError(f"Unknown scenario '{name}'. Choose from {list(SCENARIOS)}")
