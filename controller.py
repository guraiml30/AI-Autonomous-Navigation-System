"""
controller.py - navigation logic: follow the path, but never hit anything.

Two layers (the same idea real robots use):
  * Deliberative layer : pure-pursuit path following on the A* path
  * Reactive safety layer: reads the LiDAR every tick and can slow down,
    steer away, or emergency-stop / back off, overriding the path follower.
"""
import math
import numpy as np
from . import config as C

SPACING = 0.25   # path densification (m)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Controller:
    def __init__(self):
        self.path = None
        self.nearest = 0
        self.prev_min = None     # previous nearest-obstacle range (to estimate closing speed)

    # ---- path handling -----------------------------------------------------
    def set_path(self, pts):
        pts = np.asarray(pts, float)
        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        s = np.concatenate([[0.0], np.cumsum(seg)])
        n = max(2, int(s[-1] / SPACING) + 1)
        ss = np.linspace(0, s[-1], n)
        self.path = np.stack([np.interp(ss, s, pts[:, 0]), np.interp(ss, s, pts[:, 1])], axis=1)
        self.nearest = 0

    def clear(self):
        self.path = None
        self.nearest = 0

    def remaining(self):
        return np.empty((0, 2)) if self.path is None else self.path[self.nearest:]

    def _advance(self, pos):
        lo, hi = self.nearest, min(len(self.path), self.nearest + 40)
        d = np.hypot(self.path[lo:hi, 0] - pos[0], self.path[lo:hi, 1] - pos[1])
        self.nearest = lo + int(np.argmin(d))

    def lookahead_point(self):
        idx = min(len(self.path) - 1, self.nearest + int(C.LOOKAHEAD / SPACING))
        return self.path[idx]

    # ---- LiDAR helpers -----------------------------------------------------
    @staticmethod
    def _sector_min(scan, lo_deg, hi_deg):
        rel = scan.angles_rel
        m = (rel >= math.radians(lo_deg)) & (rel <= math.radians(hi_deg))
        return float(scan.ranges[m].min()) if m.any() else C.LIDAR_RANGE

    @staticmethod
    def _front_distance(scan):
        """Distance to the nearest thing inside the corridor the robot will sweep."""
        rel, r = scan.angles_rel, scan.ranges
        fwd, lat = r * np.cos(rel), np.abs(r * np.sin(rel))
        m = (fwd > 0) & (lat < C.CORRIDOR_HALF_WIDTH) & scan.hit
        return float(fwd[m].min()) if m.any() else C.LIDAR_RANGE

    @staticmethod
    def _rear_distance(scan):
        rel, r = scan.angles_rel, scan.ranges
        back, lat = -r * np.cos(rel), np.abs(r * np.sin(rel))
        m = (back > 0) & (lat < C.CORRIDOR_HALF_WIDTH) & scan.hit
        return float(back[m].min()) if m.any() else C.LIDAR_RANGE

    # ---- main entry ----------------------------------------------------------
    def command(self, pose, goal, scan):
        """Returns (v_cmd, w_cmd, state_name)."""
        x, y, th = pose
        if self.path is None:                                   # nothing to follow: wait safely
            v, w, state = 0.0, 0.0, "NO_PATH"
        else:
            self._advance((x, y))
            tx, ty = self.lookahead_point()
            err = wrap(math.atan2(ty - y, tx - x) - th)
            dist_goal = math.hypot(goal[0] - x, goal[1] - y)

            # -- deliberative: pure pursuit -------------------------------
            v = 0.0 if abs(err) > math.pi / 2 else C.MAX_V * max(0.15, 1.0 - abs(err) / (math.pi / 2))
            v = min(v, max(0.4, 0.8 * dist_goal))               # slow down near goal
            w = max(-C.MAX_W, min(C.MAX_W, 2.5 * err))
            state = "NAVIGATING"

        # -- reactive safety layer ----------------------------------------
        front = self._front_distance(scan)
        rear = self._rear_distance(scan)
        left = self._sector_min(scan, 15, 90)
        right = self._sector_min(scan, -90, -15)

        # proximity speed limit: the closer anything is (any direction ahead), the slower we go.
        # This is what lets the robot react to obstacles that move *into* its path.
        fm = np.abs(scan.angles_rel) < math.radians(100)
        clearance = float(scan.ranges[fm].min()) - C.ROBOT_RADIUS
        v_limit = max(0.3, 2.0 * clearance)
        if v > v_limit:
            v = v_limit
            state = "CAUTION"

        # nearest thing in ANY direction (obstacles can also approach from the side/back)
        k = int(np.argmin(scan.ranges))
        nearest_rel, nearest_r = float(scan.angles_rel[k]), float(scan.ranges[k])
        closing = 0.0 if self.prev_min is None else (self.prev_min - nearest_r) / C.DT
        self.prev_min = nearest_r
        too_close = nearest_r < C.ROBOT_RADIUS + 0.2
        closing_in = nearest_r < C.ROBOT_RADIUS + 0.45 and closing > 0.6
        danger = front < C.STOP_DIST or (bool(scan.hit[k]) and (too_close or closing_in))

        if danger:
            state = "EMERGENCY_STOP"
            if abs(nearest_rel) < math.pi / 2:                 # threat ahead -> back straight off
                v, w = (-1.0, 0.0) if rear > 1.5 else (0.0, 1.0 if left >= right else -1.0)
            else:                                              # threat behind/side -> drive away
                v, w = (1.0, 0.0) if front > 2.0 else (0.0, 1.0 if left >= right else -1.0)
        elif front < C.CAUTION_DIST:
            scale = (front - C.STOP_DIST) / (C.CAUTION_DIST - C.STOP_DIST)
            v *= max(0.2, scale)
            steer = max(-1.0, min(1.0, (left - right) / C.CAUTION_DIST))
            if abs(steer) > 0.15:
                w += steer
                state = "AVOIDING"
            else:
                state = "CAUTION"
        return v, max(-C.MAX_W, min(C.MAX_W, w)), state
