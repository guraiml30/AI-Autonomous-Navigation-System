"""
perception.py - turns raw LiDAR numbers into understanding.

1. Occupancy mapping : log-odds grid, updated every scan (what is free / blocked)
2. Obstacle detection: clusters neighbouring LiDAR hits into discrete obstacles
3. Cost-map building : OpenCV dilation inflates obstacles by the robot size so
                       the planner can treat the robot as a single point
"""
import math

try:
    import cv2  # type: ignore[import-not-found]
except ModuleNotFoundError:
    cv2 = None

import numpy as np
from . import config as C

L_OCC, L_FREE = 1.5, -0.5
L_MIN, L_MAX = -2.0, 3.0
OCC_THRESHOLD = 0.5


def _require_cv2():
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for perception processing. Install opencv-python.")


class Perception:
    def __init__(self):
        self.logodds = np.zeros((C.GRID_H, C.GRID_W), dtype=np.float32)
        self.t = 0.0                 # internal clock (s)
        self.tracks = []             # moving-obstacle tracks {pos, vel, last_t, hits, miss}
        self.dyn_blocked = np.zeros((C.GRID_H, C.GRID_W), dtype=bool)

    # ---- occupancy mapping -------------------------------------------------
    def _add(self, xs, ys, value):
        ix = np.floor(xs / C.RES).astype(int)
        iy = np.floor(ys / C.RES).astype(int)
        ok = (ix >= 0) & (ix < C.GRID_W) & (iy >= 0) & (iy < C.GRID_H)
        flat = np.unique(iy[ok] * C.GRID_W + ix[ok])
        lo = self.logodds.reshape(-1)
        lo[flat] += value

    def update(self, pose, scan):
        self.t += C.DT
        prev = self.logodds.copy()
        x, y, th = pose
        ang = th + scan.angles_rel
        dirs = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        steps = np.arange(0.0, C.LIDAR_RANGE, C.RES * 0.8)
        pts = np.array([x, y]) + steps[None, :, None] * dirs[:, None, :]
        free = steps[None, :] < (scan.ranges[:, None] - C.RES)
        self._add(pts[..., 0][free], pts[..., 1][free], L_FREE)
        h = scan.hit
        self._add(x + scan.ranges[h] * dirs[h, 0], y + scan.ranges[h] * dirs[h, 1], L_OCC)
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)
        # cells that were seen FREE and are now OCCUPIED => something moved in
        flipped = (self.logodds > OCC_THRESHOLD) & (prev < -0.2)
        self._track_movers(flipped)

    # ---- moving-obstacle tracking + short-term motion prediction ----------
    def _track_movers(self, flipped):
        n, _, stats, cent = cv2.connectedComponentsWithStats(flipped.astype(np.uint8), connectivity=8)
        dets = [np.array([(c[0] + 0.5) * C.RES, (c[1] + 0.5) * C.RES])
                for i, c in enumerate(cent) if i > 0 and stats[i, cv2.CC_STAT_AREA] >= 1]
        used = set()
        for d in dets:
            best, bd = None, 1.5
            for k, tr in enumerate(self.tracks):
                dist = float(np.hypot(*(tr["pos"] - d)))
                if k not in used and dist < bd:
                    best, bd = k, dist
            if best is None:
                self.tracks.append({"pos": d, "vel": np.zeros(2), "last_t": self.t, "hits": 1, "miss": 0})
                used.add(len(self.tracks) - 1)
            else:
                tr = self.tracks[best]
                dt = max(self.t - tr["last_t"], C.DT)
                tr["vel"] = 0.6 * tr["vel"] + 0.4 * (d - tr["pos"]) / dt
                tr["pos"], tr["last_t"], tr["hits"], tr["miss"] = d, self.t, tr["hits"] + 1, 0
                used.add(best)
        for k, tr in enumerate(self.tracks):
            if k not in used:
                tr["miss"] += 1
        self.tracks = [t for t in self.tracks if t["miss"] <= 12]

        mask = np.zeros((C.GRID_H, C.GRID_W), np.uint8)
        for tr in self.tracks:
            speed = float(np.hypot(*tr["vel"]))
            if tr["hits"] >= 3 and 0.3 < speed < 3.0:
                for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5):
                    p = tr["pos"] + tr["vel"] * t
                    cv2.circle(mask, (int(p[0] / C.RES), int(p[1] / C.RES)), int(1.0 / C.RES), 1, -1)
        if mask.any():
            k = int(math.ceil(C.INFLATION_RADIUS / C.RES))
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
        self.dyn_blocked = mask.astype(bool)

    def moving_tracks(self):
        return [t for t in self.tracks if t["hits"] >= 3 and 0.3 < float(np.hypot(*t["vel"])) < 3.0]

    def occupied(self):
        return self.logodds > OCC_THRESHOLD

    # ---- cost map for the planner -----------------------------------------
    def costmap(self):
        """
        Returns (raw, blocked, cost, dist):
          raw     - occupied cells (bool)
          blocked - raw obstacles inflated by the robot size (bool, planner may not enter)
          cost    - per-cell travel cost (>=1); higher near obstacles so paths keep a margin
          dist    - distance to the nearest obstacle in metres (OpenCV distance transform)
        """
        raw = self.occupied().astype(np.uint8)
        raw[0, :] = raw[-1, :] = raw[:, 0] = raw[:, -1] = 1       # world boundary
        free = (raw == 0).astype(np.uint8)
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 3) * C.RES
        blocked = (dist < C.INFLATION_RADIUS) | self.dyn_blocked
        cost = 1.0 + 3.0 * np.clip((2.5 - dist) / 1.5, 0.0, 1.0)
        return raw.astype(bool), blocked, cost.astype(np.float32), dist

    def blob_count(self):
        """OpenCV connected-components: how many separate obstacles are mapped."""
        n, _ = cv2.connectedComponents(self.occupied().astype(np.uint8), connectivity=8)
        return n - 1

    # ---- obstacle detection ------------------------------------------------
    @staticmethod
    def detect(pose, scan, gap=1.0, max_items=8):
        x, y, th = pose
        idx = np.where(scan.hit)[0]
        if len(idx) == 0:
            return []
        ang = th + scan.angles_rel[idx]
        r = scan.ranges[idx]
        px, py = x + r * np.cos(ang), y + r * np.sin(ang)
        keep = (px > 0.4) & (px < C.WORLD_W - 0.4) & (py > 0.4) & (py < C.WORLD_H - 0.4)
        idx, ang, r, px, py = idx[keep], ang[keep], r[keep], px[keep], py[keep]
        if len(idx) == 0:
            return []

        clusters, cur = [], [0]
        for i in range(1, len(idx)):
            consecutive = idx[i] - idx[i - 1] == 1
            close = math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]) < gap
            if consecutive and close:
                cur.append(i)
            else:
                clusters.append(cur)
                cur = [i]
        clusters.append(cur)

        dets = []
        for c in clusters:
            cx, cy = float(px[c].mean()), float(py[c].mean())
            size = float(math.hypot(px[c].max() - px[c].min(), py[c].max() - py[c].min()))
            j = c[int(np.argmin(r[c]))]
            bearing = math.degrees(math.atan2(math.sin(ang[j] - th), math.cos(ang[j] - th)))
            dets.append({"x": round(cx, 2), "y": round(cy, 2), "size": round(max(size, 0.3), 2),
                         "distance": round(float(r[j]), 2), "bearing_deg": round(bearing, 1)})
        dets.sort(key=lambda d: d["distance"])
        for k, d in enumerate(dets[:max_items]):
            d["id"] = k + 1
        return dets[:max_items]
