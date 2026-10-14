# pyright: reportMissingImports=false
import os
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from autonav import config as C                      # noqa: E402
from autonav.perception import Perception           # noqa: E402
from autonav.planner import astar, world_to_cell     # noqa: E402
from autonav.sensors import Lidar                    # noqa: E402
from autonav.simulator import Simulation             # noqa: E402
from autonav.world import make_world                 # noqa: E402


def empty_grid():
    return np.zeros((C.GRID_H, C.GRID_W), dtype=bool)


class TestPlanner(unittest.TestCase):
    def test_straight_path_in_empty_map(self):
        g = empty_grid()
        path = astar(g, g, (5, 5), (50, 30))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (5.0, 5.0))
        self.assertEqual(path[-1], (50.0, 30.0))

    def test_path_goes_around_wall(self):
        g = empty_grid()
        g[:60, 58:62] = True                       # wall with a gap at the top
        path = astar(g, g, (5, 20), (55, 20))
        self.assertIsNotNone(path)
        self.assertTrue(max(p[1] for p in path) > 29)   # had to detour through the gap

    def test_no_path_when_goal_enclosed(self):
        g = empty_grid()
        g[:, 58:62] = True                         # full wall
        self.assertIsNone(astar(g, g, (5, 20), (55, 20)))

    def test_path_never_enters_blocked_cells(self):
        g = empty_grid()
        g[20:50, 50:56] = True
        path = astar(g, g, (5, 20), (55, 20))
        for x, y in path:
            ix, iy = world_to_cell(x, y)
            self.assertFalse(g[iy, ix])


class TestPerception(unittest.TestCase):
    def test_lidar_detects_obstacle_ahead(self):
        w = make_world("easy")
        lidar = Lidar(noise=0.0)
        scan = lidar.scan(w, (10.0, 14.0, 0.0))   # block at x=14..19, y=8..22 straight ahead
        ahead = scan.ranges[np.argmin(np.abs(scan.angles_rel))]
        self.assertAlmostEqual(ahead, 4.0, delta=0.3)

    def test_mapping_marks_obstacle_and_detection_reports_it(self):
        w, p = make_world("easy"), Perception()
        pose = (10.0, 14.0, 0.0)
        scan = Lidar(noise=0.0).scan(w, pose)
        p.update(pose, scan)
        iy, ix = int(14 / C.RES), int(14.2 / C.RES)
        self.assertTrue(p.occupied()[iy, ix])
        dets = p.detect(pose, scan)
        self.assertTrue(len(dets) >= 1)
        self.assertLess(dets[0]["distance"], 5.0)

    def test_costmap_inflates_obstacles(self):
        w, p = make_world("easy"), Perception()
        pose = (10.0, 14.0, 0.0)
        p.update(pose, Lidar(noise=0.0).scan(w, pose))
        raw, blocked, cost, dist = p.costmap()
        self.assertGreater(blocked.sum(), raw.sum())


class TestSimulation(unittest.TestCase):
    def test_easy_scenario_succeeds_without_collision(self):
        m = Simulation("easy", seed=0).run_until_done()
        self.assertEqual(m["status"], "SUCCESS")
        self.assertEqual(m["collisions"], 0)

    def test_warehouse_scenario_succeeds(self):
        self.assertEqual(Simulation("warehouse", seed=0).run_until_done()["status"], "SUCCESS")

    def test_dynamic_scenario_succeeds(self):
        self.assertEqual(Simulation("dynamic", seed=0).run_until_done()["status"], "SUCCESS")

    def test_snapshot_is_json_serialisable(self):
        import json
        s = Simulation("easy")
        s.step()
        json.dumps(s.snapshot())


if __name__ == "__main__":
    unittest.main()
