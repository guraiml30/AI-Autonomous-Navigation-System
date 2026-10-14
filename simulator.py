"""
simulator.py - the main loop that ties every module together.

One tick (0.1 s) =
   world moves -> LiDAR scan -> perception -> (re)plan if needed -> control -> robot moves
"""
import math
import numpy as np
from . import config as C
from .controller import Controller
from .perception import Perception
from .planner import astar
from .robot import Robot
from .sensors import Lidar
from .world import make_world


class Simulation:
    def __init__(self, scenario="warehouse", seed=0):
        self.seed = seed
        self.scenario = scenario
        self.reset(scenario)

    # ------------------------------------------------------------------ setup
    def reset(self, scenario=None):
        if scenario:
            self.scenario = scenario
        self.world = make_world(self.scenario)
        sx, sy, sth = self.world.start
        self.robot = Robot(sx, sy, sth)
        self.lidar = Lidar(seed=self.seed)
        self.perception = Perception()
        self.controller = Controller()
        self.tick = 0
        self.status = "READY"          # READY | RUNNING | SUCCESS | FAILED
        self.state = "IDLE"
        self.result_reason = ""
        self.trail = [(sx, sy)]
        self.events = []
        self.history = {"t": [], "speed": [], "clearance": []}
        self.replans = 0
        self.emergency_stops = 0
        self.distance = 0.0
        self.min_clearance = C.LIDAR_RANGE
        self.replan_flash = 0
        self.last_plan_tick = 0
        self.no_path_ticks = 0
        self.estop_ticks = 0
        self.scan = None
        self.detections = []
        self.raw = None
        self.blocked = None
        self.cost = None
        self.dist = None
        self.plan = None
        self._perceive()
        self._plan("initial plan")
        self._log(f"Scenario '{self.scenario}' loaded")

    # ---------------------------------------------------------------- helpers
    def _log(self, msg):
        self.events.append({"t": round(self.tick * C.DT, 1), "msg": msg})
        self.events = self.events[-60:]

    def _perceive(self):
        self.scan = self.lidar.scan(self.world, self.robot.pose)
        self.perception.update(self.robot.pose, self.scan)
        self.detections = self.perception.detect(self.robot.pose, self.scan)
        self.raw, self.blocked, self.cost, self.dist = self.perception.costmap()

    def _plan(self, reason):
        path = astar(self.blocked, self.raw, (self.robot.x, self.robot.y), self.world.goal,
                     cost=self.cost, smooth_grid=self.dist < 1.6)
        self.last_plan_tick = self.tick
        if path is None:
            self.plan = None
            self.controller.clear()
            return False
        self.plan = np.array(path)
        self.controller.set_path(path)
        if reason not in ("initial plan", "periodic refresh"):
            self.replans += 1
            self.replan_flash = 6
            self._log(f"Replanned: {reason}")
        return True

    def _path_blocked(self):
        rem = self.controller.remaining()
        if len(rem) == 0:
            return False
        d = np.hypot(rem[:, 0] - self.robot.x, rem[:, 1] - self.robot.y)
        ahead = rem[(d > 1.5) & (d < 12.0)]
        if len(ahead) == 0:
            return False
        ix = np.clip((ahead[:, 0] // C.RES).astype(int), 0, C.GRID_W - 1)
        iy = np.clip((ahead[:, 1] // C.RES).astype(int), 0, C.GRID_H - 1)
        return bool(self.blocked[iy, ix].any())

    def _set_state(self, new):
        if new != self.state:
            if new == "EMERGENCY_STOP":
                self.emergency_stops += 1
                self._log("Emergency stop - obstacle too close")
            elif new in ("CAUTION", "AVOIDING", "NO_PATH", "GOAL_REACHED", "COLLISION"):
                self._log(new.replace("_", " ").title())
            self.state = new

    @property
    def done(self):
        return self.status in ("SUCCESS", "FAILED")

    # ------------------------------------------------------------------- tick
    def step(self):
        if self.done:
            return
        self.status = "RUNNING"
        self.tick += 1
        self.world.step(C.DT)
        self._perceive()

        # ---- decide whether to (re)plan --------------------------------
        if self.controller.path is None:
            if self.tick % 5 == 0:
                self._plan("retrying after no path")
        elif self._path_blocked() and self.tick - self.last_plan_tick >= 4:
            self._plan("path blocked by newly detected obstacle")
        elif self.estop_ticks >= 4 and self.tick - self.last_plan_tick >= 8:
            self._plan("emergency stop")
        elif self.tick - self.last_plan_tick >= C.REPLAN_PERIOD:
            self._plan("periodic refresh")

        # ---- control ----------------------------------------------------
        v, w, state = self.controller.command(self.robot.pose, self.world.goal, self.scan)
        if state == "NO_PATH":
            self.no_path_ticks += 1
            if self.no_path_ticks > C.NO_PATH_TIMEOUT:
                self.status, self.result_reason = "FAILED", "No route to goal"
        else:
            self.no_path_ticks = 0
            if state == "NAVIGATING" and self.replan_flash > 0:
                state = "REPLANNING"
        if self.replan_flash > 0:
            self.replan_flash -= 1
        self.estop_ticks = self.estop_ticks + 1 if state == "EMERGENCY_STOP" else 0

        # ---- act --------------------------------------------------------
        px, py = self.robot.x, self.robot.y
        self.robot.step(v, w)
        self.distance += math.hypot(self.robot.x - px, self.robot.y - py)
        self.trail.append((self.robot.x, self.robot.y))

        # ---- outcome checks ----------------------------------------------
        if self.world.collision(self.robot.x, self.robot.y):
            state, self.status, self.result_reason = "COLLISION", "FAILED", "Collision"
        elif math.hypot(self.world.goal[0] - self.robot.x, self.world.goal[1] - self.robot.y) < C.GOAL_TOL:
            state, self.status, self.result_reason = "GOAL_REACHED", "SUCCESS", "Goal reached"
            self.robot.v = 0.0
        elif self.tick >= C.MAX_STEPS:
            self.status, self.result_reason = "FAILED", "Timeout"
        self._set_state(state)
        if self.status == "FAILED" and self.state not in ("COLLISION",):
            self._log(f"Mission failed: {self.result_reason}")

        # ---- bookkeeping ----------------------------------------------------
        clr = float(self.scan.ranges.min()) - C.ROBOT_RADIUS
        if self.scan.hit.any():
            self.min_clearance = min(self.min_clearance, clr)
        if self.tick % 2 == 0:
            self.history["t"].append(round(self.tick * C.DT, 1))
            self.history["speed"].append(round(self.robot.v, 2))
            self.history["clearance"].append(round(min(clr, C.LIDAR_RANGE), 2))
            for k in self.history:
                self.history[k] = self.history[k][-400:]

    def run_until_done(self, max_steps=C.MAX_STEPS):
        while not self.done and self.tick < max_steps:
            self.step()
        return self.metrics()

    # ----------------------------------------------------------------- output
    def metrics(self):
        t = max(self.tick * C.DT, 1e-6)
        gx, gy = self.world.goal
        sx, sy, _ = self.world.start
        straight = math.hypot(gx - sx, gy - sy)
        return {
            "scenario": self.scenario,
            "status": self.status,
            "result": self.result_reason or "-",
            "sim_time_s": round(t, 1),
            "distance_m": round(self.distance, 1),
            "straight_line_m": round(straight, 1),
            "path_efficiency_pct": round(100 * straight / self.distance, 1) if self.distance > 1 else 0.0,
            "avg_speed_mps": round(self.distance / t, 2),
            "replans": self.replans,
            "emergency_stops": self.emergency_stops,
            "min_clearance_m": round(self.min_clearance, 2),
            "collisions": 1 if self.state == "COLLISION" else 0,
            "mapped_obstacles": self.perception.blob_count(),
        }

    def snapshot(self):
        r = self.robot
        gx, gy = self.world.goal
        return {
            "scenario": self.scenario,
            "status": self.status,
            "state": self.state,
            "state_color": C.STATE_COLORS[self.state],
            "tick": self.tick,
            "time_s": round(self.tick * C.DT, 1),
            "pose": {"x": round(r.x, 2), "y": round(r.y, 2), "heading_deg": round(math.degrees(r.theta), 1)},
            "speed": round(r.v, 2),
            "goal": {"x": gx, "y": gy},
            "distance_to_goal": round(math.hypot(gx - r.x, gy - r.y), 1),
            "front_obstacle_m": round(float(self.scan.ranges.min()), 2) if self.scan.hit.any() else None,
            "detections": self.detections,
            "moving_obstacles": len(self.perception.moving_tracks()),
            "metrics": self.metrics(),
            "events": self.events[-8:][::-1],
            "history": self.history,
        }
