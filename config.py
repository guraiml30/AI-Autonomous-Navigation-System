"""
config.py - ONE place for every tunable number and colour in the project.

Units: metres, seconds, radians.  World frame: x -> right, y -> up, theta CCW.
"""

# ---------------- World / map ----------------
WORLD_W = 60.0                 # metres
WORLD_H = 40.0
RES = 0.5                      # occupancy-grid cell size (m)
GRID_W = int(WORLD_W / RES)    # 120 cells
GRID_H = int(WORLD_H / RES)    # 80 cells

# ---------------- Simulation ----------------
DT = 0.1                       # seconds per tick (10 Hz control loop)
MAX_STEPS = 4000               # timeout = 400 s of simulated time
GOAL_TOL = 0.8                 # reached goal when closer than this (m)

# ---------------- Robot ----------------
ROBOT_RADIUS = 0.6
INFLATION_RADIUS = 1.0         # robot radius + safety margin used by the planner
MAX_V = 2.5                    # m/s
MAX_W = 3.0                    # rad/s
ACCEL = 0.4                    # speed gained per tick
BRAKE = 0.9                    # speed lost per tick (brakes harder than accelerates)

# ---------------- Sensor (simulated 360 deg LiDAR) ----------------
LIDAR_RANGE = 10.0
LIDAR_RAYS = 90                # one ray every 4 degrees
LIDAR_NOISE = 0.03             # std-dev of range noise (m)

# ---------------- Navigation logic ----------------
CAUTION_DIST = 3.5             # start slowing down / steering away
STOP_DIST = 1.1                # emergency stop below this distance ahead
CORRIDOR_HALF_WIDTH = 0.8     # "swept corridor" half width used for front check
LOOKAHEAD = 2.5                # pure-pursuit look-ahead distance (m)
REPLAN_PERIOD = 100            # silent re-plan every N ticks (finds shorter routes)
NO_PATH_TIMEOUT = 200          # ticks without any path before giving up

# ---------------- Colours (hex) - used by OpenCV view AND the web dashboard ----
# Colour language: green = safe, amber = caution, orange = avoiding, red = danger,
# blue = plan / replanning, purple = no path, gold = goal.
PALETTE = {
    "bg": "#0b1220",
    "grid": "#16233a",
    "truth_obstacle": "#1d2b44",    # real (not yet discovered) obstacles, dim
    "mapped_obstacle": "#8aa0bd",   # obstacles the robot has actually perceived
    "inflation": "#ef4444",         # safety margin overlay (drawn transparent)
    "dynamic_obstacle": "#d946ef",  # moving obstacles (magenta)
    "lidar_ray": "#22d3ee",
    "lidar_hit": "#f43f5e",
    "path": "#3b82f6",              # planned path (blue)
    "trail": "#86efac",             # where the robot has been (light green)
    "goal": "#facc15",              # gold
    "start": "#60a5fa",
    "text": "#e2e8f0",
}

STATE_COLORS = {
    "IDLE": "#94a3b8",
    "NAVIGATING": "#22c55e",       # green  - all clear
    "CAUTION": "#f59e0b",          # amber  - obstacle ahead, slowing
    "AVOIDING": "#f97316",         # orange - steering around obstacle
    "REPLANNING": "#38bdf8",       # sky    - computing a new path
    "EMERGENCY_STOP": "#ef4444",   # red    - too close, braking / backing off
    "NO_PATH": "#a855f7",          # purple - planner found no route
    "GOAL_REACHED": "#34d399",     # emerald- mission success
    "COLLISION": "#b91c1c",        # dark red - mission failed
}


def hex_to_bgr(h: str):
    """'#rrggbb' -> (b, g, r) tuple for OpenCV."""
    h = h.lstrip("#")
    return (int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16))
