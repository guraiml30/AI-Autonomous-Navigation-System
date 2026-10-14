"""
render.py - draws the simulation as an image (used by the dashboard stream,
the saved screenshots/GIFs and the optional live OpenCV window).
"""
try:
    import cv2  # type: ignore[reportMissingImports]
except ImportError:  # pragma: no cover
    raise RuntimeError("OpenCV is required for autonav.render") from None

import numpy as np
from . import config as C

S = 15  # pixels per metre  -> 900 x 600 image
W_PX, H_PX = int(C.WORLD_W * S), int(C.WORLD_H * S)
col = {k: C.hex_to_bgr(v) for k, v in C.PALETTE.items()}
scol = {k: C.hex_to_bgr(v) for k, v in C.STATE_COLORS.items()}


def px(x, y):
    return int(round(x * S)), int(round((C.WORLD_H - y) * S))


def _grid_to_img(mask, color, img, alpha):
    """Paint a boolean occupancy grid onto the image with transparency."""
    m = cv2.resize(mask.astype(np.uint8), (W_PX, H_PX), interpolation=cv2.INTER_NEAREST)
    m = m[::-1] > 0                                     # flip y: row 0 is the bottom
    overlay = img.copy()
    overlay[m] = color
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, dst=img)


def render(sim, hud=True):
    img = np.full((H_PX, W_PX, 3), col["bg"], np.uint8)
    for gx in range(0, int(C.WORLD_W) + 1, 5):
        cv2.line(img, px(gx, 0), px(gx, C.WORLD_H), col["grid"], 1)
    for gy in range(0, int(C.WORLD_H) + 1, 5):
        cv2.line(img, px(0, gy), px(C.WORLD_W, gy), col["grid"], 1)

    # real obstacles (dim) -> what the robot has mapped (bright) -> safety margin
    _grid_to_img(sim.world.static_grid, col["truth_obstacle"], img, 1.0)
    _grid_to_img(sim.blocked & ~sim.raw, col["inflation"], img, 0.22)
    _grid_to_img(sim.raw, col["mapped_obstacle"], img, 1.0)

    # predicted motion of tracked moving obstacles (magenta) + velocity arrows
    _grid_to_img(sim.perception.dyn_blocked, col["dynamic_obstacle"], img, 0.18)
    for tr in sim.perception.moving_tracks():
        a, b = tr["pos"], tr["pos"] + tr["vel"] * 2.0
        cv2.arrowedLine(img, px(*a), px(*b), col["dynamic_obstacle"], 2, cv2.LINE_AA, tipLength=0.3)

    # LiDAR
    layer = img.copy()
    rx, ry, th = sim.robot.pose
    ang = th + sim.scan.angles_rel
    for a, r, h in zip(ang, sim.scan.ranges, sim.scan.hit):
        end = (rx + r * np.cos(a), ry + r * np.sin(a))
        cv2.line(layer, px(rx, ry), px(*end), col["lidar_ray"], 1, cv2.LINE_AA)
    cv2.addWeighted(layer, 0.25, img, 0.75, 0, dst=img)
    for a, r, h in zip(ang, sim.scan.ranges, sim.scan.hit):
        if h:
            cv2.circle(img, px(rx + r * np.cos(a), ry + r * np.sin(a)), 2, col["lidar_hit"], -1)

    # moving obstacles
    for d in sim.world.dynamics:
        cv2.circle(img, px(*d.pos), int(d.radius * S), col["dynamic_obstacle"], -1, cv2.LINE_AA)
        cv2.circle(img, px(*d.pos), int(d.radius * S) + 3, col["dynamic_obstacle"], 1, cv2.LINE_AA)

    # trail + planned path
    if len(sim.trail) > 1:
        pts = np.array([px(*p) for p in sim.trail[::2]], np.int32)
        cv2.polylines(img, [pts], False, col["trail"], 2, cv2.LINE_AA)
    if sim.plan is not None:
        rem = sim.controller.remaining()
        if len(rem) > 1:
            pts = np.array([px(*p) for p in rem[::2]], np.int32)
            cv2.polylines(img, [pts], False, col["path"], 2, cv2.LINE_AA)
            cv2.circle(img, px(*sim.controller.lookahead_point()), 4, (255, 255, 255), -1, cv2.LINE_AA)

    # detections
    for d in sim.detections:
        c = px(d["x"], d["y"])
        cv2.circle(img, c, int(max(d["size"], 0.8) * S / 2) + 8, scol["EMERGENCY_STOP"], 1, cv2.LINE_AA)
        cv2.putText(img, f"#{d['id']} {d['distance']:.1f}m", (c[0] + 10, c[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, scol["EMERGENCY_STOP"], 1, cv2.LINE_AA)

    # start, goal
    sx, sy, _ = sim.world.start
    gx, gy = sim.world.goal
    cv2.circle(img, px(sx, sy), 6, col["start"], 2, cv2.LINE_AA)
    cv2.circle(img, px(gx, gy), int(C.GOAL_TOL * S), col["goal"], 2, cv2.LINE_AA)
    cv2.circle(img, px(gx, gy), 5, col["goal"], -1, cv2.LINE_AA)
    cv2.putText(img, "GOAL", (px(gx, gy)[0] - 18, px(gx, gy)[1] - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col["goal"], 1, cv2.LINE_AA)

    # robot, coloured by navigation state
    sc = scol[sim.state]
    cv2.circle(img, px(rx, ry), int(C.ROBOT_RADIUS * S) + 4, sc, 2, cv2.LINE_AA)
    cv2.circle(img, px(rx, ry), int(C.ROBOT_RADIUS * S), sc, -1, cv2.LINE_AA)
    cv2.line(img, px(rx, ry), px(rx + 1.2 * np.cos(th), ry + 1.2 * np.sin(th)), (255, 255, 255), 2, cv2.LINE_AA)

    if hud:
        cv2.rectangle(img, (8, 8), (300, 74), (20, 28, 46), -1)
        cv2.rectangle(img, (8, 8), (12, 74), sc, -1)
        cv2.putText(img, sim.state.replace("_", " "), (22, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, sc, 2, cv2.LINE_AA)
        cv2.putText(img, f"t={sim.tick * C.DT:5.1f}s  v={sim.robot.v:4.1f} m/s  scenario={sim.scenario}",
                    (22, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col["text"], 1, cv2.LINE_AA)
        cv2.putText(img, f"goal {np.hypot(gx - rx, gy - ry):4.1f} m   replans={sim.replans}   e-stops={sim.emergency_stops}",
                    (22, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col["text"], 1, cv2.LINE_AA)
    return img
