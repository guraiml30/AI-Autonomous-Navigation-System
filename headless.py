"""headless.py - run a scenario without a window and save proof assets."""
import json
import os

try:
    import cv2  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - optional dependency for image export
    cv2 = None

try:
    import imageio  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - optional dependency for GIF export
    imageio = None

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from . import config as C
from .render import render
from .simulator import Simulation


def _require_cv2():
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for headless rendering and image export.")


def save_telemetry_plot(sim, path):
    h = sim.history
    fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True, facecolor=C.PALETTE["bg"])
    for a in ax:
        a.set_facecolor(C.PALETTE["bg"])
        a.tick_params(colors=C.PALETTE["text"])
        for s in a.spines.values():
            s.set_color(C.PALETTE["grid"])
        a.grid(color=C.PALETTE["grid"])
    ax[0].plot(h["t"], h["speed"], color=C.STATE_COLORS["NAVIGATING"], lw=2)
    ax[0].set_ylabel("speed (m/s)", color=C.PALETTE["text"])
    ax[1].plot(h["t"], h["clearance"], color=C.STATE_COLORS["CAUTION"], lw=2)
    ax[1].axhline(C.STOP_DIST - C.ROBOT_RADIUS, color=C.STATE_COLORS["EMERGENCY_STOP"], ls="--", label="stop zone")
    ax[1].axhline(C.CAUTION_DIST - C.ROBOT_RADIUS, color=C.STATE_COLORS["CAUTION"], ls=":", label="caution zone")
    ax[1].set_ylabel("clearance (m)", color=C.PALETTE["text"])
    ax[1].set_xlabel("time (s)", color=C.PALETTE["text"])
    ax[1].legend(facecolor=C.PALETTE["bg"], labelcolor=C.PALETTE["text"])
    fig.suptitle(f"Telemetry - {sim.scenario}", color=C.PALETTE["text"])
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)


def run_headless(scenario="warehouse", out_dir="outputs", seed=0, make_gif=True, verbose=True):
    os.makedirs(out_dir, exist_ok=True)
    sim = Simulation(scenario, seed=seed)
    frames = []
    if verbose:
        print(f"[run] scenario={scenario} seed={seed}")
    snap_marks = {}
    while not sim.done and sim.tick < C.MAX_STEPS:
        sim.step()
        if make_gif and sim.tick % 5 == 0:
            f = cv2.resize(render(sim), None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
            frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
        if verbose and sim.tick % 100 == 0:
            print(f"  t={sim.tick * C.DT:6.1f}s state={sim.state:<14} v={sim.robot.v:4.1f} "
                  f"goal_dist={sim.snapshot()['distance_to_goal']:5.1f}m replans={sim.replans}")
        if sim.state in ("EMERGENCY_STOP", "AVOIDING") and "danger" not in snap_marks:
            snap_marks["danger"] = render(sim)         # screenshot of obstacle reaction
    final = render(sim)
    cv2.imwrite(os.path.join(out_dir, f"{scenario}_final.png"), final)
    if "danger" in snap_marks:
        cv2.imwrite(os.path.join(out_dir, f"{scenario}_obstacle_reaction.png"), snap_marks["danger"])
    save_telemetry_plot(sim, os.path.join(out_dir, f"{scenario}_telemetry.png"))
    if make_gif and frames:
        if imageio is None:
            raise RuntimeError("imageio is required for GIF export.")
        imageio.mimsave(os.path.join(out_dir, f"{scenario}_demo.gif"), frames, duration=0.08, loop=0)
    m = sim.metrics()
    with open(os.path.join(out_dir, f"{scenario}_metrics.json"), "w") as fh:
        json.dump(m, fh, indent=2)
    if verbose:
        print(f"[done] {m['status']} - {m['result']} | time={m['sim_time_s']}s dist={m['distance_m']}m "
              f"replans={m['replans']} e-stops={m['emergency_stops']} min_clearance={m['min_clearance_m']}m")
    return m
