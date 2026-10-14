"""
run_simulation.py - run ONE scenario.

  python run_simulation.py --scenario warehouse            # headless, saves proof assets to outputs/
  python run_simulation.py --scenario dynamic --live       # opens a live OpenCV window (press q to quit)
"""
import argparse
import importlib
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")
if os.path.isdir(SRC_ROOT):
    sys.path.insert(0, SRC_ROOT)

# Pylance does not evaluate runtime sys.path mutations, so dynamic imports keep both
# runtime behavior and static analysis happy when the package is located under src/.
run_headless = importlib.import_module("autonav.headless").run_headless
SCENARIOS = importlib.import_module("autonav.world").SCENARIOS


def live(scenario, seed):
    import cv2  # type: ignore[import-not-found]

    C = importlib.import_module("autonav.config")
    render = importlib.import_module("autonav.render").render
    Simulation = importlib.import_module("autonav.simulator").Simulation
    sim = Simulation(scenario, seed)
    while not sim.done:
        sim.step()
        cv2.imshow("AutoNav - press q to quit", render(sim))
        if cv2.waitKey(int(C.DT * 500)) & 0xFF == ord("q"):
            break
    cv2.waitKey(1500)
    cv2.destroyAllWindows()
    print(sim.metrics())


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="warehouse", choices=list(SCENARIOS))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="outputs")
    ap.add_argument("--live", action="store_true", help="show a live window instead of saving files")
    ap.add_argument("--no-gif", action="store_true")
    a = ap.parse_args()
    if a.live:
        live(a.scenario, a.seed)
    else:
        run_headless(a.scenario, a.out, a.seed, make_gif=not a.no_gif)
