"""
runall.py - ONE command that runs the WHOLE pipeline and then deploys the API.

  python runall.py                    full pipeline + start API/dashboard on http://127.0.0.1:5000
  python runall.py --no-server        pipeline only (CI-style run, exits when finished)
  python runall.py --fast             skip GIF creation (quicker)
  python runall.py --host 0.0.0.0     expose the dashboard on your local network

Stages
  1. environment check      (python + libraries)
  2. unit tests             (planner, perception, simulation)
  3. simulations            (every scenario -> PNG, GIF, telemetry plot, JSON metrics)
  4. benchmark              (results table + comparison chart + README images)
  5. API self-test          (calls every REST endpoint)
  6. deploy                 (REST API + live dashboard)
"""
import argparse
import importlib
import json
import os
import shutil
import sys
import threading
import time
import unittest
import webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "src")
for path in (SRC, ROOT):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
OUT, IMG = os.path.join(ROOT, "outputs"), os.path.join(ROOT, "images")


def banner(n, text):
    print(f"\n{'=' * 64}\n  STAGE {n}: {text}\n{'=' * 64}")


def stage_env():
    banner(1, "Environment check")
    print(f"Python {sys.version.split()[0]}")
    missing = []
    for mod in ("numpy", "cv2", "matplotlib", "flask", "imageio"):
        try:
            m = importlib.import_module(mod)
            print(f"  [ok] {mod:<11} {getattr(m, '__version__', '')}")
        except ImportError:
            print(f"  [!!] {mod} missing")
            missing.append(mod)
    if missing:
        sys.exit("\nInstall dependencies first:  pip install -r requirements.txt")


def stage_tests():
    banner(2, "Unit tests")
    suite = unittest.defaultTestLoader.discover(os.path.join(ROOT, "tests"), top_level_dir=ROOT)
    res = unittest.TextTestRunner(verbosity=1).run(suite)
    if not res.wasSuccessful():
        sys.exit("Tests failed - fix them before publishing results.")


def stage_sims(scenarios, gif):
    banner(3, "Running simulations")
    run_headless = importlib.import_module("autonav.headless").run_headless
    results = []
    for sc in scenarios:
        results.append(run_headless(sc, OUT, seed=0, make_gif=gif))
    return results


def stage_benchmark(results, scenarios, seeds=10):
    banner(4, f"Benchmark ({seeds} random seeds per scenario)")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    C = importlib.import_module("autonav.config")
    Simulation = importlib.import_module("autonav.simulator").Simulation

    rows = []
    for sc in scenarios:
        ok, times, dists, reps, ests, clr = 0, [], [], [], [], []
        for seed in range(seeds):
            m = Simulation(sc, seed).run_until_done()
            ok += m["status"] == "SUCCESS"
            times.append(m["sim_time_s"]); dists.append(m["distance_m"])
            reps.append(m["replans"]); ests.append(m["emergency_stops"]); clr.append(m["min_clearance_m"])
        avg = lambda v: round(sum(v) / len(v), 1)
        rows.append({"scenario": sc, "success_rate": round(100 * ok / seeds), "avg_time_s": avg(times),
                     "avg_distance_m": avg(dists), "avg_replans": avg(reps), "avg_estops": avg(ests),
                     "worst_clearance_m": round(min(clr), 2)})
        print(f"  {sc:<10} success {rows[-1]['success_rate']:>3}%  time {rows[-1]['avg_time_s']}s  "
              f"replans {rows[-1]['avg_replans']}  e-stops {rows[-1]['avg_estops']}")

    md = ["| Scenario | Success rate | Avg time (s) | Avg distance (m) | Avg replans | Avg e-stops | Worst clearance (m) |",
          "|---|---|---|---|---|---|---|"]
    for r in rows:
        md.append(f"| {r['scenario']} | {r['success_rate']}% | {r['avg_time_s']} | {r['avg_distance_m']} | "
                  f"{r['avg_replans']} | {r['avg_estops']} | {r['worst_clearance_m']} |")
    with open(os.path.join(OUT, "results.md"), "w", encoding="utf-8") as f:
        f.write(f"Benchmark over {seeds} seeds per scenario\n\n" + "\n".join(md) + "\n")
    with open(os.path.join(OUT, "benchmark.json"), "w") as f:
        json.dump(rows, f, indent=2)

    fig, ax = plt.subplots(1, 4, figsize=(14, 3.6), facecolor=C.PALETTE["bg"])
    spec = [("success_rate", "Success rate (%)", C.STATE_COLORS["NAVIGATING"]),
            ("avg_time_s", "Avg mission time (s)", C.PALETTE["path"]),
            ("avg_replans", "Avg replans", C.STATE_COLORS["REPLANNING"]),
            ("avg_estops", "Avg emergency stops", C.STATE_COLORS["EMERGENCY_STOP"])]
    for a, (key, title, color) in zip(ax, spec):
        a.set_facecolor(C.PALETTE["bg"])
        a.bar([r["scenario"] for r in rows], [r[key] for r in rows], color=color)
        a.set_title(title, color=C.PALETTE["text"], fontsize=10)
        a.tick_params(colors=C.PALETTE["text"])
        for s in a.spines.values():
            s.set_color(C.PALETTE["grid"])
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "benchmark_summary.png"), dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)

    os.makedirs(IMG, exist_ok=True)                     # copy proof assets to images/ for the README
    for f in os.listdir(OUT):
        if f.endswith((".png", ".gif")):
            shutil.copy(os.path.join(OUT, f), os.path.join(IMG, f))
    print(f"  results table -> outputs/results.md   images copied -> images/")


def stage_api_selftest():
    banner(5, "API self-test")
    api_mod = importlib.import_module("autonav.api")
    SimRunner = api_mod.SimRunner
    create_app = api_mod.create_app
    runner = SimRunner("easy")
    client = create_app(runner).test_client()
    checks = [("GET", "/api/health"), ("GET", "/api/config"), ("GET", "/api/status"),
              ("GET", "/api/metrics"), ("GET", "/api/frame.png"), ("GET", "/")]
    for method, url in checks:
        r = client.open(url, method=method)
        print(f"  {method} {url:<18} -> {r.status_code}")
        assert r.status_code == 200, url
    r = client.post("/api/control", json={"action": "step"})
    assert r.status_code == 200 and r.get_json()["tick"] == 1
    print("  POST /api/control      -> 200")
    print("  all endpoints OK")


def stage_deploy(host, port, scenario, open_browser):
    banner(6, "Deploying API + dashboard")
    api_mod = importlib.import_module("autonav.api")
    SimRunner = api_mod.SimRunner
    create_app = api_mod.create_app
    runner = SimRunner(scenario)
    app = create_app(runner)
    url = f"http://{'127.0.0.1' if host == '0.0.0.0' else host}:{port}"
    print(f"  Dashboard : {url}")
    print(f"  API status: {url}/api/status")
    print("  Press START on the dashboard.  Ctrl+C here to stop.")
    if open_browser:
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()
    app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    SCENARIOS = importlib.import_module("autonav.world").SCENARIOS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-server", action="store_true", help="run the pipeline but do not start the API")
    ap.add_argument("--skip-tests", action="store_true")
    ap.add_argument("--fast", action="store_true", help="skip GIF generation")
    ap.add_argument("--seeds", type=int, default=10, help="benchmark seeds per scenario")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--scenario", default="warehouse", choices=list(SCENARIOS), help="scenario preloaded in the dashboard")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()

    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    stage_env()
    if not a.skip_tests:
        stage_tests()
    scen = list(SCENARIOS)
    res = stage_sims(scen, gif=not a.fast)
    stage_benchmark(res, scen, a.seeds)
    stage_api_selftest()
    print(f"\nPipeline finished in {time.time() - t0:.0f}s. Proof assets are in outputs/ and images/.")
    if not a.no_server:
        stage_deploy(a.host, a.port, a.scenario, not a.no_browser)
