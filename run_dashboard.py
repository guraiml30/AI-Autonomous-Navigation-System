"""
run_dashboard.py - start ONLY the REST API + live dashboard.

  python run_dashboard.py                 -> http://127.0.0.1:5000
  python run_dashboard.py --autostart --scenario dynamic
"""
import argparse
import os
import sys

# pyright: reportMissingImports=false
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))
from autonav.api import serve                      # noqa: E402
from autonav.world import SCENARIOS                # noqa: E402

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--scenario", default="warehouse", choices=list(SCENARIOS))
    ap.add_argument("--autostart", action="store_true")
    a = ap.parse_args()
    serve(a.host, a.port, a.scenario, a.autostart)
