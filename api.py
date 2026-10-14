"""
api.py - REST API + live web dashboard (Flask).

Endpoints
  GET  /                  live dashboard (HTML)
  GET  /video_feed        MJPEG stream of the simulation view
  GET  /api/health        {"status": "ok"}
  GET  /api/config        scenarios + colour palette
  GET  /api/status        full telemetry snapshot (state, pose, metrics, events, history)
  GET  /api/metrics       just the mission metrics
  GET  /api/frame.png     one still frame
  POST /api/control       {"action": "start|pause|toggle|reset|step", "scenario": "...", "speed": 1-8}
"""
import os
import threading
import time
import cv2  # type: ignore[import-not-found]
from flask import Flask, Response, jsonify, request, send_file  # type: ignore[import-not-found]
from . import config as C
from .render import render
from .simulator import Simulation
from .world import SCENARIOS

DASH_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "dashboard")


class SimRunner:
    """Runs the simulation in a background thread so the API stays responsive."""

    def __init__(self, scenario="warehouse", seed=0):
        self.sim = Simulation(scenario, seed)
        self.seed = seed
        self.lock = threading.Lock()
        self.running = False
        self.speed = 2            # simulation ticks per 50 ms frame
        self.jpg = None
        self._render()
        threading.Thread(target=self._loop, daemon=True).start()

    def _render(self):
        ok, buf = cv2.imencode(".jpg", render(self.sim), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            self.jpg = buf.tobytes()

    def _loop(self):
        while True:
            with self.lock:
                if self.running and not self.sim.done:
                    for _ in range(self.speed):
                        self.sim.step()
                        if self.sim.done:
                            self.running = False
                            break
                self._render()
            time.sleep(0.05)

    def control(self, data):
        with self.lock:
            action = data.get("action")
            if "speed" in data:
                self.speed = max(1, min(8, int(data["speed"])))
            if action == "start":
                if self.sim.done:
                    self.sim.reset()
                self.running = True
            elif action == "pause":
                self.running = False
            elif action == "toggle":
                self.running = not self.running
            elif action == "reset":
                self.running = False
                sc = data.get("scenario")
                if sc and sc not in SCENARIOS:
                    return {"error": f"unknown scenario {sc}"}, 400
                self.sim.reset(sc)
            elif action == "step":
                self.sim.step()
            self._render()
            snap = self.sim.snapshot()
            snap["running"], snap["sim_speed"] = self.running, self.speed
            return snap, 200

    def snapshot(self):
        with self.lock:
            snap = self.sim.snapshot()
            snap["running"], snap["sim_speed"] = self.running, self.speed
            return snap


def create_app(runner=None, scenario="warehouse"):
    runner = runner or SimRunner(scenario)
    app = Flask(__name__, static_folder=None)
    app.config["runner"] = runner

    @app.after_request
    def cors(resp):
        resp.headers["Access-Control-Allow-Origin"] = "*"
        resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return resp

    @app.get("/")
    def index():
        return send_file(os.path.join(DASH_DIR, "index.html"))

    @app.get("/api/health")
    def health():
        return jsonify(status="ok")

    @app.get("/api/config")
    def config():
        return jsonify(scenarios=SCENARIOS, state_colors=C.STATE_COLORS, palette=C.PALETTE,
                       stop_dist=C.STOP_DIST, caution_dist=C.CAUTION_DIST, max_v=C.MAX_V)

    @app.get("/api/status")
    def status():
        return jsonify(runner.snapshot())

    @app.get("/api/metrics")
    def metrics():
        return jsonify(runner.snapshot()["metrics"])

    @app.get("/api/frame.png")
    def frame():
        with runner.lock:
            ok, buf = cv2.imencode(".png", render(runner.sim))
        return Response(buf.tobytes(), mimetype="image/png")

    @app.post("/api/control")
    def control():
        body, code = runner.control(request.get_json(silent=True) or {})
        return jsonify(body), code

    @app.get("/video_feed")
    def video_feed():
        def gen():
            while True:
                if runner.jpg:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + runner.jpg + b"\r\n"
                time.sleep(0.05)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def serve(host="127.0.0.1", port=5000, scenario="warehouse", autostart=False):
    runner = SimRunner(scenario)
    runner.running = autostart
    app = create_app(runner)
    print(f"[api] dashboard -> http://{host}:{port}   (Ctrl+C to stop)")
    app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)
