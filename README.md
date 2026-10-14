<div align="center">

# 🤖 AI-Based Autonomous Navigation System

**A virtual robot that senses, maps, plans, and drives itself through cluttered and dynamic environments, with a live mission-control dashboard and REST API.**

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-computer%20vision-5C3EE8?logo=opencv&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-vectorised-013243?logo=numpy)
![Flask](https://img.shields.io/badge/Flask-REST%20API-000000?logo=flask)
![License](https://img.shields.io/badge/License-MIT-green)

![demo](images/warehouse_demo.gif)

</div>

## 📌 Overview
This project implements the full **sense → map → decide → plan → act** loop of an autonomous mobile robot (AGV / delivery robot / self-driving agent) inside a **custom 2D virtual simulator**. No hardware and no GPU are required.

The robot knows **nothing** about the map in advance. It discovers obstacles through a simulated noisy 360° LiDAR, builds an occupancy map, plans routes with **A\***, follows them with a **pure-pursuit controller**, tracks and predicts **moving obstacles**, and falls back on a **reactive safety layer** (slow down → steer → emergency stop / back off) when something gets too close.

## 🎯 Problem statement
Robots and vehicles must reach a goal safely in places they have never seen, around obstacles that may move. Hard-coded routes break the moment the environment changes. This project shows how perception, planning and control combine to solve that problem, and measures how reliably they do it.

## 🏭 Industry relevance
| Domain | Same ideas used for |
|---|---|
| Warehouse automation | AGVs/AMRs moving shelves and totes around people and forklifts |
| Autonomous vehicles | Perception → prediction → planning → control stack |
| Delivery robots | Sidewalk navigation around pedestrians |
| Drones / industrial safety | Obstacle-aware flight, safety-stop zones around machinery |

## 🧰 Tech stack
Python · NumPy (vectorised ray-casting & mapping) · OpenCV (distance transform, connected components, rendering) · Matplotlib (telemetry plots) · Flask (REST API + MJPEG stream) · vanilla JS dashboard · unittest

## 🏗️ Architecture
```
                ┌──────────────┐
                │  world.py    │  hidden ground truth (obstacles, movers)
                └──────┬───────┘
                       │ ray casting
                ┌──────▼───────┐
                │  sensors.py  │  noisy 360° LiDAR  ──► Scan(angles, ranges, hits)
                └──────┬───────┘
                ┌──────▼─────────────────────────────────────────────┐
                │ perception.py                                      │
                │  log-odds occupancy map · obstacle clustering ·    │
                │  moving-obstacle tracking + 2.5 s prediction ·     │
                │  OpenCV cost-map (inflation / distance transform)  │
                └──────┬─────────────────────────┬───────────────────┘
                       │ cost-map                │ raw scan
                ┌──────▼───────┐          ┌──────▼───────────────┐
                │  planner.py  │ path     │   controller.py      │
                │  A* + smooth ├─────────►│ pure pursuit +       │──► (v, w)
                └──────────────┘          │ reactive safety layer│
                       ▲ replan           └──────────────────────┘
                       │                          │
                ┌──────┴──────────────────────────▼──┐
                │ simulator.py  (state machine, metrics)│──► robot.py (kinematics)
                └──────┬──────────────────────────────┘
                       │ snapshot / frames
           ┌───────────▼───────────┐     ┌────────────────────┐
           │ api.py  (Flask REST)  │────►│ dashboard/index.html│
           └───────────────────────┘     └────────────────────┘
```

## 📁 Folder structure
```
AI-Autonomous-Navigation-System/
├── runall.py              # ONE command: tests → simulations → benchmark → API self-test → deploy
├── run_simulation.py      # run a single scenario (headless or live window)
├── run_dashboard.py       # start only the API + dashboard
├── requirements.txt
├── src/autonav/
│   ├── config.py          # all parameters + colour palette
│   ├── world.py           # scenarios, static & moving obstacles, ray casting
│   ├── sensors.py         # simulated LiDAR
│   ├── perception.py      # mapping, detection, tracking, cost-map
│   ├── planner.py         # A* + path smoothing
│   ├── controller.py      # path following + safety layer
│   ├── robot.py           # unicycle kinematics
│   ├── simulator.py       # main loop, state machine, metrics
│   ├── render.py          # OpenCV visualisation
│   ├── headless.py        # saves PNG / GIF / plots / JSON
│   └── api.py             # Flask REST API + MJPEG stream
├── dashboard/index.html   # live mission-control UI
├── tests/test_autonav.py  # unit + integration tests
├── outputs/               # generated results (metrics, plots, GIFs)
├── images/                # screenshots used in this README
├── videos/                # screen recordings
└── docs/GUIDE.md          # full learning + GitHub + interview guide
```

## ⚙️ Installation
```bash
git clone https://github.com/<your-username>/AI-Autonomous-Navigation-System.git
cd AI-Autonomous-Navigation-System
python -m venv .venv
# Windows:      .venv\Scripts\activate
# macOS/Linux:  source .venv/bin/activate
pip install -r requirements.txt
```

## ▶️ How to run
```bash
python runall.py                 # full pipeline, then dashboard at http://127.0.0.1:5000
python runall.py --no-server     # pipeline only
python runall.py --fast          # skip GIF creation
python run_simulation.py --scenario dynamic --live    # watch one run in an OpenCV window
python run_dashboard.py --autostart                   # only API + dashboard
python -m unittest discover tests                     # tests only
```

## 🌐 REST API
| Method | Endpoint | Purpose |
|---|---|---|
| GET | `/` | Live dashboard |
| GET | `/video_feed` | MJPEG stream of the simulation |
| GET | `/api/status` | State, pose, speed, detections, metrics, event log, history |
| GET | `/api/metrics` | Mission metrics only |
| GET | `/api/config` | Scenarios + colour palette |
| GET | `/api/frame.png` | Single frame |
| POST | `/api/control` | `{"action":"start\|pause\|toggle\|reset\|step","scenario":"dynamic","speed":4}` |

```bash
curl http://127.0.0.1:5000/api/status
curl -X POST http://127.0.0.1:5000/api/control -H "Content-Type: application/json" -d '{"action":"start"}'
```

## 🎨 Colour language (dashboard + simulator)
| Colour | Meaning |
|---|---|
| 🟢 Green | Navigating, path clear |
| 🟡 Amber | Caution: obstacle ahead, slowing |
| 🟠 Orange | Actively steering around an obstacle |
| 🔵 Sky blue | Replanning a new route (planned path line is royal blue) |
| 🔴 Red | Emergency stop / danger / detections |
| 🟣 Purple | No path available |
| 🟪 Magenta | Moving obstacles and their predicted motion |
| 🟡 Gold | Goal |
| 🟥 Dark red | Collision (mission failed) |

## 🧪 Simulation workflow
1. Load a scenario (`easy`, `warehouse`, `dynamic`) and place the robot at the start.
2. Each 0.1 s tick: move obstacles → LiDAR scan → update occupancy map → track movers.
3. Build an inflated cost-map; replan with A\* if the current path is blocked.
4. Pure-pursuit steering; safety layer may slow, steer, or brake / back away.
5. Record telemetry; stop on goal, collision, no-route or timeout.

## 📊 Results
Benchmark of 10 random sensor-noise seeds per scenario (`python runall.py`):

| Scenario | Success rate | Avg time (s) | Avg distance (m) | Avg replans | Avg e-stops | Worst clearance (m) |
|---|---|---|---|---|---|---|
| easy | 100% | 31.1 | 67.0 | 8.2 | 5.2 | 0.13 |
| warehouse | 100% | 74.0 | 133.6 | 39.0 | 5.6 | 0.21 |
| dynamic | 100% | 31.0 | 68.2 | 12.4 | 0.4 | 0.29 |

> A larger 40-seed stress test of the `dynamic` scenario succeeded in 36/40 runs; the 4 failures were collisions with obstacles crossing the robot's path. This is a known limitation (constant-velocity prediction, no side-stepping) and the main target for future work.



## 🚀 Future improvements
YOLO camera perception · ROS 2 / Gazebo / CARLA port · SLAM instead of known pose · D\* Lite / RRT\* / MPC · reinforcement-learning local planner · multi-robot coordination · cloud dashboard (Docker + WebSockets) · drone (3D) version.

## 📚 Learning outcomes
Occupancy grids and log-odds mapping · A\* and heuristics · pure-pursuit control · state machines for safety behaviour · sensor noise · REST API design · testing simulation code · benchmarking with random seeds · Git/GitHub workflow.


## 📄 License
MIT
