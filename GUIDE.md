# AI-Based Autonomous Navigation System: Complete Student Guide

Order follows your brief: A (explanation) → P (troubleshooting). All code is already in this repo; section G maps each file to what it does.

---
## A. Project explanation

**Simple language.** Imagine a robot vacuum that has never seen your house. It must reach the kitchen without hitting chairs, and if someone walks in front of it, it must stop or go around. An AI-based autonomous navigation system is the "brain" that does this: it looks (sensors), understands (perception), chooses a route (planning), and steers (control), many times per second.

**Technical language.** A closed-loop autonomy stack: range sensing → occupancy mapping → obstacle detection/tracking → cost-map generation → global planning (graph search) → local control with a reactive safety layer → actuation, evaluated in simulation with metrics.

**Problem it solves.** Fixed routes fail when the world changes. Autonomy lets machines reach goals safely in unknown or changing spaces.

**Why it matters in industry.** Labour-intensive transport and inspection tasks, safety (machines that stop for people), 24/7 operation, consistency.

**Where used.** Warehouse AGVs/AMRs (Amazon, Ocado-style), self-driving cars (Waymo, Tesla, Mobileye stacks), sidewalk delivery robots, hospital/hotel robots, drones, mining and factory vehicles, floor-cleaning robots, smart-mobility shuttles.

**Workflow of a real system and where it lives in this repo**

| Stage | What it does | In this project |
|---|---|---|
| Perception | Turn raw sensor data into a world model | `sensors.py`, `perception.py` (log-odds map) |
| Computer vision | Image-style processing | OpenCV distance transform, connected components, cost-map |
| Object detection | Find discrete things | LiDAR clustering in `Perception.detect` (YOLO is the camera upgrade) |
| Lane/path detection | Find drivable space | Free-space in the occupancy grid + A* corridor (lane detection is a future camera feature) |
| Obstacle avoidance | Never collide | Safety layer in `controller.py` |
| Decision making | Pick behaviour | State machine: NAVIGATING / CAUTION / AVOIDING / REPLANNING / EMERGENCY_STOP / NO_PATH |
| Path planning | Route to goal | `planner.py` (A* + smoothing + clearance cost) |
| Control | Convert path to speed/steering | Pure pursuit in `controller.py`, kinematics in `robot.py` |
| Simulation/testing | Prove it works safely | `simulator.py`, `tests/`, benchmark in `runall.py` |

**Be honest about "AI".** This project uses classical robotics AI: search (A*), probabilistic mapping, tracking and prediction. It does **not** train a neural network. In interviews say exactly that, and mention YOLO/RL as planned upgrades.

---
## B. Tech-stack options

| | Option A: Easiest | Option B: Intermediate | Option C: Advanced |
|---|---|---|---|
| Tools | Python, NumPy, Matplotlib, grid + A* | Python, NumPy, OpenCV, custom 2D simulator, LiDAR model, Flask dashboard | ROS 2 + Gazebo/Webots or CARLA, YOLOv8, SLAM |
| Difficulty | Easy | Medium | Hard |
| GPU | No | No | Recommended (CARLA, YOLO) |
| Outcome | Animated path on a grid | Robot that senses, maps, replans, avoids moving obstacles, with API/dashboard | Near-industry stack |
| Time | 1-2 days | 5-7 days | 3-6 weeks |
| Best for fast GitHub proof | Too basic | **Best** | Too slow for most laptops |

## C. Selected approach: Option B
Chosen because it runs on any laptop, shows every stage of the autonomy pipeline, produces visual proof (GIFs, plots, dashboard), and every line is explainable in an interview. It is also a clean base to upgrade towards C.

---
## D. Architecture

```
 INPUT            PREPROCESS             PERCEPTION                 PLANNING            CONTROL        OUTPUT
 world (hidden) → LiDAR rays + noise → log-odds occupancy map  →  A* on inflated   →  pure pursuit  → v, w → robot moves
                                         obstacle clusters         cost-map            + safety layer
                                         mover tracking/prediction ↑ replan when path
                                         OpenCV cost-map             blocked / e-stop
```
**Data flow per 0.1 s tick:** `world.step` → `Lidar.scan` (ranges) → `Perception.update` (map, movers) → `detect` (obstacle list) → `costmap` (blocked grid) → `astar` if needed (path) → `Controller.command` (v, w, state) → `Robot.step` (new pose) → metrics/telemetry → API/dashboard.

**Modules:** world = ground truth the robot cannot read; sensors = only window onto the world; perception = understanding; planner = global route; controller = local behaviour and safety; simulator = glue + state machine; api/dashboard = observability.

---
## E. Folder structure
See README. Purpose of each folder: `src/autonav` code, `tests` automated checks, `outputs` generated evidence, `images` README screenshots, `videos` screen recordings, `docs` guides, `dashboard` web UI. Root scripts are entry points (`runall.py`, `run_simulation.py`, `run_dashboard.py`).

---
## F. Installation

Python 3.10 or newer (3.12 tested).

**Windows (PowerShell)**
```powershell
python --version
cd AI-Autonomous-Navigation-System
python -m venv .venv
.venv\Scripts\Activate.ps1          # if blocked: Set-ExecutionPolicy -Scope Process Bypass
pip install --upgrade pip
pip install -r requirements.txt
python -c "import numpy, cv2, flask, matplotlib, imageio; print('OK')"
pip freeze > requirements-lock.txt   # optional exact versions
```
**macOS / Linux**
```bash
python3 --version
cd AI-Autonomous-Navigation-System
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
python -c "import numpy, cv2, flask, matplotlib, imageio; print('OK')"
```
Generate requirements yourself later with `pip freeze > requirements.txt` (the shipped file lists only direct dependencies, which is cleaner).

---
## G. Full code, file by file

| File | What it does |
|---|---|
| `src/autonav/config.py` | Every tunable number (sizes, speeds, thresholds) and the colour palette. Change behaviour here. |
| `src/autonav/world.py` | Ground-truth environment: rectangles, patrolling circular obstacles, vectorised ray casting, collision check, 3 scenarios. |
| `src/autonav/sensors.py` | 90-beam, 10 m LiDAR with Gaussian noise. |
| `src/autonav/perception.py` | Log-odds occupancy mapping; LiDAR clustering into obstacles; **moving-obstacle detection** (cells that flip from free to occupied → track → velocity → 2.5 s prediction); OpenCV `distanceTransform` cost-map and inflation. |
| `src/autonav/planner.py` | 8-connected A* with octile heuristic, no corner cutting, clearance-aware cost, line-of-sight smoothing. |
| `src/autonav/controller.py` | Pure-pursuit follower + safety layer: proximity speed limit, caution slow-down, steering away, emergency stop / back-off / drive-away. |
| `src/autonav/robot.py` | Unicycle model with acceleration and braking limits. |
| `src/autonav/simulator.py` | The loop, replan logic, state machine, event log, metrics, JSON snapshot. |
| `src/autonav/render.py` | OpenCV drawing (colour-coded by state). |
| `src/autonav/headless.py` | Runs a scenario and saves PNG, GIF, telemetry plot, JSON. |
| `src/autonav/api.py` | Flask REST API, MJPEG stream, background simulation thread. |
| `dashboard/index.html` | Mission-control UI: state pill, KPIs, chart, detections, event log, controls. |
| `runall.py` | Whole pipeline + deployment. |
| `tests/test_autonav.py` | 11 tests (planner, perception, full runs). |

Open each file in VS Code and read the comments top to bottom; each starts with a docstring explaining its role.

**Upgrade hints:** swap `Lidar` for a camera + YOLO; replace A* with D* Lite; replace pure pursuit with MPC; replace "known pose" with SLAM.

---
## H. Virtual simulation workflow

**Simulator:** a custom 2D simulator written in NumPy/OpenCV. Why: zero installation pain, runs anywhere, fully transparent, fast enough to benchmark hundreds of runs. (Gazebo/CARLA are heavier; see section O.)

**Step by step**
1. **Install:** nothing beyond `pip install -r requirements.txt`.
2. **Launch:** `python run_simulation.py --scenario warehouse --live` or `python runall.py`.
3. **Map creation:** `make_world()` in `world.py` places rectangles (shelves, crates, blocks) on a 60×40 m grid.
4. **Robot agent:** `Robot` starts at the scenario start pose.
5. **Obstacles:** static rectangles + `DynamicObstacle` patrols (scenario `dynamic`).
6. **AI reads the environment:** only through `Lidar.scan`, which returns noisy distances; `Perception` converts them into a map.
7. **Path planning:** A* on the inflated cost-map; replans when new obstacles block the route.
8. **Collision avoidance:** speed limit by proximity, steering away, emergency stop/back-off, moving-obstacle prediction.
9. **Visualisation:** OpenCV view (grey = mapped obstacles, dim = undiscovered, red tint = safety margin, magenta = movers, blue = path, light green = trail, cyan = LiDAR).
10. **Final run:** the robot reaches the gold goal; a `GOAL_REACHED` state, metrics JSON, PNG and GIF are saved.

**Proof to capture:** see section M.

---
## I. How to run

```bash
python runall.py                         # everything, then dashboard
python runall.py --no-server             # everything, exits
python run_simulation.py --scenario dynamic          # one headless run
python run_simulation.py --scenario dynamic --live   # watch it
python run_dashboard.py --autostart                  # only API + dashboard
python -m unittest discover tests -v                 # tests
```
**Success looks like:** tests `OK`, each scenario ends `SUCCESS - Goal reached`, API self-test prints `all endpoints OK`, dashboard opens at http://127.0.0.1:5000.

**Sample terminal output**
```
STAGE 2: Unit tests   Ran 11 tests ... OK
[run] scenario=dynamic seed=0
  t=  10.0s state=AVOIDING       v= 0.2 goal_dist= 42.3m replans=10
  t=  30.0s state=NAVIGATING     v= 2.5 goal_dist= 10.1m replans=21
[done] SUCCESS - Goal reached | time=34.5s dist=71.7m replans=21 e-stops=2 min_clearance=0.29m
STAGE 5: API self-test   GET /api/health -> 200 ... all endpoints OK
```
(Numbers vary slightly by machine/library versions.)

**Files created:** `outputs/<scenario>_final.png`, `_obstacle_reaction.png`, `_telemetry.png`, `_demo.gif`, `_metrics.json`, `results.md`, `benchmark.json`, `benchmark_summary.png`, copies in `images/`.

**Dashboard walk-through:** press Start; the border and state pill change colour with the robot state (green clear → amber caution → orange avoiding → red emergency); the chart shows speed (green) and clearance (amber) with dashed red/amber threshold lines.

---
## J. GitHub upload steps

1. Create account at github.com; install Git (git-scm.com). One-time setup:
```bash
git config --global user.name  "Your Name"
git config --global user.email "you@example.com"
```
2. On GitHub: **New repository** → name `AI-Autonomous-Navigation-System` → description below → **Public** → do NOT add README/.gitignore (you already have them) → Create.
3. In the project folder:
```bash
git init
git add .
git commit -m "feat: project skeleton, config and scenarios"
git branch -M main
git remote add origin https://github.com/<your-username>/AI-Autonomous-Navigation-System.git
git push -u origin main
```
4. Later pushes: `git add . && git commit -m "message" && git push`.
5. Upload screenshots/GIF: they live in `images/` and `outputs/`; committing them pushes them. GIFs under 100 MB are fine (the ones here are 1-4 MB). For long videos, upload to YouTube/Drive and link in the README.
6. Browser alternative: GitHub → Add file → Upload files (fine for images only).

**Repository name:** `AI-Autonomous-Navigation-System`
**Description:** `Virtual autonomous robot with LiDAR mapping, A* planning, moving-obstacle prediction, safety controller, REST API and live dashboard (Python, OpenCV, Flask).`
**Topics:** `autonomous-navigation` `robotics` `path-planning` `a-star` `lidar` `occupancy-grid` `obstacle-avoidance` `computer-vision` `opencv` `simulation` `python` `flask` `dashboard` `autonomous-vehicles`
**Pin:** this repo on your profile (Profile → Customize your pins).
**Recruiter-friendly docs:** README top = GIF + 3-line summary; results table; architecture; `docs/GUIDE.md` for depth.
**Commit message style:** `type: what changed` (`feat:`, `fix:`, `docs:`, `test:`, `refactor:`).

---
## K. README.md
The finished README is in the repo root (`README.md`). Replace `<Your Name>`, links, add `images/dashboard.png`, and add a LICENSE name.

---
## L. Commit strategy (proof plan, 7 days)

> **Important:** commit as you really work, using real dates. Never backdate commits or fake history. Interviewers ask "walk me through your commits," and the code here is yours to understand, so spend the time reading each module.

| Day | Do | Commit message | Proof to save | LinkedIn/resume note |
|---|---|---|---|---|
| 1 | Setup, `config.py`, `world.py` (easy scenario), `.gitignore`, `requirements.txt` | `chore: project structure and environment` / `feat: world model and easy scenario` | terminal `pip list`, folder tree | "Designed virtual environment" |
| 2 | `robot.py`, `render.py`; draw the map | `feat: robot kinematics and OpenCV renderer` | `images/01_environment.png` | simulator built from scratch |
| 3 | `sensors.py`, `perception.py` (mapping, detection) | `feat: LiDAR model and occupancy mapping` | `02_obstacle_detection.png` | LiDAR + occupancy grid |
| 4 | `planner.py` + tests | `feat: A* planner with smoothing` / `test: planner tests` | `03_path_planning.png` | A* path planning |
| 5 | `controller.py`, safety layer | `feat: pure pursuit and reactive safety layer` | `04_navigation.png` | collision avoidance state machine |
| 6 | `simulator.py`, `headless.py`, scenarios, moving obstacles | `feat: full pipeline, warehouse and dynamic scenarios` | GIFs, telemetry plots | metrics + benchmarking |
| 7 | `api.py`, dashboard, `runall.py`, README | `feat: REST API and dashboard` / `docs: README and results` | dashboard screenshot, repo screenshot | deployable API + dashboard |

Because this repo is already complete, a good way to "build" it is to recreate it in this order in a fresh folder, typing/adapting each file and committing as you go. You will learn far more and be able to answer every question.

---
## M. Screenshots / proof checklist

| Asset | File name | Where | How |
|---|---|---|---|
| Terminal run | `01_terminal_runall.png` | `images/` | screenshot of `python runall.py` output |
| Environment | `02_environment.png` | `images/` | dashboard before pressing Start |
| Obstacle detection | `dynamic_obstacle_reaction.png` (auto) | `images/` | generated, shows red detections |
| Path planning | `03_path_planning.png` | `images/` | screenshot just after start (blue path) |
| Navigation output | `warehouse_telemetry.png` (auto) | `images/` | generated |
| Final success | `warehouse_final.png` (auto) | `images/` | generated |
| Dashboard | `dashboard.png` | `images/` | browser screenshot mid-run |
| Screen recording | `demo.mp4` or link | `videos/` | Win+G / QuickTime / OBS, 30-60 s |
| GitHub repo | `github_repo.png` | `images/` | screenshot of repo page |
| README preview | `readme_preview.png` | `images/` | scrolled README |

Reference in README: `![Final run](images/warehouse_final.png)`.

---
## N. Resume / LinkedIn / interview

**Resume bullets**
- Built a simulated autonomous navigation system in Python (NumPy, OpenCV) with noisy LiDAR sensing, log-odds occupancy mapping, A* planning and a pure-pursuit controller; reached the goal in 100% of 10-seed benchmark runs across 3 scenarios.
- Implemented moving-obstacle tracking with 2.5 s motion prediction and a reactive safety layer (speed limiting, steering, emergency stop), cutting collisions in the dynamic scenario from about 33% to 10% of randomized runs during development.
- Deployed a Flask REST API and real-time dashboard (state-coloured UI, telemetry charts, MJPEG stream) plus a one-command pipeline (`runall.py`) running tests, benchmarks and API self-tests.

**LinkedIn descriptions (pick one)**
1. *Short:* "Built an autonomous navigation simulator: the robot maps unknown environments with LiDAR, plans with A*, predicts moving obstacles and brakes safely. Includes a live dashboard and REST API. Python · OpenCV · Flask. GitHub: link"
2. *Story:* "I wanted to understand how warehouse robots avoid people, so I built the whole stack in simulation: perception, planning, control and monitoring. The hardest bug was a robot reversing into obstacles that approached from the side, which led me to add motion prediction and a smarter emergency policy."
3. *Recruiter style:* "Autonomous Navigation System | Robotics + AI | Skills: path planning, occupancy mapping, OpenCV, REST APIs, testing, benchmarking. Repo includes demo GIFs, tests and reproducible results."

**HR explanation:** "It's software for a robot that has to drive itself to a destination without hitting anything. I tested it in a computer simulation with moving obstacles, and it reaches the goal reliably. I also built a dashboard to watch what the robot is thinking."
**Technical explanation:** "2D simulator with a noisy LiDAR. I build a log-odds occupancy grid, inflate it with a distance transform, plan with 8-connected A* using a clearance-weighted cost, follow with pure pursuit, and wrap it in a reactive safety layer. Movers are detected as cells flipping free→occupied, tracked, and projected forward 2.5 s into the cost-map. Metrics are benchmarked over random seeds."
**Industry relevance:** "AGVs in warehouses, delivery robots and AV stacks use the same perception→planning→control loop, with heavier sensors and learned perception."

**10 interview questions and answers**
1. *Why A*?* It is optimal with an admissible heuristic and expands far fewer nodes than Dijkstra; the grid is small, so it replans in milliseconds.
2. *What heuristic and why admissible?* Octile distance; it never overestimates cost on an 8-connected grid.
3. *How does the robot know where obstacles are?* A simulated LiDAR gives ranges; I update a log-odds grid: free cells along each beam get negative evidence, hit cells get positive evidence.
4. *Why inflate obstacles?* So the planner can treat the robot as a point; inflation equals robot radius plus a safety margin (1.0 m), computed with OpenCV's distance transform.
5. *How do you handle moving obstacles?* Cells that flip from free to occupied are clustered, matched across ticks, velocity is filtered, and positions up to 2.5 s ahead are added to the cost-map so the planner avoids where they will be.
6. *What if the planner finds no path?* State `NO_PATH`: robot holds still but the safety layer stays active; retries every 0.5 s; fails after 20 s.
7. *Difference between global and local planning?* A* gives the global route; pure pursuit and the safety layer handle immediate behaviour at 10 Hz.
8. *What failed and how did you fix it?* Early on the robot reversed into side-approaching obstacles and hugged corners causing false stops. I added clearance-based path cost, a closing-speed trigger, and motion prediction. Dynamic success went from about 67% to 90% over 40 seeds.
9. *Limitations?* Perfect localisation is assumed, constant-velocity prediction, 2D only, no learned perception; 4/40 dynamic runs still collide.
10. *How would you extend it?* Camera + YOLO detection, SLAM, D* Lite/MPC, ROS 2, RL local planner, multi-robot.

---
## O. Future improvements and how to upgrade

| Idea | How |
|---|---|
| Real-time camera input | Add `camera.py` with `cv2.VideoCapture`; run YOLOv8n (`pip install ultralytics`) and convert boxes to obstacles |
| ROS 2 | Wrap `Simulation` in a node publishing `/scan`, `/cmd_vel`; planner as a service |
| CARLA | Replace `world/sensors` with CARLA actors/LiDAR; keep `planner/controller` |
| Lane detection | Canny + Hough on dashcam frames; lane centre as the path |
| Reinforcement learning | Gymnasium wrapper around `Simulation`; train PPO as a local planner |
| SLAM | Replace known pose with scan matching (ICP) and pose-graph optimisation |
| Multiple agents | List of robots; treat each other as movers; add priority rules |
| Cloud dashboard | Dockerise, deploy on Render/Fly.io, replace polling with WebSockets |
| IoT | Publish metrics to MQTT; Grafana dashboard |
| Warehouse robot | Task queue (pick/drop), fleet manager, one-way aisles |
| Drone version | 3D grid + 3D A*, altitude control |

---
## P. Troubleshooting

| Problem | Reason | Solution | Prevention |
|---|---|---|---|
| `python` not found (Windows) | Python not on PATH | Reinstall tick "Add to PATH" or use `py -3` | Verify with `python --version` |
| `pip install` fails / SSL | Network or old pip | `python -m pip install --upgrade pip`; retry | Use venv |
| `ModuleNotFoundError: cv2` | Wrong environment | Activate `.venv`; `pip install opencv-python` | Always activate venv first |
| numpy/opencv version conflict | Mixed packages | New venv; `pip install -r requirements.txt` | Don't install `opencv-python` and `opencv-contrib-python` together |
| PowerShell blocks activation | Execution policy | `Set-ExecutionPolicy -Scope Process Bypass` | Use Command Prompt `.venv\Scripts\activate.bat` |
| `--live` window does not open | Headless OpenCV / remote server | Use dashboard or headless mode; on Linux install `opencv-python` (not `-headless`) | Use dashboard by default |
| Dashboard blank / port in use | Another app on 5000 (macOS AirPlay) | `python runall.py --port 5055` | Pick a free port |
| Video stream frozen | Browser cache | Hard refresh (Ctrl+F5) | - |
| GIF too large | Many frames | `--fast` or lower the scale in `headless.py` | Keep demos <10 MB |
| Model not loading (when you add YOLO) | Weights not downloaded / no internet | Download `yolov8n.pt` manually next to the script | Cache weights in `models/` |
| Path planning returns None | Goal inside inflated zone or enclosed | Move the goal; lower `INFLATION_RADIUS` | Test scenarios with `tests/` |
| Robot jitters or stops near corners | Safety thresholds too strict | Tune `STOP_DIST`, `CORRIDOR_HALF_WIDTH` in `config.py` | Change one value at a time and run the benchmark |
| Collisions with moving obstacles | Prediction horizon too short / speed too high | Increase prediction horizon in `_track_movers`, lower `MAX_V` | Benchmark many seeds |
| `git push` rejected (non-fast-forward) | Remote has commits you lack | `git pull --rebase origin main` then push | Don't add README on GitHub when creating the repo |
| `Authentication failed` | GitHub removed password auth | Use a Personal Access Token or `gh auth login` | Set up GitHub CLI once |
| `remote origin already exists` | Added twice | `git remote set-url origin <url>` | Check `git remote -v` |
| Large file rejected (>100 MB) | GitHub limit | Remove the file, use Git LFS or external link | Keep videos out of git |
| Images not showing in README | Wrong relative path / case | Use `images/name.png` exactly, commit the image | Check on GitHub web view |
