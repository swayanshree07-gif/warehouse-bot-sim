# warehouse-bot-sim

A mobile warehouse robot in **Isaac Sim 6.2** (headless) that maps a small
warehouse with SLAM (slam_toolbox) over ROS 2 Jazzy, navigates with Nav2, and
moves boxes between shelves when it gets a task over a ROS 2 action. The 30-day
plan is in [PLAN.md](PLAN.md), the daily log in [PROGRESS.md](PROGRESS.md).

## Requirements (already on this machine)
- Isaac Sim 6.2 pip install at `~/omniverse/isaacsim-env` (used as-is, never modified)
- ROS 2 Jazzy at `/opt/ros/jazzy` (incl. slam_toolbox, nav2), Isaac Sim ROS
  workspace at `~/IsaacSim-ros_workspaces/jazzy_ws/install` (read-only)
- NVIDIA GPU (developed on an RTX 5050 Laptop, 8 GB VRAM, 16 GB RAM)

## Environment
```bash
source ./env.sh   # isolated shell: ROS_DOMAIN_ID=21, localhost-only discovery,
                  # Isaac Sim venv + Jazzy + bridge ws, other workspaces removed
```

## Checks
```bash
scripts/check_env.sh     # environment isolation + required packages (seconds)
scripts/smoke_clock.sh   # headless Isaac Sim publishes /clock on domain 21 (~2 min)
scripts/check_scene.sh   # builds out/warehouse.usd, simulates, boxes stay put (~30 s)
scripts/check_robot.sh   # robot drives 1 m, turns 90 deg, lift rises 0.2 m, no tilt (~30 s)
scripts/check_cmd_vel.sh # ROS 2 /cmd_vel drives the sim robot (~40 s)
python3 -m pytest -q tests/   # pure-Python tests (no sim); run with system python3,
                              # not inside env.sh (the Isaac venv has no pytest)
```
Each sim check prints `PASS`/`FAIL` and also requires an `*_OK` marker in its log,
because `SimulationApp.close()` can end the process with exit code 0 before
`sys.exit()` runs.

## Run the sim
```bash
source ./env.sh
timeout 900 python sim/run_sim.py --seconds 300 --start=-2,0,0   # x,y,yaw_deg
# in another shell (source ./env.sh): drive it
python3 scripts/cmd_vel_driver.py 0.5,0,2 0,1,1.5     # vx,wz,seconds segments
```
`run_sim.py` builds the warehouse + robot, publishes `/clock`, subscribes
`/cmd_vel` and prints ground-truth `POSE` lines. It runs at real time
(`--no-realtime` to run as fast as possible).
Sim logs go to `out/logs/` (git-ignored). Every sim run is wrapped in `timeout`.

## Layout
| Path | Contents |
|---|---|
| `env.sh` | project environment |
| `sim/` | Isaac Sim Python (standalone, headless) |
| `sim/warehouse_layout.py` | room/shelf/box geometry (single source of truth) |
| `sim/scene.py` | builds the warehouse USD stage (primitive collisions) |
| `sim/robot_params.py` | robot geometry + diff-drive kinematics (pure Python) |
| `sim/robot.py` | builds the diff-drive robot articulation (wheels, casters, lift) |
| `sim/ros_graphs.py` | OmniGraph ROS 2 graphs: `/clock`, `/cmd_vel` -> wheels |
| `sim/run_sim.py` | headless sim runner (scene + robot + ROS 2 bridge) |
| `scripts/` | launchers and scripted checks (`lib.sh`: leftover-process cleanup) |
| `tests/` | pytest tests that run without Isaac Sim |
| `ros_ws/src/` | own ROS 2 packages (colcon) |
| `out/` | logs, maps, bags, media (git-ignored, keep < 4 GB) |

## Status
- Day 1: plan, isolated environment (`env.sh`, domain 21), headless ROS 2
  bridge smoke test publishing `/clock` — done.
- Day 2: warehouse scene — 10 m x 8 m room, 4 walls, 4 solid racks with 3
  slots each, 10 boxes — builds, saves and simulates stably — done.
- Day 3: own diff-drive robot (0.5 x 0.36 m chassis, 2 driven wheels with
  sphere colliders, 2 frictionless casters, prismatic lift plate) — done.
- Day 4: `/cmd_vel` -> OmniGraph DifferentialController -> wheel drives — done.
- Next: `/odom`, TF and joint states (Day 5), 2D lidar `/scan` (Day 6).

Notes: the headless sim runs faster than real time unless paced
(`run_sim.py` paces to real time by default); nodes must use `use_sim_time`. Kit leaves an `omni.telemetry.transmitter` process after
shutdown; the scripts stop it (`scripts/lib.sh`).
