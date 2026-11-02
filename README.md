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
scripts/check_scene.sh   # builds out/warehouse.usd, simulates, boxes stay put (~1 min)
python3 -m pytest -q tests/   # pure-Python layout tests (no sim)
```
Sim logs go to `out/logs/` (git-ignored). Every sim run is wrapped in `timeout`.

## Layout
| Path | Contents |
|---|---|
| `env.sh` | project environment |
| `sim/` | Isaac Sim Python (standalone, headless) |
| `sim/warehouse_layout.py` | room/shelf/box geometry (single source of truth) |
| `sim/scene.py` | builds the warehouse USD stage (primitive collisions) |
| `scripts/` | launchers and scripted checks (`lib.sh`: leftover-process cleanup) |
| `tests/` | pytest tests that run without Isaac Sim |
| `ros_ws/src/` | own ROS 2 packages (colcon) |
| `out/` | logs, maps, bags, media (git-ignored, keep < 4 GB) |

## Status
- Day 1: plan, isolated environment (`env.sh`, domain 21), headless ROS 2
  bridge smoke test publishing `/clock` — done.
- Day 2 (started early): warehouse scene — 10 m x 8 m room, 4 walls, 4 solid
  racks with 3 slots each, 10 boxes — builds, saves and simulates stably.
- Next: own differential-drive robot (Day 3), `/cmd_vel` bridge (Day 4).

Notes: the headless sim runs faster than real time (~150 Hz /clock); nodes must
use `use_sim_time`. Kit leaves an `omni.telemetry.transmitter` process after
shutdown; the scripts stop it (`scripts/lib.sh`).
