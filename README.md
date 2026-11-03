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

## Build the ROS 2 workspace
```bash
scripts/build_ws.sh      # regenerates the URDF from sim/robot_params.py, then
                         # colcon build in ros_ws/ (clean shell: /opt/ros/jazzy +
                         # system python3, never the Isaac Sim venv)
source ./env.sh          # also sources ros_ws/install if it exists
```

## Run
```bash
scripts/run_sim.sh --seconds 600 [--start=-2,0,0]   # headless sim + ROS 2 bridge
```
`run_sim.sh` sources `env.sh`, starts `sim/run_sim.py` under `timeout` (hard cap
900 s), logs to `out/logs/run_sim.log`, prints `LAUNCH_READY` when the sim plays
and stops it cleanly on Ctrl-C/SIGTERM (no leftover Kit processes). The sim
publishes `/clock`, `/odom`, `/tf` (odom->base_link), `/tf_static`
(base_link->laser), `/joint_states`, `/scan` (360 beams, 1 deg, 10 Hz) and
subscribes `/cmd_vel`. It runs at real time and prints ground-truth `POSE` lines.

In a second shell (`source ./env.sh`):
```bash
ros2 launch warehouse_bringup description.launch.py   # robot_state_publisher (URDF)
ros2 launch warehouse_bringup slam.launch.py          # description + slam_toolbox -> /map
python3 scripts/cmd_vel_driver.py 0.5,0,2 0,1,1.5      # open loop: vx,wz,seconds segments
python3 scripts/waypoint_driver.py h:90 2,0 2,3       # closed loop on /odom: x,y or h:deg
scripts/map_warehouse.sh   # all-in-one: sim + SLAM + exploration loop -> out/maps/warehouse.{pgm,yaml}
```
Frames: `map` (slam_toolbox) -> `odom` (sim, starts at the spawn pose) ->
`base_link` (chassis centre, 0.11 m above floor) -> `base_footprint`, wheels,
`lift_link`, `laser`. Everything uses sim time (`use_sim_time:=true`).

## Checks
Each check prints `ok`/`FAIL` lines and `<name>: PASS|FAIL`; exit code 0 = PASS.
```bash
scripts/check_env.sh         # environment isolation + required packages (seconds)
scripts/smoke_clock.sh       # headless Isaac Sim publishes /clock on domain 21 (~1 min)
scripts/check_scene.sh       # builds out/warehouse.usd, boxes stay put (~30 s)
scripts/check_robot.sh       # robot drives 1 m, turns 90 deg, lift rises, no tilt (~30 s)
scripts/check_cmd_vel.sh     # ROS 2 /cmd_vel drives the sim robot (~40 s)
scripts/check_odom.sh        # /odom, TF, /joint_states >= 20 Hz and match ground truth (~50 s)
scripts/check_scan.sh        # /scan matches a 2D ground-truth raycast within 5 cm (~50 s)
scripts/check_launch.sh      # run_sim.sh: all topics, URDF TFs, teleop, clean shutdown (~1.5 min)
scripts/check_description.sh # colcon build + description launch static TFs (no sim, ~15 s)
scripts/check_slam.sh        # slam_toolbox /map + map->odom, map grows while driving (~2 min)
scripts/map_warehouse.sh     # full mapping run, coverage >= 80 % vs scene geometry (~3 min)
python3 -m pytest -q tests/  # pure-Python tests (no sim); run with system python3,
                             # not inside env.sh (the Isaac venv has no pytest)
```
Sim checks also require an `*_OK`/`SIM_READY` marker in their logs, because
`SimulationApp.close()` can end the process with exit code 0 before `sys.exit()`
runs. Logs go to `out/logs/`, maps to `out/maps/` (both git-ignored).

## Layout
| Path | Contents |
|---|---|
| `env.sh` | project environment |
| `sim/` | Isaac Sim Python (standalone, headless) |
| `sim/warehouse_layout.py` | room/shelf/box geometry (single source of truth) |
| `sim/scene.py` | builds the warehouse USD stage (primitive collisions) |
| `sim/robot_params.py` | robot geometry + diff-drive kinematics (pure Python) |
| `sim/robot.py` | builds the diff-drive robot articulation (wheels, casters, lift) |
| `sim/ros_graphs.py` | OmniGraph ROS 2 graphs: `/clock`, `/cmd_vel`, `/odom`+TF, `/joint_states`, `/scan` |
| `sim/sensors.py` | 2D lidar (PhysX raycast sensor, no RTX rendering) |
| `sim/run_sim.py` | headless sim runner (scene + robot + ROS 2 bridge) |
| `sim/geometry2d.py` | 2D ground truth (obstacle rectangles, exact raycasts) |
| `sim/urdf_gen.py` | generates the URDF from `robot_params.py` (`--check` for staleness) |
| `sim/map_eval.py` | compares a saved map with the scene geometry (coverage) |
| `scripts/` | launchers, drivers, probes and scripted checks (`lib.sh`: cleanup helpers) |
| `ros_ws/src/warehouse_bringup` | URDF, `description.launch.py`, `slam.launch.py`, slam_toolbox params |
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
- Day 5: `/odom` (~60 Hz), TF odom->base_link, `/joint_states` — done.
- Day 6: 2D lidar `/scan` (PhysX raycast, 360 beams) + static TF — done.
- Day 7: one-shot launcher `scripts/run_sim.sh` with clean shutdown — done.
- Day 8: `ros_ws` + `warehouse_bringup` (generated URDF, robot_state_publisher) — done.
- Day 9: slam_toolbox online async mapping — done.
- Day 10: scripted exploration + saved map, 100 % coverage of free area — done.
- Next: Day 11 map quality (IoU vs ground truth), Day 12 Nav2 + AMCL.

Notes: the headless sim runs faster than real time unless paced
(`run_sim.py` paces to real time by default); nodes must use `use_sim_time`.
`/odom` is ground truth of the chassis relative to the spawn pose (no wheel
slip), so in-place turns from open-loop `/cmd_vel` undershoot in the world
but odometry stays exact; use `waypoint_driver.py` for repeatable routes.
Kit leaves an `omni.telemetry.transmitter` process after shutdown; the scripts
stop it (`scripts/lib.sh`), together with the domain-21 ros2 daemon.
