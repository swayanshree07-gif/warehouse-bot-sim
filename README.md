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
Quick start: everything with one command (sim + Nav2 + task manager):
```bash
scripts/launch_all.sh [--slam] [--seconds 600] [--record [DIR]] [--media [DIR]]
# wait for ALL_READY, then in a second shell:
source ./env.sh && python3 scripts/move_box.py box_09 shelf_D
```
Fleet mode (Day 26), 3 robots + fleet manager in Isaac with one command:
```bash
scripts/launch_all.sh --fleet [--seconds 600] [--tasks] [--record [DIR]] [--media [DIR]]
# FLEET_READY, then send tasks:  python3 scripts/fleet_client.py "box_00 shelf_D" ...
```
It runs `scripts/launch_fleet.sh`: fleet sim, per-robot AMCL, `fleet_isaac.py`,
optional bag of `/robotN/*`, `/fleet/*`, `/boxes`, `/tf`, `/map` (about 55 MB for
10 min) and overview + robot1 chase mp4s. `--tasks` sends the 6 demo tasks and
stops when they are done. Logs: `out/logs/launch_fleet_*.log`.

`launch_all.sh` starts `run_sim.sh` (spawn at world (-2, 0) = map origin), Nav2
(AMCL on the packaged map, or slam_toolbox from an empty map with `--slam`) and
`task_manager` (`backend:=ros`). Then it runs `scripts/health_check.py` (topics
`/clock /odom /scan /joint_states /lift/height /boxes /map` have messages, TF
map/odom->base_link, Nav2 lifecycle nodes `active`, gripper services, `/move_box`
action server) and prints `ALL_READY`. `--record` also records a filtered rosbag
(mcap + zstd, about 2 MB per minute) into `out/bags/run_<date>_<time>/` or DIR. The
stack stops when the sim run time ends, when any part exits, or on Ctrl-C/SIGTERM.
The recorder is closed first, then everything else is stopped, and
`ALL_STOPPED` is printed. Logs: `out/logs/launch_all_*.log`.
`--media` captures camera frames from the headless sim (see below) into
`out/media/run_<date>_<time>/` or DIR, and after the stop builds
`overview.mp4`, `chase.mp4` and a side-by-side `demo.mp4` from them.

Screenshots/video (`sim/media.py`, Day 26): `run_sim.py --capture DIR
[--capture-every 0.5] [--capture-res 480x270]` adds two cameras, a fixed
"overview" over the room and a "chase" camera behind the robot. Each one gets a
low-res replicator render product with an `rgb` annotator. A PNG is written to
`DIR/<camera>/frame_NNNN.png` every 0.5 sim-s (about 45 KB per frame, so
about 11 MB per minute for both cameras). Without `--capture` nothing is
rendered. `scripts/make_video.sh DIR [FPS]` turns the frames into H.264 mp4s
with ffmpeg (10 fps by default, so 5x speed). The scene light is 1200 (was
3000) so the floor is not overexposed.

The parts can also be started one by one:
```bash
scripts/run_sim.sh --seconds 600 [--start=-2,0,0]   # headless sim + ROS 2 bridge
```
`run_sim.sh` sources `env.sh`, starts `sim/run_sim.py` under `timeout` (hard cap
900 s), logs to `out/logs/run_sim.log`, prints `LAUNCH_READY` when the sim plays
and stops it cleanly on Ctrl-C/SIGTERM (no leftover Kit processes). The sim
publishes `/clock`, `/odom`, `/tf` (odom->base_link), `/tf_static`
(base_link->laser), `/joint_states`, `/scan` (360 beams, 1 deg, 10 Hz) and
subscribes `/cmd_vel`. It runs at real time and prints ground-truth `POSE` lines.
Box handling (`sim/box_handler.py`, an rclpy node inside the sim process):
`/lift/command` (std_msgs/Float64, m, 0–0.30) and `/lift/height`,
`/gripper/attach` / `/gripper/detach` (std_srvs/Trigger) and `/gripper/held`
(std_msgs/String). The gripper is a fork-style virtual gripper: attach
creates a fixed joint between the lift carriage and the box whose centre is
within 0.10 m of the grip point 0.50 m ahead of base_link, at the current
relative pose. Raising the lift 3 cm takes the box off the rack. The sim also
prints `BOXES` ground-truth lines.

In a second shell (`source ./env.sh`):
```bash
ros2 launch warehouse_bringup description.launch.py   # robot_state_publisher (URDF)
ros2 launch warehouse_bringup slam.launch.py          # description + slam_toolbox -> /map
python3 scripts/cmd_vel_driver.py 0.5,0,2 0,1,1.5      # open loop: vx,wz,seconds segments
python3 scripts/waypoint_driver.py h:90 2,0 2,3       # closed loop on /odom: x,y or h:deg
scripts/map_warehouse.sh   # all-in-one: sim + SLAM + exploration loop -> out/maps/warehouse.{pgm,yaml}
ros2 launch warehouse_bringup nav2.launch.py           # Nav2 + AMCL on the packaged map
ros2 launch warehouse_bringup nav2.launch.py slam:=true   # Nav2 on slam_toolbox's live map
python3 scripts/nav_probe.py --sim-log out/logs/run_sim.log --goal 3.5,0,0   # NavigateToPose (map frame)
python3 scripts/gripper_cmd.py lift 0.03               # lift to 3 cm, waits until within 1 cm
python3 scripts/gripper_cmd.py attach                  # / detach (Trigger services)
```
Nav2 (`config/nav2.yaml`): map_server + AMCL (starts at the spawn pose), NavFn
(A*) planner, Regulated Pure Pursuit controller (0.4 m/s, publishes `Twist` on
`/cmd_vel` directly), behaviors, bt_navigator, waypoint_follower; footprint
from `robot_params.footprint()`. The packaged map
`ros_ws/src/warehouse_bringup/maps/warehouse.{pgm,yaml}` is a copy of the
Day 10 `map_warehouse.sh` output; the map frame origin is the spawn pose
world (-2, 0, 0), so map = world + (2, 0). Run the sim with `--start=-2,0,0`.
Shelf/box registry: `ros_ws/src/warehouse_bringup/config/registry.yaml`
is generated by `python3 sim/registry_gen.py` (`--check` reports a stale file).
It lists every shelf and slot with its centre, a Nav2 `approach` pose (0.9 m
from the slot) and a `dock` pose for the gripper (0.5 m), all in the map frame,
plus the starting box placement. The `warehouse_interfaces` package defines
`MoveBox.action` (box_id, target_shelf, target_slot) and the `BoxState` /
`BoxStates` messages.
Task manager (`ros_ws/src/warehouse_tasks`): the `task_manager` node serves
the `MoveBox` action on `/move_box`. Each goal is checked against the
registry and the node's own record of which box is on which slot (unknown
box/shelf, occupied slot, no free slot → aborted with the reason). Goals run
one at a time in arrival order. The steps are NAV_TO_PICK (Nav2 to the slot's
approach pose) → DOCK_PICK (lidar dock, see below) → LIFT (attach + lift 3 cm) → UNDOCK_PICK (reverse to the approach pose) →
NAV_TO_PLACE → DOCK_PLACE → LOWER (lift 0 + detach) → RETREAT. If a step fails
at a rack and no box is held, the robot backs out to the approach pose first.
`backend:=mock` runs the same state machine without the sim (for tests).

Docking (`rack_align.py` + `ros_robot.py`): at the approach pose the robot faces
the rack and fits the rack's front face in `/scan` (line fit seeded by the
beams straight ahead, both rack corners confirmed by the next beam outward).
Together with the slot's offset from the rack centre, that gives the dock pose
relative to the robot, independent of AMCL. It becomes an odom-frame target. If
the lateral error is above 2 cm, the robot first reverses 0.35 m along the
corrected line, then line-tracks forward to the dock point and turns to its
yaw. If the rack is not recognised, it falls back to the AMCL-only dock on TF
map->base_link (also available with `lidar_dock:=false`).
```bash
ros2 launch warehouse_bringup nav2.launch.py                       # with the sim running
ros2 launch warehouse_tasks task_manager.launch.py backend:=ros    # or backend:=mock (no sim)
python3 scripts/move_box.py box_09 shelf_D [slot]                  # prints feedback + MOVE_BOX_RESULT
```
Frames: `map` (slam_toolbox) -> `odom` (sim, starts at the spawn pose) ->
`base_link` (chassis centre, 0.11 m above floor) -> `base_footprint`, wheels,
`lift_link`, `laser`. Everything uses sim time (`use_sim_time:=true`).

## Checks
Each check prints `ok`/`FAIL` lines and `<name>: PASS|FAIL`; exit code 0 = PASS.
```bash
scripts/lint.sh              # flake8 (.flake8 config) + bash -n on all scripts (seconds)
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
scripts/map_warehouse.sh     # full mapping run, coverage >= 80 % + IoU >= 0.7 vs scene geometry (~3 min)
scripts/check_map.sh [map.yaml]  # map quality without the sim: coverage + occupied IoU (seconds)
scripts/check_nav.sh         # Nav2 + AMCL: nodes active, AMCL < 0.2 m, 3 goals < 0.25 m (~2 min)
scripts/check_nav.sh --slam  # same with slam_toolbox instead of map + AMCL, map must grow (~2 min)
scripts/check_gripper.sh     # lift within 1 cm, attach/carry/place box_09 to a free slot (~1.5 min)
scripts/check_interfaces.sh  # warehouse_interfaces builds, interface show, Python types (no sim, ~15 s)
scripts/check_task_manager.sh  # MoveBox action server, mock backend: success, invalid goals, queue, cancel, failure (no sim, ~30 s)
scripts/check_move_box.sh [BOX:SHELF[:SLOT] ...]  # end-to-end in the sim: Nav2 + task_manager, box on target slot (~2-3 min)
scripts/check_dock.sh [--slot shelf_X:N ...] [--no-lidar]  # 5 docking trials vs ground truth, each < 3 cm / 3 deg (~3.5 min)
scripts/check_task_queue.sh  # in the sim: /boxes ground truth, 4 queued goals (2 succeed, missing box + occupied slot abort) (~4 min)
scripts/check_launch_all.sh [--slam]  # launch_all.sh --record --media: ALL_READY, health check, 1 MoveBox, clean SIGTERM stop, bag + media valid (~3.5 min)
scripts/check_launch_all.sh --supervise  # sim ends by itself after 45 s -> whole stack stops, no leftovers (~1.5 min)
python3 scripts/health_check.py [--slam]  # health of a running stack (HEALTH_PASS / HEALTH_FAIL)
python3 scripts/check_bag.py out/bags/<dir>  # bag < 300 MB with all key topics (no ROS needed)
scripts/check_media.sh       # sim camera capture while driving -> PNGs + mp4s, not blank, robot moved (~1 min)
python3 scripts/check_media.py out/media/<dir> [--need-video]  # validate captured frames/videos
scripts/check_fleet_boxes.sh # Isaac fleet: 3 robots each attach, lift and carry a different box; /boxes holder ids
scripts/check_fleet_nav.sh   # Isaac + AMCL x3 + traffic-manager follower: 3 robots reach crossing goals at once (~2 min)
scripts/check_fleet_amcl.sh  # Isaac + AMCL x3 on saved map: each /robotN/amcl_pose within 0.2 m of ground truth (~1.5 min)
scripts/check_fleet_sim.sh   # Isaac, 3 namespaced robots: 9 /robotN/{cmd_vel,odom,scan} topics, robotN/* TF, /robot2/cmd_vel moves only robot2, VRAM < 7 GB (~1 min)
scripts/check_fleet_traffic.sh   # Isaac: head-on swap + crossing with reservations, GT min sep >= 0.45 m (~2.5 min)
scripts/check_launch_fleet.sh    # launch_all.sh --fleet --tasks --record --media: 6 tasks, no leftovers, bag < 300 MB with /robotN topics, media valid (~14 min)
scripts/check_fleet_isaac.sh     # Isaac fleet scenario: 3 robots, 6 chained pick/drop tasks over /fleet/tasks, boxes at targets, metrics JSON (~8 min)
scripts/check_fleet_manager.sh   # fleet_manager ROS 2 node (light backend): 6 tasks over /fleet/tasks all done, /fleet/status assignments (~30 s)
scripts/check_fleet_scenario.sh  # light sim 3 robots / 9 tasks + metrics floor (no GPU, seconds)
scripts/check_fleet_robustness.sh # Day 27: 3x light-sim scenario + 3x FleetExecutor on kinematic robots (shelf lock, 6/6 tasks), no GPU (seconds)
scripts/check_robustness.sh [N] [--slam]  # check_launch_all N times (default 3) + VRAM/RAM peaks (~4 min/run)
python3 -m pytest -q tests/  # pure-Python tests (no sim); run with system python3,
                             # not inside env.sh (the Isaac venv has no pytest)
```
Sim checks also require an `*_OK`/`SIM_READY` marker in their logs, because
`SimulationApp.close()` can end the process with exit code 0 before `sys.exit()`
runs. Logs go to `out/logs/`, maps to `out/maps/` (both git-ignored).

## Architecture
```
                       ROS 2 Jazzy, ROS_DOMAIN_ID=21, localhost only
 ┌──────────────── Isaac Sim 6.2 (headless, sim/run_fleet_sim.py) ───────────────┐
 │ warehouse scene (4 racks, 10 boxes)   robot1..3: diff drive + lift + gripper  │
 │ OmniGraph: /clock /robotN/{cmd_vel,odom,scan,joint_states}  TF robotN/odom->… │
 │ box_handler: /robotN/lift/*, /robotN/gripper/{attach,detach}, /boxes          │
 │ media.py: overview + chase cameras -> PNG -> mp4                              │
 └───────────────▲───────────────────────────────┬───────────────────────────────┘
        cmd_vel, │ gripper/lift                  │ scan, odom, TF, /boxes
 ┌───────────────┴───────────┐   ┌───────────────▼──────────────────────────────┐
 │ scripts/fleet_isaac.py    │◄──┤ fleet_localization.launch.py                 │
 │  └ fleet_exec.FleetExecutor│   │ map_server (/map) + per robot RSP + AMCL     │
 │     ├ fleet.py  nearest-free assignment, FIFO, box/slot claims              │
 │     ├ traffic.py grid roadmap, A*, cell reservations, deadlock yield        │
 │     ├ rack_align.py lidar rack-face docking                                 │
 │     └ metrics.py tasks/h, idle %, traffic wait                              │
 └───────▲───────────────┬───┘   └──────────────────────────────────────────────┘
 /fleet/tasks (String     │ /fleet/status (JSON)
 "box_id shelf_id")       ▼
   scripts/fleet_client.py            rosbag (mcap) of /robotN/*, /fleet/*, /boxes, /tf, /map
```
Single-robot mode (`launch_all.sh` without `--fleet`) uses `sim/run_sim.py`,
Nav2 + AMCL or slam_toolbox, and the `task_manager` MoveBox action server.
The light 2D simulator (`warehouse_tasks/lightsim.py`) runs the same fleet
logic without a GPU for fast tests.

## Fresh-shell walkthrough
```bash
cd ~/thirty-days/projects/warehouse-bot-sim
scripts/check_env.sh                       # env sanity (domain 21, paths)
scripts/build_ws.sh && source ./env.sh     # build ros_ws, load environment
/usr/bin/python3 -m pytest -q tests        # ~170 unit tests, no GPU, ~2 s
scripts/lint.sh                            # flake8 + bash -n
timeout 300 scripts/check_fleet_robustness.sh   # fleet logic, light sim, 3 runs
timeout 1000 scripts/check_fleet_isaac.sh       # 3 robots, 6 tasks in Isaac (~12 min)
scripts/launch_all.sh --fleet --tasks --record --media   # demo: bag + mp4
```

## Troubleshooting
| Symptom | Cause / fix |
|---|---|
| Nodes from other projects show up in `ros2 topic list` | shell not isolated: open a new shell and `source ./env.sh` (domain 21, localhost) |
| `colcon build` picks up the Isaac venv python | use `scripts/build_ws.sh` (clean shell with `/opt/ros/jazzy` + system python) |
| Sim seems hung after a run / VRAM still used | `pkill -f run_fleet_sim`; `scripts/lib.sh` cleanup also stops `omni.telemetry.transmitter` and the ros2 daemon; check `nvidia-smi` |
| Robots in Isaac stop moving together | lockstep traffic ticks; see `settled` / `lock_waits` in `out/fleet_isaac_report.json` and the fm log in `out/logs/` |
| Pick or drop fails at a shelf | docking error > grip tolerance (0.10 m); the robot re-docks once with a fresh lidar measure (Day 29), drops re-dock up to 2 times |
| AMCL pose far off | robot spawned away from `sim/fleet_layout.py` starts; restart, or check `scripts/check_fleet_amcl.sh` |
| Out of GPU memory | keep headless, capture res 480x270; fleet peak is ~1.2 GB VRAM |
| `out/` grows large | bags/media are git-ignored; delete old `out/bags/run_*` (keep < 4 GB) |

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
| `sim/registry_gen.py` | generates the shelf/box registry YAML (slot centres, approach/dock poses) |
| `sim/box_handler.py` | in-sim rclpy node: lift command, virtual gripper attach/detach, `/boxes` ground truth, `/sim/remove_box` test hook |
| `sim/box_state.py` | box -> shelf slot classification for `/boxes` (pure Python) |
| `sim/media.py` | overview + chase cameras, replicator `rgb` annotators -> PNG frames |
| `sim/map_eval.py` | compares a saved map with the scene geometry (coverage, occupied-cell IoU) |
| `scripts/` | launchers, drivers, probes and scripted checks (`lib.sh`: cleanup helpers) |
| `ros_ws/src/warehouse_bringup` | URDF, `description`/`slam`/`nav2` launch files, slam_toolbox + Nav2 params, saved map, registry |
| `ros_ws/src/warehouse_interfaces` | `MoveBox.action`, `BoxState.msg`, `BoxStates.msg` |
| `ros_ws/src/warehouse_tasks` | `task_manager` node: MoveBox action server, pick/place state machine (`mission.py`), registry belief, docking law (`line_track.py`), lidar rack-face measurement (`rack_align.py`), mock + ROS backends |
| `sim/run_fleet_sim.py`, `sim/fleet_layout.py`, `sim/fleet_boxes.py` | 3-robot Isaac scene, spawn poses, per-robot box handling |
| `ros_ws/src/warehouse_tasks` (fleet) | `fleet.py`, `traffic.py`, `fleet_exec.py`, `fleet_manager.py`, `lightsim.py`, `metrics.py` |
| `scripts/fleet_isaac.py`, `launch_fleet.sh` | fleet manager Isaac backend, fleet launcher |
| `tests/` | pytest tests that run without Isaac Sim |
| `ros_ws/src/` | own ROS 2 packages (colcon) |
| `out/` | logs, maps, bags, media (git-ignored, keep < 4 GB) |

## Status
- Last re-verified 2026-11-08 (session Day 14): `scripts/check_nav.sh --slam`
  PASS (Nav2 on slam_toolbox's live map, 3-goal tour, errors <= 0.15 m, map
  grew 3.9 -> 74.1 m^2). Day 13: `scripts/check_nav.sh` (AMCL) PASS.
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
- Day 11: map quality check — occupied-cell IoU 0.915 (2-cell tolerance) — done.
- Day 12: Nav2 + AMCL on the saved map, all lifecycle nodes active — done.
- Day 13: NavigateToPose via nav2_simple_commander, goals reached within ~0.15 m — done.
- Day 14: Nav2 while mapping with slam_toolbox (from an empty map) — done.
- Day 15: shelf/box registry YAML (generated, validated by pytest) — done.
- Day 16: lift command over ROS 2 (`/lift/command`, within 1 cm) — done.
- Day 17: box attach/detach (fixed joint), carry and place on a free slot — done.
- Day 18: `warehouse_interfaces` (MoveBox action, BoxState(s) msgs) — done.
- Day 19: `task_manager` MoveBox action server + state machine, mocked steps tested — done.
- Day 20: precise docking with a lidar rack-face measurement: 10/10 trials within
  3 cm / 3 deg (worst 1.0 cm, 1.4 deg); the AMCL-only dock managed 2/5 — done.
- Day 21: end-to-end `MoveBox` via the action in the sim: boxes placed 1.3 cm
  from the slot centre — done.
- Day 22: queued tasks (FIFO tickets) with failure handling: missing box and
  occupied slot abort with a reason, the other goals still succeed — done
  (an unreachable Nav2 goal is only tested with the mock backend).
- Day 23: ground-truth `/boxes` (BoxStates, ~10 Hz, map frame) from the sim;
  the task manager syncs its belief before each goal and reports the real
  `place_error` — done.
- Day 24: single launch script `scripts/launch_all.sh` + `health_check.py`
  (19 items), supervision and clean shutdown; AMCL and SLAM modes — done.
- Day 25: rosbag recording (`launch_all.sh --record`, 15 topics incl. the
  MoveBox action status/feedback, ~2.6 MB for a 75 s task) — done.
- Day 26: headless camera capture (overview + chase, 480x270) -> PNG frames
  -> mp4 (`launch_all.sh --media`, `make_video.sh`); a full MoveBox run gives
  149 frames per camera and a 15 s side-by-side `demo.mp4` — done.
- Day 27: `check_robustness.sh`: 3/3 full runs in a row passed (sim + Nav2 +
  MoveBox + bag + media, ~127 s each), peak VRAM 1.6 GB, peak system RAM
  7.8 GB — done (the plan also allows fixing flakiness here; none showed up).
- 2026-11-08 re-check: the Day 11 map check still passes with a freshly built
  SLAM map (coverage 1.000, IoU 1.000).
- 2026-11-08 (session Day 15): the original single-robot plan is complete.
  Days 15–27 were re-planned for a 3-robot fleet (see PLAN.md). Fleet core
  `warehouse_tasks/fleet.py` (nearest-free-robot assignment, FIFO queue,
  box/slot claims) is done and tested (`tests/test_fleet.py`).
  Traffic manager `warehouse_tasks/traffic.py` (Day 16) is done: 0.5 m grid
  roadmap from the saved map, A*, per-tick cell reservations by priority (no
  shared cells, no swaps), wait-for-graph deadlock detection, priority yield
  to a refuge cell, parked-robot push. Tested in `tests/test_traffic.py`.
  Light 2D fleet simulator `warehouse_tasks/lightsim.py` (Day 17) is done:
  kinematic robots on the traffic grid (0.5 m/s), timed pick/drop, a 3-slot
  floor drop zone `zone_E` (light sim only). Run the 3-robot, 9-task scenario
  with `timeout 120 python3 scripts/run_lightsim.py -v` (JSON report in
  `out/fleet/lightsim.json`; exits 0 only if 9/9 done, no collisions, min
  separation >= 0.5 m). Tested in `tests/test_lightsim.py`.
  Throughput metrics `warehouse_tasks/metrics.py` (Day 18) are done:
  tasks/hour, mean/max task time, queue wait, per-robot idle % and traffic
  wait. Headless scenario check: `scripts/check_fleet_scenario.sh` (report in
  `out/fleet/scenario_report.json`; floor `MIN_TPH`, default 200/h; current
  337.5/h). Tested in `tests/test_metrics.py`.
  Isaac fleet scene (Day 19) is done: `sim/run_fleet_sim.py` spawns
  robot1..3 (`sim/fleet_layout.py`, central aisle x=-3/0/3) with topics
  `/robotN/cmd_vel|odom|scan|joint_states`, TF frames `robotN/odom ->
  robotN/base_link -> robotN/laser` on the shared `/tf`, one `/clock`.
  Peak GPU memory ~1.2 GB. Check: `scripts/check_fleet_sim.sh`.
  Fleet localization (Day 20) is done: `ros2 launch warehouse_bringup
  fleet_localization.launch.py` runs one shared map_server (`/map`) and, per
  namespace, robot_state_publisher (frame_prefix `robotN/`) + AMCL
  (`/robotN/amcl_pose`, TF `map -> robotN/odom`). Map frame = world + 2 m in x.
  Check: `scripts/check_fleet_amcl.sh` (all 3 AMCL poses within 0.2 m of
  ground truth, also after driving robot2/robot3).
  Fleet navigation (Day 21) is done, without Nav2: the traffic manager
  (`warehouse_tasks/path_follower.py`) reserves grid cells, and a small
  per-robot controller drives `/robotN/cmd_vel` from TF `map -> robotN/base_footprint`.
  Run it with `scripts/fleet_follow.py robot1=x,y robot2=x,y robot3=x,y` (map frame).
  Check: `scripts/check_fleet_nav.sh` (3 crossing goals reached at once
  within 0.25 m, peak RAM ~7.4 GB, VRAM ~1.2 GB).
  Per-robot box handling (Day 22) is done: in the fleet sim each robot has
  `/robotN/lift/command`, `/robotN/lift/height`, `/robotN/gripper/{attach,detach,held}`
  (`scripts/gripper_cmd.py ... --ns robotN`), and the one global `/boxes`
  reports `holder` (robot namespace) for each held box. A robot never grabs a box
  another robot holds. `run_fleet_sim.py --starts robotN=x,y,yaw` overrides spawn poses.
  Check: `scripts/check_fleet_boxes.sh` (3 robots lift and carry 3 different
  boxes at once, ~1.5 min).
- Day 23: `fleet_manager` ROS 2 node (`ros2 launch warehouse_tasks
  fleet_manager.launch.py`). Send tasks as `std_msgs/String` "box_id shelf_id"
  on `/fleet/tasks`; JSON state (robots, assignments, task states, metrics)
  on `/fleet/status`. Backend: light sim stepped in real time. Client:
  `scripts/fleet_client.py`. Check: `scripts/check_fleet_manager.sh`.
- Day 24: cell reservations gate robots in Isaac. robot1 and robot3 swap
  places head-on while robot2 crosses. The deadlock is resolved by priority
  yield. Ground-truth min separation was 0.51 m. Check:
  `scripts/check_fleet_traffic.sh` (~2.5 min; evaluator `scripts/eval_fleet_traffic.py`).
- Day 25: fleet manager Isaac backend `scripts/fleet_isaac.py` (same
  `/fleet/tasks` + `/fleet/status` interface; logic in
  `warehouse_tasks/fleet_exec.py`). Nearest-free assignment, lockstep cell
  reservations, then per robot: approach pose -> face rack -> lidar rack-face
  dock (AMCL alone is ~0.1 m off) -> gripper via `gripper_cmd.py`. Before a
  drop, the held box's `/boxes` position is compared with the slot centre and
  the robot re-docks once to cancel the pick offset. A docked robot also
  reserves the neighbour cell it overhangs ("ghost" cell). Report:
  `out/fleet_isaac_report.json`. Check: `scripts/check_fleet_isaac.sh`.
- Day 26: idle robots park off the shelf fronts after each task, a stall
  detector asks idle robots to step aside, and drops re-dock up to 2 times.
  `launch_all.sh --fleet` (sim + AMCL x3 + fleet manager + bag + video) works
  end to end: bag 55 MB with all `/robotN/*` topics, overview/chase/demo mp4.
- Day 27: shelf lock (one robot at a time travels to, docks at or undocks
  from a shelf; others wait off the shelf fronts; `lock_waits` in the report)
  and lidar re-measure on each drop re-dock (fresh rack measurement minus the
  accumulated held-box offset). Fleet robustness without the GPU:
  `scripts/check_fleet_robustness.sh` (3/3 PASS).
- Status: the Isaac fleet scenario is still not reliable. Day 27 run 2: 1/6
  tasks, two robots froze in `to_drop` (probably a shelf lock that was never
  released; fixed, but the fix is not yet checked in Isaac). Min separation
  0.507 m, peak VRAM 1.2 GB.
- Day 28: lint is clean (`scripts/lint.sh`, flake8 config in `.flake8`).
  An Isaac re-run showed the Day 27 lock fix works, but the fleet froze in
  another way: traffic ticks are lockstep and one robot stood still 0.14 m
  off its cell centre. Fixed with a lockstep watchdog (still for 5 s inside
  its target cell counts as arrived; used for the tick gate only, not for
  docking). Unit test plus light robustness check 3/3 pass. See PROGRESS.md
  for details. Isaac fleet run after the fix: 5/6 tasks (was 1/6), no
  freeze; the 6th failed a pick by 9 mm (box 0.109 m from the grip point,
  tolerance 0.10 m). Min separation 0.502 m, peak VRAM 1.2 GB.
- Day 29: docs (architecture diagram, fresh-shell walkthrough,
  troubleshooting). A missed pick now re-docks once with a fresh lidar
  measure before it fails. **Isaac fleet check PASS: 6/6 tasks**, 0 failed,
  min separation 0.506 m, 38.8 tasks/h, idle 43 %, peak VRAM 1.2 GB.
- Day 30: final demo recorded in `out/demo/` (git-ignored) with
  `timeout 900 scripts/launch_all.sh --fleet --seconds 780 --tasks --record
  out/demo/bag --media out/demo/media`. Bag 54.6 MB / 597 s (BAG_PASS with
  the fleet topic list), `demo.mp4` (overview + chase, 117 s, MEDIA_PASS).
  Result: **5/6 tasks done, 0 failed**. The 6th (box_03 → shelf_C) was still
  in `to_drop` when the 780 s run ended. 25 tasks/h, idle 38 %.
  **Known issue:** robot2 and robot3 came within 0.18–0.20 m (AMCL poses) near
  (−0.4, 0.2) at t≈425 s, so this run broke the 0.3 m separation rule; the
  Day 29 run kept 0.506 m. Isaac fleet runs vary from run to run.

## Final state (end of the 30-day plan)
- Single robot: SLAM mapping, Nav2 + AMCL, lidar docking and MoveBox
  pick/drop work reliably in Isaac Sim (Days 1–24).
- Fleet (3 robots, namespaced, domain 21): the fleet manager uses nearest-free
  assignment, cell reservations, shelf locks and parking; it reports throughput
  and idle-time metrics. Light-sim scenario and robustness checks pass every
  time. Isaac fleet runs: 6/6 (Day 29), 5/6 within the time limit plus one
  close approach (Day 30).
- Open work: in Isaac the separation guard has to rely on AMCL poses that
  update slowly (≈0.5 Hz while a robot stands still). Next steps would be to
  use odometry-fused poses for the separation and reservation checks, and to
  add a hard stop when a robot comes within 0.4 m of another.

Notes: the headless sim runs faster than real time unless paced
(`run_sim.py` paces to real time by default); nodes must use `use_sim_time`.
`/odom` is ground truth of the chassis relative to the spawn pose (no wheel
slip), so in-place turns from open-loop `/cmd_vel` undershoot in the world
but odometry stays exact; use `waypoint_driver.py` for repeatable routes.
Kit leaves an `omni.telemetry.transmitter` process after shutdown; the scripts
stop it (`scripts/lib.sh`), together with the domain-21 ros2 daemon.
