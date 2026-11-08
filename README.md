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
scripts/check_robustness.sh [N] [--slam]  # check_launch_all N times (default 3) + VRAM/RAM peaks (~4 min/run)
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
| `sim/registry_gen.py` | generates the shelf/box registry YAML (slot centres, approach/dock poses) |
| `sim/box_handler.py` | in-sim rclpy node: lift command, virtual gripper attach/detach, `/boxes` ground truth, `/sim/remove_box` test hook |
| `sim/box_state.py` | box -> shelf slot classification for `/boxes` (pure Python) |
| `sim/media.py` | overview + chase cameras, replicator `rgb` annotators -> PNG frames |
| `sim/map_eval.py` | compares a saved map with the scene geometry (coverage, occupied-cell IoU) |
| `scripts/` | launchers, drivers, probes and scripted checks (`lib.sh`: cleanup helpers) |
| `ros_ws/src/warehouse_bringup` | URDF, `description`/`slam`/`nav2` launch files, slam_toolbox + Nav2 params, saved map, registry |
| `ros_ws/src/warehouse_interfaces` | `MoveBox.action`, `BoxState.msg`, `BoxStates.msg` |
| `ros_ws/src/warehouse_tasks` | `task_manager` node: MoveBox action server, pick/place state machine (`mission.py`), registry belief, docking law (`line_track.py`), lidar rack-face measurement (`rack_align.py`), mock + ROS backends |
| `tests/` | pytest tests that run without Isaac Sim |
| `ros_ws/src/` | own ROS 2 packages (colcon) |
| `out/` | logs, maps, bags, media (git-ignored, keep < 4 GB) |

## Status
- Last re-verified 2026-11-08 (session Day 12): `scripts/check_nav.sh` PASS
  (Nav2 + AMCL on the saved map, 3-goal tour).
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
- Next: Day 28 polish/lint, Day 29 docs, Day 30 final demo.

Notes: the headless sim runs faster than real time unless paced
(`run_sim.py` paces to real time by default); nodes must use `use_sim_time`.
`/odom` is ground truth of the chassis relative to the spawn pose (no wheel
slip), so in-place turns from open-loop `/cmd_vel` undershoot in the world
but odometry stays exact; use `waypoint_driver.py` for repeatable routes.
Kit leaves an `omni.telemetry.transmitter` process after shutdown; the scripts
stop it (`scripts/lib.sh`), together with the domain-21 ros2 daemon.
