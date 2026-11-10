#!/usr/bin/env bash
# Single launch script for the whole stack (Day 24):
#   sim (run_sim.sh) + Nav2 (AMCL on the packaged map, or --slam) + task_manager
#
#   scripts/launch_all.sh [--slam] [--seconds N] [--no-tasks] [--no-health]
#                         [--record [DIR]] [--media [DIR]]
#
# Everything runs headless on ROS_DOMAIN_ID=21. When all parts are up,
# scripts/health_check.py runs and the script prints `ALL_READY` (or exits 1 on
# a failed health check). Then send tasks, e.g.
#   source env.sh && python3 scripts/move_box.py box_09 shelf_D
# The stack stops when the sim run time (--seconds, default 600, cap 840)
# ends, when any part dies, or on Ctrl-C / SIGTERM; all child processes are
# stopped and Kit leftovers removed. Prints `ALL_STOPPED` at the end.
# --record (Day 25) records a short, filtered rosbag (mcap, zstd) of $BAG_TOPICS
# into DIR (default out/bags/run_<date>_<time>); recording starts with Nav2 and
# is stopped first (SIGINT) on shutdown so the bag is closed cleanly.
# --media (Day 26) captures overview + chase camera PNGs (480x270, every 0.5
# sim-s) into DIR (default out/media/run_<date>_<time>) and, after the sim has
# stopped, turns them into DIR/{overview,chase,demo}.mp4 (scripts/make_video.sh).
# --fleet (Day 26) runs the 3-robot fleet stack instead (scripts/launch_fleet.sh;
# accepts --seconds, --tasks, --record [DIR], --media [DIR]).
# Logs: out/logs/launch_all_{sim,nav2,tm,health,bag,video}.log (override the prefix
# with LAUNCH_ALL_LOG_PREFIX).
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ "$1" = "--fleet" ]; then shift; exec bash "$ROOT/scripts/launch_fleet.sh" "$@"; fi
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
P="${LAUNCH_ALL_LOG_PREFIX:-$ROOT/out/logs/launch_all}"

SLAM=false; SECS=600; TASKS=1; HEALTH=1; BAG=""; MEDIA=""
# Small topics only: no camera, ~10 Hz scan. /map is a 200x160 grid. The
# /move_box/_action/* topics are "hidden", hence --include-hidden-topics.
BAG_TOPICS=(/clock /tf /tf_static /odom /scan /joint_states /cmd_vel /map
  /amcl_pose /plan /boxes /lift/height /gripper/held
  /move_box/_action/status /move_box/_action/feedback)
while [ $# -gt 0 ]; do
  case "$1" in
    --slam) SLAM=true; shift ;;
    --seconds) SECS="$2"; shift 2 ;;
    --seconds=*) SECS="${1#*=}"; shift ;;
    --no-tasks) TASKS=0; shift ;;
    --no-health) HEALTH=0; shift ;;
    --record)
      if [ -n "$2" ] && [ "${2#--}" = "$2" ]; then BAG="$2"; shift 2
      else BAG="$ROOT/out/bags/run_$(date +%Y%m%d_%H%M%S)"; shift; fi ;;
    --media)
      if [ -n "$2" ] && [ "${2#--}" = "$2" ]; then MEDIA="$2"; shift 2
      else MEDIA="$ROOT/out/media/run_$(date +%Y%m%d_%H%M%S)"; shift; fi ;;
    -h|--help) sed -n 2,27p "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done
[ -f "$ROOT/ros_ws/install/warehouse_tasks/share/warehouse_tasks/package.xml" ] \
  || { echo "ros_ws not built: run scripts/build_ws.sh"; exit 1; }
if [ -n "$(list_sim_leftovers)" ]; then
  echo "ALL another sim of this project is still running:"; list_sim_leftovers; exit 1
fi
# ROS-side processes live a bit longer than the sim so they can stop cleanly
ROS_CAP=$((SECS + 360))

SIM_EXTRA=()
if [ -n "$MEDIA" ]; then
  [ -e "$MEDIA" ] && { echo "ALL media dir exists: $MEDIA"; exit 1; }
  SIM_EXTRA+=(--capture "$MEDIA" --capture-every 0.5)
fi
export RUN_SIM_LOG="${P}_sim.log"
bash "$ROOT/scripts/run_sim.sh" --seconds "$SECS" --start=-2,0,0 "${SIM_EXTRA[@]}" >"${P}_launch.log" 2>&1 &
LPID=$!
NPID=""; TPID=""; BPID=""
STOPPING=0
stop_all() {
  [ "$STOPPING" = 1 ] && return; STOPPING=1
  echo "ALL stopping"
  if [ -n "$BPID" ]; then   # close the bag first, while topics still flow
    kill -INT -- -"$BPID" 2>/dev/null
    for _ in $(seq 1 15); do kill -0 -- -"$BPID" 2>/dev/null || break; sleep 1; done
    kill -9 -- -"$BPID" 2>/dev/null
    echo "ALL bag closed: $BAG ($(du -sh "$BAG" 2>/dev/null | cut -f1))"
  fi
  # task_manager and Nav2 first (SIGINT to their process groups, as Ctrl-C would)
  for g in $TPID $NPID; do kill -INT -- -"$g" 2>/dev/null; done
  for _ in $(seq 1 10); do
    alive=0; for g in $TPID $NPID; do kill -0 -- -"$g" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 1
  done
  for g in $TPID $NPID; do kill -9 -- -"$g" 2>/dev/null; done
  # then the sim (run_sim.sh stops it gracefully and cleans Kit leftovers)
  kill -TERM "$LPID" 2>/dev/null
  for _ in $(seq 1 60); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -9 "$LPID" 2>/dev/null
  pkill -9 -f "warehouse-bot-sim/sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
  if [ -n "$MEDIA" ] && [ -d "$MEDIA" ]; then
    timeout 600 bash "$ROOT/scripts/make_video.sh" "$MEDIA" >"${P}_video.log" 2>&1
    echo "ALL media: $(grep -o 'CAPTURE_DONE.*' "$RUN_SIM_LOG" | tail -1); $(tail -1 "${P}_video.log")"
  fi
  echo "ALL_STOPPED ($(grep -o 'SIM_DONE.*' "$RUN_SIM_LOG" 2>/dev/null | tail -1))"
}
trap stop_all EXIT
trap 'exit 130' INT TERM

echo "ALL sim starting (log $RUN_SIM_LOG, ${SECS}s, slam=$SLAM, ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
for _ in $(seq 1 300); do
  grep -q "LAUNCH_READY" "${P}_launch.log" && break
  kill -0 "$LPID" 2>/dev/null || break
  sleep 1
done
grep -q "LAUNCH_READY" "${P}_launch.log" || { echo "ALL sim not ready:"; tail -20 "${P}_launch.log"; exit 1; }
echo "ALL sim ready; starting Nav2 (slam=$SLAM)"
if [ -n "$BAG" ]; then
  [ -e "$BAG" ] && { echo "ALL bag dir exists: $BAG"; exit 1; }
  mkdir -p "$(dirname "$BAG")"
  echo "ALL recording ${#BAG_TOPICS[@]} topics to $BAG"
  setsid timeout -s INT --kill-after=20 "$ROS_CAP" \
    ros2 bag record -s mcap --storage-preset-profile zstd_fast --use-sim-time --include-hidden-topics \
    -o "$BAG" "${BAG_TOPICS[@]}" >"${P}_bag.log" 2>&1 &
  BPID=$!
fi
setsid timeout -s INT --kill-after=20 "$ROS_CAP" \
  ros2 launch warehouse_bringup nav2.launch.py slam:=$SLAM >"${P}_nav2.log" 2>&1 &
NPID=$!
HARGS=()
[ "$SLAM" = true ] && HARGS+=(--slam)
if [ $TASKS = 1 ]; then
  echo "ALL starting task_manager"
  setsid timeout -s INT --kill-after=20 "$ROS_CAP" \
    ros2 launch warehouse_tasks task_manager.launch.py backend:=ros >"${P}_tm.log" 2>&1 &
  TPID=$!
else
  HARGS+=(--no-tasks)
fi

if [ $HEALTH = 1 ]; then
  if timeout 150 python3 "$ROOT/scripts/health_check.py" --timeout 120 "${HARGS[@]}" >"${P}_health.log" 2>&1; then
    tail -1 "${P}_health.log"
  else
    echo "ALL health check failed:"; tail -2 "${P}_health.log"; exit 1
  fi
fi
echo "ALL_READY"

# supervise: stop everything as soon as any part exits
while true; do
  kill -0 "$LPID" 2>/dev/null || { echo "ALL sim exited"; break; }
  kill -0 "$NPID" 2>/dev/null || { echo "ALL nav2 exited"; break; }
  if [ -n "$TPID" ]; then kill -0 "$TPID" 2>/dev/null || { echo "ALL task_manager exited"; break; }; fi
  if [ -n "$BPID" ]; then kill -0 "$BPID" 2>/dev/null || { echo "ALL bag recorder exited"; break; }; fi
  sleep 1
done
exit 0
