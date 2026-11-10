#!/usr/bin/env bash
# Fleet mode of the single launch script (Day 26). Called by
#   scripts/launch_all.sh --fleet [--seconds N] [--tasks] [--record [DIR]] [--media [DIR]]
# Starts: Isaac fleet sim (3 robots, sim/run_fleet_sim.py), per-robot AMCL
# (fleet_localization.launch.py), the fleet manager Isaac backend
# (scripts/fleet_isaac.py, /fleet/tasks in, /fleet/status out), optionally a
# rosbag of namespaced topics (regex $BAG_RE) and overview + robot1 chase
# frames -> mp4. Prints FLEET_READY when the fleet manager is up. With --tasks
# the 6 demo tasks are sent (scripts/fleet_client.py) and the stack stops when
# they finish (FLEET_TASKS rc=<n>); otherwise it runs until --seconds or Ctrl-C.
# Always ends with ALL_STOPPED and leaves no processes. Logs: out/logs/launch_fleet_*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh" >/dev/null
source "$ROOT/scripts/lib.sh"
source "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/out/logs"
P="${LAUNCH_ALL_LOG_PREFIX:-$ROOT/out/logs/launch_fleet}"
SECS=600; BAG=""; MEDIA=""; SEND=0
BAG_RE='^/(clock|tf|tf_static|map|boxes|fleet/(tasks|status)|robot[0-9]+/(odom|scan|cmd_vel|amcl_pose|joint_states|lift/height|gripper/held))$'
TASKS=("box_00 shelf_D" "box_04 shelf_D" "box_06 shelf_A" "box_07 shelf_B" "box_01 shelf_C" "box_03 shelf_C")
while [ $# -gt 0 ]; do
  case "$1" in
    --seconds) SECS="$2"; shift 2 ;;
    --tasks) SEND=1; shift ;;
    --record)
      if [ -n "$2" ] && [ "${2#--}" = "$2" ]; then BAG="$2"; shift 2
      else BAG="$ROOT/out/bags/fleet_$(date +%Y%m%d_%H%M%S)"; shift; fi ;;
    --media)
      if [ -n "$2" ] && [ "${2#--}" = "$2" ]; then MEDIA="$2"; shift 2
      else MEDIA="$ROOT/out/media/fleet_$(date +%Y%m%d_%H%M%S)"; shift; fi ;;
    *) echo "unknown fleet argument: $1"; exit 2 ;;
  esac
done
[ "$SECS" -gt 840 ] && SECS=840
if [ -n "$(list_sim_leftovers)" ]; then echo "ALL another sim is still running"; exit 1; fi
CAP=$((SECS + 60))
SIM_EXTRA=()
if [ -n "$MEDIA" ]; then
  [ -e "$MEDIA" ] && { echo "ALL media dir exists: $MEDIA"; exit 1; }
  SIM_EXTRA+=(--capture "$MEDIA" --capture-every 0.5)
fi
setsid timeout --kill-after=20 "$CAP" python "$ROOT/sim/run_fleet_sim.py" --seconds "$SECS" \
  --pose-every 0.2 --box-every 1 "${SIM_EXTRA[@]}" >"${P}_sim.log" 2>&1 &
SPID=$!; LPID=""; FPID=""; BPID=""; STOPPING=0
stop_all() {
  [ "$STOPPING" = 1 ] && return; STOPPING=1
  echo "ALL stopping"
  if [ -n "$BPID" ]; then
    kill -INT -- -"$BPID" 2>/dev/null
    for _ in $(seq 1 15); do kill -0 -- -"$BPID" 2>/dev/null || break; sleep 1; done
    kill -9 -- -"$BPID" 2>/dev/null
    echo "ALL bag closed: $BAG ($(du -sh "$BAG" 2>/dev/null | cut -f1))"
  fi
  for g in $FPID $LPID; do kill -INT -- -"$g" 2>/dev/null; done
  for _ in $(seq 1 10); do
    alive=0; for g in $FPID $LPID; do kill -0 -- -"$g" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 1
  done
  for g in $FPID $LPID; do kill -9 -- -"$g" 2>/dev/null; done
  kill -INT -- -"$SPID" 2>/dev/null
  for _ in $(seq 1 60); do kill -0 "$SPID" 2>/dev/null || break; sleep 1; done
  kill -9 -- -"$SPID" 2>/dev/null
  pkill -9 -f "sim/run_fleet_si[m].py" 2>/dev/null
  pkill -f "scripts/gripper_cm[d].py" 2>/dev/null
  kill_kit_leftovers
  if [ -n "$MEDIA" ] && [ -d "$MEDIA" ]; then
    timeout 600 bash "$ROOT/scripts/make_video.sh" "$MEDIA" >"${P}_video.log" 2>&1
    echo "ALL media: $(grep -o 'CAPTURE_DONE.*' "${P}_sim.log" | tail -1); $(tail -1 "${P}_video.log")"
  fi
  echo "ALL_STOPPED ($(grep -o 'SIM_DONE.*' "${P}_sim.log" | tail -1))"
}
trap stop_all EXIT
trap 'exit 130' INT TERM

echo "ALL fleet sim starting (${SECS}s, ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
for _ in $(seq 1 600); do
  grep -q "SIM_READY" "${P}_sim.log" && break
  kill -0 "$SPID" 2>/dev/null || break
  sleep 1
done
grep -q "SIM_READY" "${P}_sim.log" || { echo "ALL sim not ready:"; tail -20 "${P}_sim.log"; exit 1; }
if [ -n "$BAG" ]; then
  [ -e "$BAG" ] && { echo "ALL bag dir exists: $BAG"; exit 1; }
  mkdir -p "$(dirname "$BAG")"
  echo "ALL recording topics matching $BAG_RE to $BAG"
  setsid timeout -s INT --kill-after=20 "$CAP" ros2 bag record -s mcap --storage-preset-profile zstd_fast \
    --use-sim-time -e "$BAG_RE" -o "$BAG" >"${P}_bag.log" 2>&1 &
  BPID=$!
fi
setsid timeout -s INT --kill-after=20 "$CAP" ros2 launch warehouse_bringup fleet_localization.launch.py \
  >"${P}_loc.log" 2>&1 &
LPID=$!
sleep 15
setsid timeout -s INT --kill-after=20 "$CAP" python3 "$ROOT/scripts/fleet_isaac.py" \
  --out "$ROOT/out/fleet_launch_report.json" --timeout "$SECS" >"${P}_fm.log" 2>&1 &
FPID=$!
for _ in $(seq 1 60); do grep -q "fleet_manager ready" "${P}_fm.log" && break; sleep 0.5; done
grep -q "fleet_manager ready" "${P}_fm.log" || { echo "ALL fleet manager did not start"; tail "${P}_fm.log"; exit 1; }
echo "FLEET_READY"
if [ $SEND = 1 ]; then
  timeout $((SECS - 30)) python3 "$ROOT/scripts/fleet_client.py" --timeout $((SECS - 60)) "${TASKS[@]}" \
    >"${P}_client.log" 2>&1
  echo "FLEET_TASKS rc=$? $(grep -v '^FLEET_STATUS' "${P}_client.log" | tail -1)"
  kill -INT -- -"$FPID" 2>/dev/null; sleep 3    # fleet manager writes its report on SIGINT
  exit 0
fi
while true; do
  kill -0 "$SPID" 2>/dev/null || { echo "ALL sim exited"; break; }
  kill -0 "$LPID" 2>/dev/null || { echo "ALL localization exited"; break; }
  kill -0 "$FPID" 2>/dev/null || { echo "ALL fleet manager exited"; break; }
  if [ -n "$BPID" ]; then kill -0 "$BPID" 2>/dev/null || { echo "ALL bag recorder exited"; break; }; fi
  sleep 1
done
exit 0
