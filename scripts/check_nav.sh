#!/usr/bin/env bash
# Days 12-13 check: sim + Nav2 (warehouse_bringup nav2.launch.py: saved map,
# AMCL, RPP controller). scripts/nav_probe.py checks that all Nav2 lifecycle
# nodes are active, AMCL is within 0.2 m of ground truth, then sends
# NavigateToPose goals (map frame) and checks each true final position is
# within 0.25 m of the goal. Exit 0 = PASS. Logs: out/logs/check_nav*.log
#   scripts/check_nav.sh [--slam] [--goal x,y,yaw_deg ...]   (default: 3-goal tour)
# --slam (Day 14): Nav2 on slam_toolbox's live map instead of the saved map +
# AMCL (nav2.launch.py slam:=true); the localisation check uses TF
# map->base_link, and /map must grow. Logs then go to out/logs/check_nav_slam*.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
SLAM=false; NAME=check_nav; LOCALIZER=amcl
if [ "$1" = "--slam" ]; then SLAM=true; NAME=check_nav_slam; LOCALIZER=slam; shift; fi
LLOG="$ROOT/out/logs/$NAME.log"
export RUN_SIM_LOG="$ROOT/out/logs/${NAME}_sim.log"
NLOG="$ROOT/out/logs/${NAME}_nav2.log"
[ -f "$ROOT/ros_ws/install/local_setup.bash" ] || { echo "ros_ws not built: run scripts/build_ws.sh"; exit 1; }
# Default tour (map frame = world shifted by +2 m in x; start world (-2,0,0)):
# east end of the central aisle, north aisle between the racks facing west,
# back to the start pose.
GOALS=("$@")
[ ${#GOALS[@]} -eq 0 ] && GOALS=(--goal 3.5,0,0 --goal 2.0,3.1,180 --goal 0,0,0)

bash "$ROOT/scripts/run_sim.sh" --seconds 480 --start=-2,0,0 >"$LLOG" 2>&1 &
LPID=$!
NPID=""
cleanup() {
  [ -n "$NPID" ] && { kill -INT -- -"$NPID" 2>/dev/null; sleep 4; kill -9 -- -"$NPID" 2>/dev/null; }
  kill -TERM "$LPID" 2>/dev/null
  for _ in $(seq 1 60); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  pkill -9 -f "warehouse-bot-sim/sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 300); do
  grep -q "LAUNCH_READY" "$LLOG" && break
  kill -0 "$LPID" 2>/dev/null || break
  sleep 1
done
grep -q "LAUNCH_READY" "$LLOG" || { echo "sim not ready"; cat "$LLOG"; exit 1; }
setsid timeout 460 ros2 launch warehouse_bringup nav2.launch.py slam:=$SLAM >"$NLOG" 2>&1 &
NPID=$!

timeout 420 python3 "$ROOT/scripts/nav_probe.py" --sim-log "$RUN_SIM_LOG" --start=-2,0,0 --localizer $LOCALIZER "${GOALS[@]}" \
  2>&1 | grep -E "^(ok|FAIL|NAV_PROBE)"
RC=${PIPESTATUS[0]}
echo "gt: $(grep '^POSE' "$RUN_SIM_LOG" | tail -1)"
grep -qiE "\[(ERROR|FATAL)\]" "$NLOG" && { echo "nav2 errors (first 5):"; grep -iE "\[(ERROR|FATAL)\]" "$NLOG" | head -5; }
[ "$RC" = 0 ] && echo "$NAME: PASS" || { echo "$NAME: FAIL (rc=$RC, see $NLOG)"; exit 1; }
