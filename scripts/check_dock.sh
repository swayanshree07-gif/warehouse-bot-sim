#!/usr/bin/env bash
# Day 20 check: docking accuracy. Sim + Nav2/AMCL on the packaged map, then
# scripts/dock_probe.py docks at 5 slots (Nav2 to the approach pose, RosRobot
# dock, undock) and compares the sim's ground-truth pose with the registry
# dock pose. PASS = 5/5 trials within 3 cm and 3 deg.
#   scripts/check_dock.sh [--slot shelf_X:N ...] [--max-pos-err M] [--max-yaw-err DEG]
# Exit 0 = PASS. Logs: out/logs/check_dock*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
NAME=check_dock
LLOG="$ROOT/out/logs/$NAME.log"
export RUN_SIM_LOG="$ROOT/out/logs/${NAME}_sim.log"
NLOG="$ROOT/out/logs/${NAME}_nav2.log"
PLOG="$ROOT/out/logs/${NAME}_probe.log"
[ -f "$ROOT/ros_ws/install/warehouse_tasks/share/warehouse_tasks/package.xml" ] \
  || { echo "ros_ws not built: run scripts/build_ws.sh"; exit 1; }

bash "$ROOT/scripts/run_sim.sh" --seconds 600 --start=-2,0,0 --pose-every 0.1 >"$LLOG" 2>&1 &
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
setsid timeout 640 ros2 launch warehouse_bringup nav2.launch.py >"$NLOG" 2>&1 &
NPID=$!
for _ in $(seq 1 90); do
  grep -q "Managed nodes are active" "$NLOG" && break; sleep 1
done
grep -q "Managed nodes are active" "$NLOG" && echo "ok   nav2 active" || { echo "FAIL nav2 not active"; exit 1; }
sleep 3

timeout 540 python3 "$ROOT/scripts/dock_probe.py" --sim-log "$RUN_SIM_LOG" --start=-2,0,0 "$@" >"$PLOG" 2>&1
RC=$?
grep -E "^(ok|FAIL|DOCK_TRIAL|dock trials)" "$PLOG"
grep -q Traceback "$PLOG" && { echo "probe traceback:"; tail -15 "$PLOG"; }
echo "gt: $(grep '^POSE' "$RUN_SIM_LOG" | tail -1)"
[ "$RC" = 0 ] && { echo "$NAME: PASS"; exit 0; }
echo "$NAME: FAIL (rc=$RC, see $PLOG)"; exit 1
