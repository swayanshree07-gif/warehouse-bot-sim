#!/usr/bin/env bash
# Day 25 check: full Isaac fleet scenario. 3 robots (AMCL on the saved map),
# fleet manager Isaac backend (scripts/fleet_isaac.py: nearest-free assignment,
# traffic reservations, shelf docking, per-robot lift/gripper), 6 chained
# pick/drop tasks sent over /fleet/tasks by scripts/fleet_client.py.
# PASS: see scripts/eval_fleet_isaac.py. Report: out/fleet_isaac_report.json.
# Whole run is bounded by timeout 900 (sim 840 s wall).
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
source "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/out/logs"
L="$ROOT/out/logs/check_fleet_isaac"
LOG="$L.log"; REPORT="$ROOT/out/fleet_isaac_report.json"; rm -f "$REPORT"
TASKS=("box_00 shelf_D" "box_04 shelf_D" "box_06 shelf_A" "box_07 shelf_B" "box_01 shelf_C" "box_03 shelf_C")

timeout --kill-after=20 840 python "$ROOT/sim/run_fleet_sim.py" --seconds 800 --pose-every 0.2 --box-every 1 >"$LOG" 2>&1 &
SIM_PID=$!
LAUNCH_PID=""; FM_PID=""
cleanup() {
  [ -n "$FM_PID" ] && kill -INT "$FM_PID" 2>/dev/null
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 2; kill "$SIM_PID" 2>/dev/null; sleep 3
  [ -n "$LAUNCH_PID" ] && kill -9 "$LAUNCH_PID" 2>/dev/null
  pkill -9 -f "sim/run_fleet_si[m].py" 2>/dev/null
  pkill -f "fleet_localization[.]launch" 2>/dev/null
  pkill -f "__ns:=/robot[0-9]" 2>/dev/null
  pkill -f "scripts/fleet_isaa[c].py" 2>/dev/null
  pkill -f "scripts/gripper_cm[d].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 600); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "sim never became ready"; exit 1; }
timeout 780 ros2 launch warehouse_bringup fleet_localization.launch.py >"${L}_launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 15
timeout 740 python3 "$ROOT/scripts/fleet_isaac.py" --out "$REPORT" --timeout 700 >"${L}_fm.log" 2>&1 &
FM_PID=$!
for _ in $(seq 1 60); do grep -q "fleet_manager ready" "${L}_fm.log" && break; sleep 0.5; done
grep -q "fleet_manager ready" "${L}_fm.log" || { echo "fleet manager did not start"; cat "${L}_fm.log"; exit 1; }
echo "ok   fleet_manager (isaac) ready"
: >"${L}_ram.log"
( while true; do free -m | awk '/Mem:/{print "RAM_USED_MB="$3}' >>"${L}_ram.log"; sleep 2; done ) &
RAM_PID=$!
timeout 680 python3 "$ROOT/scripts/fleet_client.py" --timeout 660 "${TASKS[@]}" >"${L}_client.log" 2>&1
RC=$?
kill $RAM_PID 2>/dev/null
grep -v "^FLEET_STATUS" "${L}_client.log" | tail -3
kill -INT "$FM_PID" 2>/dev/null; wait "$FM_PID" 2>/dev/null; FM_PID=""
grep -E "^\[|ACTION_FAIL" "${L}_fm.log" | tail -40
sleep 2
PEAK=$(grep -oE "VRAM_MB=[0-9]+" "$LOG" | cut -d= -f2 | sort -n | tail -1)
RAM=$(cut -d= -f2 "${L}_ram.log" | sort -n | tail -1)
/usr/bin/python3 "$ROOT/scripts/eval_fleet_isaac.py" "$LOG" "$REPORT" "$RC" "${PEAK:-99999}" "${RAM:-99999}"
