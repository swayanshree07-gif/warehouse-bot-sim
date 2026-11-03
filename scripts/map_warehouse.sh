#!/usr/bin/env bash
# Day 10: build the warehouse map. Starts the sim (scripts/run_sim.sh) and
# slam_toolbox (warehouse_bringup slam.launch.py), drives a scripted
# exploration loop around all four racks with scripts/waypoint_driver.py,
# saves out/maps/warehouse.{pgm,yaml} and checks coverage against the scene
# geometry (sim/map_eval.py, >= 80 % of free area). Exit 0 = PASS.
# Logs: out/logs/map_warehouse*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs" "$ROOT/out/maps"
START="-2,0,0"          # world x,y,yaw_deg; map/odom frames start here
MAP="${MAP_NAME:-$ROOT/out/maps/warehouse}"
LLOG="$ROOT/out/logs/map_warehouse.log"
export RUN_SIM_LOG="$ROOT/out/logs/map_warehouse_sim.log"
SLOG="$ROOT/out/logs/map_warehouse_slam.log"
[ -f "$ROOT/ros_ws/install/local_setup.bash" ] || { echo "ros_ws not built: run scripts/build_ws.sh"; exit 1; }

bash "$ROOT/scripts/run_sim.sh" --seconds 420 --start="$START" >"$LLOG" 2>&1 &
LPID=$!
SPID=""
cleanup() {
  [ -n "$SPID" ] && { kill -INT -- -"$SPID" 2>/dev/null; sleep 3; kill -9 -- -"$SPID" 2>/dev/null; }
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
setsid timeout 400 ros2 launch warehouse_bringup slam.launch.py >"$SLOG" 2>&1 &
SPID=$!
sleep 8

# Route in the odom frame (= world shifted by +2 m in x, since START is
# (-2, 0, 0)). World loop: spin, (0,0) -> north aisle y=3.1 -> east x=4 ->
# south aisle y=-3.1 -> west x=-4 -> back north to y=3.1 -> (-4,0) -> (0,0).
# Clearances: >= 0.75 m to racks, >= 0.9 m to walls.
timeout 400 python3 "$ROOT/scripts/waypoint_driver.py" \
  h:90 h:180 h:-90 h:0 \
  2,0 2,3.1 6,3.1 6,-3.1 -2,-3.1 -2,3.1 -2,0 2,0 h:0 | grep -E "WAYPOINT"
RC_DRIVE=${PIPESTATUS[0]}
sleep 4
echo "gt: $(grep '^POSE' "$RUN_SIM_LOG" | tail -1)"
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$MAP" --ros-args -p save_map_timeout:=10.0 \
  >"$ROOT/out/logs/map_warehouse_saver.log" 2>&1 || { echo "FAIL map_saver (out/logs/map_warehouse_saver.log)"; exit 1; }
echo "saved $MAP.{pgm,yaml}"
[ "$RC_DRIVE" = 0 ] && echo "ok   route driven" || echo "FAIL route (waypoint driver rc=$RC_DRIVE)"
python3 "$ROOT/sim/map_eval.py" "$MAP.yaml" --start="$START" --min-coverage 0.8
RC=$?
[ "$RC_DRIVE" = 0 ] && [ $RC = 0 ] && echo "map_warehouse: PASS" || { echo "map_warehouse: FAIL"; exit 1; }
