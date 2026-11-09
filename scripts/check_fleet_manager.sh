#!/usr/bin/env bash
# Day 23 check: fleet_manager node on the light sim backend (no GPU).
# Sends 6 tasks + 1 malformed task over /fleet/tasks; PASS iff all 6 are done,
# the bad one is rejected, every task has a robot, >= 2 robots were used,
# 0 collisions, min separation >= 0.5 m, status carries metrics.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_fleet_manager.log"
NLOG="$ROOT/out/logs/check_fleet_manager_node.log"
timeout 300 "$ROOT/scripts/build_ws.sh" --packages-select warehouse_interfaces warehouse_bringup warehouse_tasks \
  >"$LOG" 2>&1 || { echo "FAIL colcon build"; tail -20 "$LOG"; echo "check_fleet_manager: FAIL"; exit 1; }
echo "ok   colcon build"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
setsid timeout 150 ros2 launch warehouse_tasks fleet_manager.launch.py tick_period:=0.05 >"$NLOG" 2>&1 &
NPID=$!
trap 'kill -INT $NPID 2>/dev/null; sleep 1; kill -9 -- -$NPID 2>/dev/null; true' EXIT
for _ in $(seq 1 30); do grep -q "fleet_manager ready" "$NLOG" && break; sleep 0.5; done
grep -q "fleet_manager ready" "$NLOG" || { echo "FAIL node did not start"; cat "$NLOG"; exit 1; }
echo "ok   fleet_manager ready"
OUT=$(timeout 120 python3 "$ROOT/scripts/fleet_client.py" --timeout 100 --expect-rejected 1 \
  "box_00 zone_E" "box_04 zone_E" "box_08 zone_E" "box_01 shelf_D" "box_03 shelf_A" "box_09 shelf_B" \
  "garbage") ; RC=$?
echo "$OUT" >>"$LOG"
[ $RC -eq 0 ] && echo "ok   client: all tasks done" || echo "FAIL client rc=$RC"
echo "$OUT" | sed -n 's/^FLEET_STATUS //p' | python3 "$ROOT/scripts/verify_fleet_status.py" && OK2=1 || OK2=0
[ $RC -eq 0 ] && [ $OK2 -eq 1 ] && { echo "check_fleet_manager: PASS"; exit 0; }
echo "check_fleet_manager: FAIL"; tail -20 "$NLOG"; exit 1
