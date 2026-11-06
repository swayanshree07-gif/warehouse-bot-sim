#!/usr/bin/env bash
# Days 22-23 check: queued MoveBox tasks with failure handling, verified
# against the sim's ground-truth /boxes topic. One sim run:
#  1. /boxes matches the registry's initial placement, ~10 Hz, frame map
#  2. /sim/remove_box box_03 (the box "goes missing"); /boxes shows it off-slot
#  3. four goals sent at once (queued, run in arrival order):
#       box_09 -> shelf_D        succeeds (auto slot 1)
#       box_03 -> shelf_D        aborts "not on any shelf slot (missing)"
#       box_01 -> shelf_D slot 1 aborts "slot occupied ... box_09" (taken by task 1)
#       box_00 -> shelf_D        succeeds (auto slot 0, freed by task 1)
#     successes must report place_error < 0.05 m (now real, from /boxes)
#  4. after each success /boxes is checked against the expected placement;
#     at the end every other box is still on its slot
# Exit 0 = PASS. Logs: out/logs/check_task_queue*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
NAME=check_task_queue
L="$ROOT/out/logs"
LLOG="$L/$NAME.log"
export RUN_SIM_LOG="$L/${NAME}_sim.log"
NLOG="$L/${NAME}_nav2.log"
TLOG="$L/${NAME}_tm.log"
[ -f "$ROOT/ros_ws/install/warehouse_tasks/share/warehouse_tasks/package.xml" ] \
  || { echo "ros_ws not built: run scripts/build_ws.sh"; exit 1; }

bash "$ROOT/scripts/run_sim.sh" --seconds 600 --start=-2,0,0 >"$LLOG" 2>&1 &
LPID=$!
NPID=""; TPID=""; QPIDS=()
cleanup() {
  for p in "${QPIDS[@]}"; do kill "$p" 2>/dev/null; done
  [ -n "$TPID" ] && { kill -INT "$TPID" 2>/dev/null; sleep 3; kill -9 -- -"$TPID" 2>/dev/null; }
  [ -n "$NPID" ] && { kill -INT -- -"$NPID" 2>/dev/null; sleep 4; kill -9 -- -"$NPID" 2>/dev/null; }
  kill -TERM "$LPID" 2>/dev/null
  for _ in $(seq 1 60); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  pkill -9 -f "warehouse-bot-sim/sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

FAILS=0
ok() { echo "ok   $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
probe() {   # probe <label> [boxes_probe args]
  local label=$1; shift
  if timeout 30 python3 "$ROOT/scripts/boxes_probe.py" "$@" >"$L/${NAME}_probe.log" 2>&1; then
    ok "/boxes $label"
  else fail "/boxes $label"; grep -v '^BOXES_GT' "$L/${NAME}_probe.log" | grep -v '^ok' | head -5; fi
}

for _ in $(seq 1 300); do
  grep -q "LAUNCH_READY" "$LLOG" && break
  kill -0 "$LPID" 2>/dev/null || break
  sleep 1
done
grep -q "LAUNCH_READY" "$LLOG" || { echo "sim not ready"; cat "$LLOG"; exit 1; }
setsid timeout 640 ros2 launch warehouse_bringup nav2.launch.py >"$NLOG" 2>&1 &
NPID=$!
setsid timeout 640 ros2 launch warehouse_tasks task_manager.launch.py backend:=ros >"$TLOG" 2>&1 &
TPID=$!
probe "initial placement, rate, frame" --rate
timeout 10 ros2 topic pub --once -w 1 /sim/remove_box std_msgs/msg/String "{data: box_03}" >/dev/null 2>&1
sleep 2
grep -q "BOX_REMOVE ok=1 box=box_03" "$RUN_SIM_LOG" && ok "box_03 removed in the sim" || fail "remove_box"
probe "box_03 off-slot after removal" --expect box_03=-
for _ in $(seq 1 90); do grep -q "Managed nodes are active" "$NLOG" && break; sleep 1; done
grep -q "Managed nodes are active" "$NLOG" && ok "nav2 active" || { fail "nav2 not active"; exit 1; }
sleep 3

# four goals at once (each sent after the previous one was accepted)
GOALS=("box_09 shelf_D" "box_03 shelf_D" "box_01 shelf_D 1" "box_00 shelf_D")
T0=$(date +%s)
for i in "${!GOALS[@]}"; do
  # shellcheck disable=SC2086
  timeout 500 python3 "$ROOT/scripts/move_box.py" ${GOALS[$i]} --timeout 480 >"$L/${NAME}_q$i.log" 2>&1 &
  QPIDS+=($!)
  # wait until accepted, so the server sees the goals in exactly this order
  for _ in $(seq 1 100); do grep -q MOVE_BOX_ACCEPTED "$L/${NAME}_q$i.log" && break; sleep 0.2; done
done
RCS=()
for p in "${QPIDS[@]}"; do wait "$p"; RCS+=($?); done
QPIDS=()
echo "     all goals finished in $(( $(date +%s) - T0 )) s"
for i in "${!GOALS[@]}"; do echo "     [${GOALS[$i]}] $(grep '^MOVE_BOX_RESULT' "$L/${NAME}_q$i.log")"; done

res() { grep '^MOVE_BOX_RESULT' "$L/${NAME}_q$1.log"; }
pe_ok() {   # place_error in [0, 0.05)
  local e; e=$(res "$1" | sed -n 's/.*"place_error": \([-0-9.e]*\).*/\1/p')
  /usr/bin/python3 -c "import sys; e=float('$e' or -1); sys.exit(0 if 0 <= e < 0.05 else 1)"
}
[ "${RCS[0]}" = 0 ] && res 0 | grep -q '"final_shelf": "shelf_D", "final_slot": 1' \
  && ok "box_09 -> shelf_D slot 1 succeeded" || fail "box_09 task"
pe_ok 0 && ok "box_09 place_error $(res 0 | grep -o '"place_error": [0-9.]*')" || fail "box_09 place_error"
[ "${RCS[1]}" != 0 ] && res 1 | grep -q '"status": "aborted".*box_03 is not on any shelf slot (missing)' \
  && ok "missing box_03 aborted with reason" || fail "missing-box task"
[ "${RCS[2]}" != 0 ] && res 2 | grep -q '"status": "aborted".*slot occupied: shelf_D slot 1 holds box_09' \
  && ok "occupied slot aborted with reason" || fail "occupied-slot task"
[ "${RCS[3]}" = 0 ] && res 3 | grep -q '"final_shelf": "shelf_D", "final_slot": 0' \
  && ok "box_00 -> shelf_D slot 0 succeeded" || fail "box_00 task"
pe_ok 3 && ok "box_00 place_error $(res 3 | grep -o '"place_error": [0-9.]*')" || fail "box_00 place_error"
ORDER=$(grep -o 'TASK_RESULT {"box_id": "box_0[0-9]"' "$TLOG" | grep -o 'box_0[0-9]' | paste -sd' ')
[ "$ORDER" = "box_09 box_03 box_01 box_00" ] && ok "ran in arrival order ($ORDER)" || fail "order: $ORDER"
probe "final placement (2 moved, box_03 missing, 7 untouched)" \
  --expect box_09=shelf_D:1,box_00=shelf_D:0,box_03=-
grep -q "ground truth differs from belief: box_03" "$TLOG" && ok "task_manager saw box_03 missing via /boxes" \
  || fail "no ground-truth sync log for box_03"
grep -qE "Traceback" "$TLOG" && fail "traceback in task_manager log"
echo "gt: $(grep '^POSE' "$RUN_SIM_LOG" | tail -1)"
[ $FAILS -eq 0 ] && { echo "$NAME: PASS"; exit 0; }
echo "$NAME: FAIL ($FAILS)"; exit 1
