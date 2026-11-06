#!/usr/bin/env bash
# Day 19 check: task_manager MoveBox action server with the mock backend (no sim).
#  1. box_09 -> shelf_D (auto slot): succeeds into slot 1, feedback lists all 8 states
#  2. box_99 -> shelf_D: aborts "unknown box"
#  3. box_00 -> shelf_D slot 1: aborts "slot occupied" (the belief was updated by 1)
#  4. two goals sent at once (box_00 -> shelf_D, box_01 -> shelf_D): both run one
#     after the other; first gets slot 0 (freed by 1), second gets slot 2
#  5. box_02 -> shelf_A slot 0 with --cancel-after: result canceled
#  6. a second node with mock_fail=navigate:unreachable: aborts "NAV_TO_PICK failed"
#  7. a third node with mock_missing=box_03 (ground-truth sync, Day 22/23):
#     box_03 aborts "missing", box_04 still succeeds with place_error 0
# Exit 0 = PASS. Logs: out/logs/check_task_manager*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_task_manager.log"
TLOG="$ROOT/out/logs/check_task_manager_node.log"
timeout 300 "$ROOT/scripts/build_ws.sh" --packages-select warehouse_interfaces warehouse_bringup warehouse_tasks \
  >"$LOG" 2>&1 || { echo "FAIL colcon build"; tail -20 "$LOG"; echo "check_task_manager: FAIL"; exit 1; }
echo "ok   colcon build"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
TPID=""
start_tm() {   # start_tm <log> [extra launch args]
  local log=$1; shift
  setsid timeout 200 ros2 launch warehouse_tasks task_manager.launch.py backend:=mock \
    use_sim_time:=false mock_step_time:=0.1 "$@" >"$log" 2>&1 &
  TPID=$!
  for _ in $(seq 1 30); do grep -q "task_manager ready" "$log" && return 0; sleep 0.5; done
  echo "FAIL task_manager did not start"; cat "$log"; return 1
}
stop_tm() {
  # SIGINT to the launch process only (it forwards one SIGINT to the node; a
  # group-wide SIGINT would hit the node twice and interrupt its shutdown)
  [ -n "$TPID" ] && { kill -INT "$TPID" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$TPID" 2>/dev/null || break; sleep 0.25; done
    kill -9 -- -"$TPID" 2>/dev/null; }
  TPID=""
}
trap 'stop_tm; kill_kit_leftovers' EXIT

FAILS=0
MB="timeout 60 python3 $ROOT/scripts/move_box.py"
expect() {   # expect <name> <regex on output> <move_box args...>
  local name=$1 re=$2; shift 2
  local out; out=$($MB "$@" 2>&1)
  echo "$out" >>"$LOG"
  if echo "$out" | grep -qE -- "$re"; then echo "ok   $name"
  else echo "FAIL $name"; echo "$out" | tail -3; FAILS=$((FAILS + 1)); fi
  LAST="$out"
}

start_tm "$TLOG" || exit 1
expect "box_09 -> shelf_D auto slot succeeds into slot 1" \
  '"status": "succeeded".*"final_shelf": "shelf_D", "final_slot": 1' box_09 shelf_D
STATES=$(echo "$LAST" | sed -n 's/^MOVE_BOX_FEEDBACK state=\([A-Z_]*\).*/\1/p' | paste -sd' ')
if [ "$STATES" = "NAV_TO_PICK DOCK_PICK LIFT UNDOCK_PICK NAV_TO_PLACE DOCK_PLACE LOWER RETREAT DONE" ]; then
  echo "ok   feedback states in order"; else echo "FAIL feedback states: $STATES"; FAILS=$((FAILS + 1)); fi
expect "unknown box aborts" '"status": "aborted".*unknown box' box_99 shelf_D
expect "occupied slot aborts" '"status": "aborted".*slot occupied: shelf_D slot 1 holds box_09' box_00 shelf_D 1

# two goals at once: the second must wait for the first (queue), then take slot 2
$MB box_00 shelf_D >"$ROOT/out/logs/check_task_manager_q1.log" 2>&1 &
Q1=$!
# wait until goal 1 is accepted, so the server sees them in this order
for _ in $(seq 1 100); do grep -q MOVE_BOX_ACCEPTED "$ROOT/out/logs/check_task_manager_q1.log" && break; sleep 0.1; done
$MB box_01 shelf_D >"$ROOT/out/logs/check_task_manager_q2.log" 2>&1 &
Q2=$!
wait $Q1; R1=$?; wait $Q2; R2=$?
if [ $R1 -eq 0 ] && [ $R2 -eq 0 ] \
   && grep -q '"final_slot": 0' "$ROOT/out/logs/check_task_manager_q1.log" \
   && grep -q '"final_slot": 2' "$ROOT/out/logs/check_task_manager_q2.log"; then
  echo "ok   two queued goals succeed (slots 0 and 2)"
else echo "FAIL queued goals (rc $R1 $R2)"; tail -n 2 "$ROOT"/out/logs/check_task_manager_q*.log; FAILS=$((FAILS + 1)); fi
ORDER=$(grep 'TASK_RESULT {"box_id": "box_0[01]"' "$TLOG" | grep '"success": true' | grep -o '"box_id": "box_0[01]"' | grep -o 'box_0[01]' | paste -sd' ')
[ "$ORDER" = "box_00 box_01" ] && echo "ok   queued goals ran in order ($ORDER)" \
  || { echo "FAIL queue order: $ORDER"; FAILS=$((FAILS + 1)); }
expect "shelf_D full -> no free slot" '"status": "aborted".*no free slot on shelf_D' box_02 shelf_D
expect "cancel mid-task" '"status": "canceled".*canceled before' box_02 shelf_A 0 --cancel-after 0.35
stop_tm

start_tm "${TLOG%.log}_fail.log" mock_fail:=navigate:unreachable || exit 1
expect "mocked nav failure aborts with reason" '"status": "aborted".*NAV_TO_PICK failed: unreachable' box_09 shelf_D
stop_tm

start_tm "${TLOG%.log}_missing.log" mock_missing:=box_03 || exit 1
expect "missing box aborts" '"status": "aborted".*box_03 is not on any shelf slot \(missing\)' box_03 shelf_D
expect "other box still moves (place_error 0)" '"status": "succeeded".*"place_error": 0.0' box_04 shelf_D
stop_tm

grep -qE "Traceback|\[ERROR\]" "$TLOG" "${TLOG%.log}_fail.log" "${TLOG%.log}_missing.log" && { echo "FAIL errors in node log"; FAILS=$((FAILS + 1)); }
[ $FAILS -eq 0 ] && { echo "check_task_manager: PASS"; exit 0; }
echo "check_task_manager: FAIL ($FAILS)"; exit 1
