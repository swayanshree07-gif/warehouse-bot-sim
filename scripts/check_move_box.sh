#!/usr/bin/env bash
# Day 21 check: end-to-end MoveBox in the sim. Starts the sim (spawn world
# -2,0,0 = map origin), Nav2 + AMCL on the packaged map and the task_manager
# with backend:=ros, then sends MoveBox goals with scripts/move_box.py.
# Default goal: box_09 -> shelf_D (first free slot = 1). Passes if every goal
# succeeds and the sim's ground truth (BOXES line) has the box on the target
# slot: xy within 0.10 m of the slot centre, z within 1 cm of the rack top, not
# held, and all other boxes moved < 1 cm.
#   scripts/check_move_box.sh [BOX:SHELF[:SLOT] ...]
# Exit 0 = PASS. Logs: out/logs/check_move_box*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
NAME=check_move_box
LLOG="$ROOT/out/logs/$NAME.log"
export RUN_SIM_LOG="$ROOT/out/logs/${NAME}_sim.log"
NLOG="$ROOT/out/logs/${NAME}_nav2.log"
TLOG="$ROOT/out/logs/${NAME}_tm.log"
CLOG="$ROOT/out/logs/${NAME}_client.log"
: >"$CLOG"
[ -f "$ROOT/ros_ws/install/warehouse_tasks/share/warehouse_tasks/package.xml" ] \
  || { echo "ros_ws not built: run scripts/build_ws.sh"; exit 1; }
TASKS=("$@")
[ ${#TASKS[@]} -eq 0 ] && TASKS=(box_09:shelf_D)

bash "$ROOT/scripts/run_sim.sh" --seconds 600 --start=-2,0,0 --box-every 0.5 >"$LLOG" 2>&1 &
LPID=$!
NPID=""; TPID=""
cleanup() {
  [ -n "$TPID" ] && { kill -INT "$TPID" 2>/dev/null; sleep 3; kill -9 -- -"$TPID" 2>/dev/null; }
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
setsid timeout 640 ros2 launch warehouse_tasks task_manager.launch.py backend:=ros >"$TLOG" 2>&1 &
TPID=$!
# Nav2 is up when bt_navigator is active (lifecycle manager logs this)
for _ in $(seq 1 90); do
  grep -q "Managed nodes are active" "$NLOG" && break; sleep 1
done
grep -q "Managed nodes are active" "$NLOG" && echo "ok   nav2 active" || { echo "FAIL nav2 not active"; exit 1; }
sleep 3
grep '^BOXES' "$RUN_SIM_LOG" | tail -1 >"$ROOT/out/logs/${NAME}_before.txt"

FAILS=0
for t in "${TASKS[@]}"; do
  IFS=: read -r BOX SHELF SLOT <<<"$t"
  echo "task $BOX -> $SHELF ${SLOT:-auto}"
  T0=$(date +%s)
  timeout 420 python3 "$ROOT/scripts/move_box.py" "$BOX" "$SHELF" ${SLOT:+"$SLOT"} --timeout 400 >>"$CLOG" 2>&1
  RC=$?
  RES=$(grep '^MOVE_BOX_RESULT' "$CLOG" | tail -1)
  echo "     $RES ($(( $(date +%s) - T0 )) s)"
  [ $RC -eq 0 ] && echo "ok   $BOX succeeded" || { echo "FAIL $BOX (rc=$RC)"; FAILS=$((FAILS + 1)); }
  sleep 2
  grep '^BOXES' "$RUN_SIM_LOG" | tail -1 >"$ROOT/out/logs/${NAME}_after.txt"
  FINAL=$(echo "$RES" | sed -n 's/.*"final_shelf": "\([^"]*\)", "final_slot": \(-\?[0-9]*\).*/\1:\2/p')
  /usr/bin/python3 - "$ROOT" "$BOX" "$FINAL" "$ROOT/out/logs/${NAME}_before.txt" "$ROOT/out/logs/${NAME}_after.txt" <<'PY' || FAILS=$((FAILS + 1))
import math, sys
sys.path.insert(0, sys.argv[1] + "/sim")
import registry_gen as R
reg = R.load()
box, final = sys.argv[2], sys.argv[3]
def parse(path):
    parts = open(path).read().split()
    d = {}
    for kv in parts[1:]:
        k, v = kv.split("=", 1)
        d[k] = tuple(float(x) for x in v.split(",")) if k.startswith("box_") else v
    return d
before, after = parse(sys.argv[4]), parse(sys.argv[5])
ok = True
def chk(c, m):
    global ok
    print(("ok   " if c else "FAIL ") + m); ok &= c
if not final or ":" not in final or final.endswith(":-1"):
    chk(False, f"no final slot in result ({final!r})"); sys.exit(1)
shelf, slot = final.split(":")
ox, oy, _ = reg["map_origin_in_world"]
c = [s for s in reg["shelves"][shelf]["slots"] if s["index"] == int(slot)][0]["center"]
p = after[box]
err = math.hypot(p[0] - (c[0] + ox), p[1] - (c[1] + oy))
chk(err < 0.10, f"{box} at {shelf} slot {slot}: ground-truth xy error {err:.3f} m")
chk(abs(p[2] - c[2]) < 0.01, f"{box} rests on rack top z={p[2]:.3f} (want {c[2]:.3f})")
chk(after["held"] == "-", "gripper empty after task")
others = [k for k in before if k.startswith("box_") and k != box]
worst = max(math.dist(before[k], after[k]) for k in others)
chk(worst < 0.01, f"other boxes undisturbed (max move {worst * 1000:.1f} mm)")
sys.exit(0 if ok else 1)
PY
  cp "$ROOT/out/logs/${NAME}_after.txt" "$ROOT/out/logs/${NAME}_before.txt"
done
echo "gt: $(grep '^POSE' "$RUN_SIM_LOG" | tail -1)"
grep -E "TASK_RESULT" "$TLOG" | sed 's/.*TASK_RESULT/TASK_RESULT/'
[ $FAILS -eq 0 ] && { echo "$NAME: PASS"; exit 0; }
echo "$NAME: FAIL ($FAILS)"; exit 1
