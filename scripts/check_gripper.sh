#!/usr/bin/env bash
# Days 16-17 check: lift command over ROS 2 and box attach/detach in the sim.
# The robot spawns at the registry dock pose of shelf_D slot 0 (box_09) and
#  1. lift -> 0.25 m and back to 0 via /lift/command, each within 1 cm
#  2. detach with nothing held must fail; attach must return box_09
#  3. lift to LIFT_CLEARANCE, back out 0.6 m: box_09 must follow (moved ~0.6 m,
#     raised ~LIFT_CLEARANCE), every other box stays put
#  4. drive to the dock pose of the empty shelf_D slot 1, lower, detach, back
#     out: box_09 must rest on the rack top at slot 1 (< 0.10 m from the slot
#     centre, z within 1 cm) and stay still
# Exit 0 = PASS. Logs: out/logs/check_gripper*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_gripper.log"
STEPS="$ROOT/out/logs/check_gripper_steps.log"
: >"$STEPS"

# world start pose = registry dock pose (map frame) + map origin
read -r START SLOT1 <<<"$(/usr/bin/python3 - "$ROOT" <<'PY'
import sys
sys.path.insert(0, sys.argv[1] + "/sim")
import registry_gen as R
reg = R.load()
ox, oy, _ = reg["map_origin_in_world"]
d = reg["shelves"]["shelf_D"]["slots"]
x, y, yaw = d[0]["dock"]
c = d[1]["center"]
print(f"{x + ox:.3f},{y + oy:.3f},{yaw:.1f} {c[0] + ox:.3f},{c[1] + oy:.3f},{c[2]:.3f}")
PY
)"
echo "start (world) $START, slot1 centre (world) $SLOT1"

timeout --kill-after=20 600 python "$ROOT/sim/run_sim.py" --seconds 240 --start="$START" \
    --box-every 0.5 >"$LOG" 2>&1 &
SIM_PID=$!
cleanup() {
  kill "$SIM_PID" 2>/dev/null; sleep 2
  pkill -9 -f "sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 600); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "sim never became ready"; exit 1; }
sleep 4

FAILS=0
step() {   # step <name> <expect rc 0|1> <cmd...>
  local name=$1 want=$2; shift 2
  local out rc
  out=$("$@" 2>&1); rc=$?
  echo "$out" | tail -2
  if { [ "$want" = 0 ] && [ $rc -eq 0 ]; } || { [ "$want" = 1 ] && [ $rc -ne 0 ]; }; then
    echo "ok   $name"
  else
    echo "FAIL $name (rc=$rc)"; FAILS=$((FAILS + 1))
  fi
}
mark() { echo "MARK $1 $(grep '^BOXES' "$LOG" | tail -1)" >>"$STEPS"; }
G="timeout 40 python3 $ROOT/scripts/gripper_cmd.py"
CLEAR=$(/usr/bin/python3 -c "import sys; sys.path.insert(0, '$ROOT/sim'); import robot_params as P; print(P.LIFT_CLEARANCE)")

mark start
step "lift to 0.25 m within 1 cm" 0 $G lift 0.25
step "lift back to 0 within 1 cm" 0 $G lift 0.0
step "detach with nothing held fails" 1 $G detach
step "attach box at dock" 0 $G attach
grep -q "GRIP_ATTACH ok=1 msg=box_09" "$LOG" && echo "ok   attached box_09" || { echo "FAIL attached box is not box_09"; FAILS=$((FAILS + 1)); }
step "attach while holding fails" 1 $G attach
step "lift to clearance $CLEAR" 0 $G lift "$CLEAR"
sleep 1; mark lifted
step "back out 0.6 m" 0 timeout 30 python3 "$ROOT/scripts/cmd_vel_driver.py" -0.2,0,3
sleep 1.5; mark carried
# odom frame = spawn pose: x = facing the rack, y = left. Slot 1 is 0.5 m to the
# robot's left (world +x for a robot facing -y).
step "drive to slot 1 dock" 0 timeout 90 python3 "$ROOT/scripts/waypoint_driver.py" -0.6,0.5 h:0 tol:0.02 0,0.5 h:0
sleep 1; mark at_slot1
echo "gt: $(grep '^POSE' "$LOG" | tail -1)"
step "lower lift" 0 $G lift 0.0
step "detach" 0 $G detach
step "back out after place" 0 timeout 30 python3 "$ROOT/scripts/cmd_vel_driver.py" -0.2,0,3
sleep 2; mark placed
sleep 2; mark placed_later

/usr/bin/python3 - "$STEPS" "$SLOT1" "$CLEAR" "$FAILS" <<'PY'
import math, sys
marks = {}
for line in open(sys.argv[1]):
    parts = line.split()
    if len(parts) < 3:
        continue
    d = {}
    for kv in parts[3:]:
        k, v = kv.split("=", 1)
        d[k] = tuple(float(x) for x in v.split(",")) if k.startswith("box_") else v
    marks[parts[1]] = d
need = ["start", "lifted", "carried", "at_slot1", "placed", "placed_later"]
missing = [m for m in need if m not in marks]
if missing:
    print(f"FAIL missing BOXES samples {missing}"); print("check_gripper: FAIL"); sys.exit(1)
sx, sy, sz = (float(v) for v in sys.argv[2].split(","))
clear, fails = float(sys.argv[3]), int(sys.argv[4])
ok = fails == 0
def chk(cond, msg):
    global ok
    print(("ok   " if cond else "FAIL ") + msg)
    ok &= cond
b = lambda m: marks[m]["box_09"]
s0, l1, c1, p1, p2 = b("start"), b("lifted"), b("carried"), b("placed"), b("placed_later")
chk(abs((l1[2] - s0[2]) - clear) < 0.01, f"box raised with lift: dz={l1[2] - s0[2]:+.3f} (want {clear})")
moved = math.hypot(c1[0] - s0[0], c1[1] - s0[1])
chk(0.45 < moved < 0.75, f"box followed robot back-out: moved {moved:.3f} m (want ~0.6), held={marks['carried']['held']}")
chk(marks["carried"]["held"] == "box_09", "held=box_09 while carrying")
err = math.hypot(p1[0] - sx, p1[1] - sy)
chk(err < 0.10, f"box placed at shelf_D slot 1: xy error {err:.3f} m")
chk(abs(p1[2] - sz) < 0.01, f"box rests on rack top: z={p1[2]:.3f} (want {sz:.3f})")
still = math.dist(p1, p2)
chk(still < 0.005, f"box stays put after detach/back-out: {still * 1000:.1f} mm in 2 s")
chk(marks["placed"]["held"] == "-", "held=- after detach")
others = [k for k in marks["start"] if k.startswith("box_") and k != "box_09"]
worst = max(math.dist(marks["start"][k], marks["placed_later"][k]) for k in others)
chk(worst < 0.01, f"other {len(others)} boxes undisturbed (max move {worst * 1000:.1f} mm)")
print("check_gripper: PASS" if ok else "check_gripper: FAIL")
sys.exit(0 if ok else 1)
PY
RC=$?
[ $RC -ne 0 ] && { grep -E "Error|Traceback|error" "$LOG" | head -10; }
exit $RC
