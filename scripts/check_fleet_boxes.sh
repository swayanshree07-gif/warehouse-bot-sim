#!/usr/bin/env bash
# Day 22 check: per-robot box handling in the 3-robot Isaac scene.
# Spawns robot1/2/3 at the registry dock poses of box_00 (shelf_A:0),
# box_04 (shelf_B:1) and box_07 (shelf_C:1), then in parallel per robot:
#   /robotN/gripper/attach -> must return that robot's box,
#   /robotN/lift/command to LIFT_CLEARANCE, back out 0.6 m on /robotN/cmd_vel.
# While carrying, one /boxes message must show held=true, holder=robotN for
# exactly those 3 boxes. PASS = every box rose ~LIFT_CLEARANCE and followed its
# robot ~0.6 m, the other 7 boxes stayed put, a 2nd attach on robot1 fails,
# then all 3 detach and /boxes shows nothing held; peak VRAM < 7000 MB.
# Logs: out/logs/check_fleet_boxes*.log  (redirect this script's output to a file)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
source "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_fleet_boxes.log"
STEPS="$ROOT/out/logs/check_fleet_boxes_steps.log"; : >"$STEPS"

PAIRS="robot1=shelf_A:0:box_00 robot2=shelf_B:1:box_04 robot3=shelf_C:1:box_07"
STARTS="$(/usr/bin/python3 - "$ROOT" "$PAIRS" <<'PY'
import sys
sys.path.insert(0, sys.argv[1] + "/sim")
import registry_gen as R
reg = R.load(); ox, oy, _ = reg["map_origin_in_world"]
out = []
for kv in sys.argv[2].split():
    ns, v = kv.split("="); shelf, slot, _ = v.split(":")
    x, y, yaw = reg["shelves"][shelf]["slots"][int(slot)]["dock"]
    out.append(f"{ns}={x + ox:.3f},{y + oy:.3f},{yaw:.1f}")
print(" ".join(out))
PY
)"
echo "starts (world): $STARTS"

timeout --kill-after=20 600 python "$ROOT/sim/run_fleet_sim.py" --seconds 300 --box-every 0.5 \
    --starts $STARTS >"$LOG" 2>&1 &
SIM_PID=$!
cleanup() {
  kill "$SIM_PID" 2>/dev/null; sleep 3
  pkill -9 -f "sim/run_fleet_si[m].py" 2>/dev/null
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
mark() { echo "MARK $1 $(grep '^FBOXES' "$LOG" | tail -1)" >>"$STEPS"; }
G="timeout 40 python3 $ROOT/scripts/gripper_cmd.py"
CLEAR=$(/usr/bin/python3 -c "import sys; sys.path.insert(0, '$ROOT/sim'); import robot_params as P; print(P.LIFT_CLEARANCE)")
boxes_once() {   # print "id:holder" for every held box in one /boxes message
  timeout 20 python3 - <<'PY'
import rclpy
from warehouse_interfaces.msg import BoxStates
rclpy.init(); n = rclpy.create_node("fleet_boxes_probe"); got = []
n.create_subscription(BoxStates, "/boxes", lambda m: got.append(m), 10)
while not got:
    rclpy.spin_once(n, timeout_sec=0.5)
print("HELD " + " ".join(f"{b.id}:{b.holder}" for b in got[0].boxes if b.held))
PY
}
mark start

robot_pick() {   # robot_pick ns box  (runs in a subshell, writes its own log)
  local ns=$1 box=$2 out
  out=$($G attach --ns "$ns" 2>&1); echo "$ns attach: $out"
  echo "$out" | grep -q "success=1 message=$box" || { echo "FAIL $ns attach != $box"; return 1; }
  $G lift "$CLEAR" --ns "$ns" || { echo "FAIL $ns lift"; return 1; }
  sleep 1
  CMD_VEL_TOPIC="/$ns/cmd_vel" timeout 30 python3 "$ROOT/scripts/cmd_vel_driver.py" -0.2,0,3 || { echo "FAIL $ns back-out"; return 1; }
}
PIDS=()
for kv in $PAIRS; do
  ns=${kv%%=*}; box=${kv##*:}
  robot_pick "$ns" "$box" >"$ROOT/out/logs/check_fleet_boxes_$ns.log" 2>&1 &
  PIDS+=($!)
done
for p in "${PIDS[@]}"; do wait "$p" || FAILS=$((FAILS + 1)); done
cat "$ROOT"/out/logs/check_fleet_boxes_robot*.log
sleep 1.5; mark carried
HELD=$(boxes_once); echo "$HELD"
$G attach --ns robot1 >/dev/null 2>&1 && { echo "FAIL 2nd attach on robot1 succeeded"; FAILS=$((FAILS + 1)); } || echo "ok   2nd attach on robot1 refused"
for kv in $PAIRS; do
  ns=${kv%%=*}
  $G lift 0.0 --ns "$ns" >/dev/null || { echo "FAIL $ns lower"; FAILS=$((FAILS + 1)); }
  $G detach --ns "$ns" >/dev/null || { echo "FAIL $ns detach"; FAILS=$((FAILS + 1)); }
done
sleep 2; mark released
HELD2=$(boxes_once); echo "after detach: $HELD2"
PEAK=$(grep -oE "VRAM_MB=[0-9]+" "$LOG" | cut -d= -f2 | sort -n | tail -1)
echo "peak VRAM: ${PEAK} MB"

PYTHONPATH="$ROOT/sim" /usr/bin/python3 - "$STEPS" "$PAIRS" "$CLEAR" "$FAILS" "$HELD" "$HELD2" "${PEAK:-99999}" <<'PY'
import math, sys
from fleet_boxes import parse_fboxes
steps, pairs, clear, fails, held, held2, peak = sys.argv[1:]
marks = {}
for line in open(steps):
    p = line.split(maxsplit=2)
    if len(p) == 3 and p[2].startswith("FBOXES"):
        marks[p[1]] = parse_fboxes(p[2])
ok = int(fails) == 0 and int(peak) < 7000
def chk(c, m):
    global ok
    print(("ok   " if c else "FAIL ") + m); ok &= bool(c)
chk(all(k in marks for k in ("start", "carried", "released")), "FBOXES samples at start/carried/released")
if not ok:
    print("check_fleet_boxes: FAIL"); sys.exit(1)
s, c = marks["start"], marks["carried"]
want = {}
for kv in pairs.split():
    ns, v = kv.split("="); box = v.split(":")[2]; want[box] = ns
    p0, p1 = s["boxes"][box][0], c["boxes"][box][0]
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    chk(0.45 < moved < 0.75, f"{box} followed {ns}: moved {moved:.3f} m")
    chk(abs((p1[2] - p0[2]) - float(clear)) < 0.015, f"{box} raised dz={p1[2] - p0[2]:+.3f}")
    chk(c["boxes"][box][1] == ns and c["robots"][ns]["held"] == box, f"{box} holder={c['boxes'][box][1]}")
others = [b for b in s["boxes"] if b not in want]
worst = max(math.dist(s["boxes"][b][0], c["boxes"][b][0]) for b in others)
chk(worst < 0.01, f"other {len(others)} boxes undisturbed (max {worst * 1000:.1f} mm)")
got = dict(x.split(":") for x in held.split()[1:])
chk(got == want, f"/boxes holders while carrying {got}")
chk(held2.strip().endswith("HELD"), "/boxes: nothing held after detach")
chk(all(r["held"] == "-" for r in marks["released"]["robots"].values()), "all grippers empty")
print(f"peak VRAM {peak} MB")
print("check_fleet_boxes: PASS" if ok else "check_fleet_boxes: FAIL")
sys.exit(0 if ok else 1)
PY
