#!/usr/bin/env bash
# Day 4 check: /cmd_vel on domain 21 drives the sim robot (OmniGraph
# ROS2SubscribeTwist -> DifferentialController -> ArticulationController).
# Publishes 0.5 m/s for 2 s, then 1 rad/s for 2 s, and checks the ground-truth
# pose (POSE lines from sim/run_sim.py). Exit 0 = PASS. Log: out/logs/check_cmd_vel.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_cmd_vel.log"

timeout --kill-after=20 600 python "$ROOT/sim/run_sim.py" --seconds 120 --start=-2,0,0 >"$LOG" 2>&1 &
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
sleep 3
last_pose() { grep "^POSE" "$LOG" | tail -1; }
drive() { timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" "$@"; sleep 1; }
P0=$(last_pose); echo "start: $P0"
drive 0.5,0.0,2.0
P1=$(last_pose); echo "after forward: $P1"
drive 0.0,1.0,2.0
P2=$(last_pose); echo "after turn: $P2"

python3 - "$P0" "$P1" "$P2" <<'PY'
import math, sys
def parse(line):
    d = dict(kv.split("=") for kv in line.split()[1:])
    return float(d["x"]), float(d["y"]), float(d["yaw"])
(x0, y0, a0), (x1, y1, a1), (x2, y2, a2) = map(parse, sys.argv[1:])
fwd, lat = x1 - x0, y1 - y0
dyaw = (a2 - a1 + 180) % 360 - 180
drift = math.hypot(x2 - x1, y2 - y1)
ok = 0.8 <= fwd <= 1.2 and abs(lat) < 0.1 and 90 <= dyaw <= 140 and drift < 0.15
print(f"CMDVEL forward dx={fwd:.3f} dy={lat:.3f} (expect ~1.0) | turn dyaw={dyaw:.1f}deg "
      f"(expect ~115) drift={drift:.3f}")
print("check_cmd_vel: PASS" if ok else "check_cmd_vel: FAIL")
sys.exit(0 if ok else 1)
PY
RC=$?
[ $RC -ne 0 ] && { grep -E "Error|Traceback|error" "$LOG" | head -10; }
exit $RC
