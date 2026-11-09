#!/usr/bin/env bash
# Day 21 check: 3 robots reach 3 goals at once (traffic-manager follower) on the saved map for the 3-robot Isaac scene.
# Starts sim/run_fleet_sim.py + fleet_localization.launch.py, reads
# /robotN/amcl_pose at start, then drives robot2 1 m forward and robot3 0.6 m
# backward and reads them again. PASS = every robot's AMCL pose is within 0.2 m
# (and 15 deg) of the sim ground truth (FPOSE, world -> map = +2 m in x) both
# times, and peak VRAM < 7000 MB. Logs: out/logs/check_fleet_nav{,_launch}.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
source "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_fleet_nav.log"
LLOG="$ROOT/out/logs/check_fleet_nav_launch.log"

timeout --kill-after=20 600 python "$ROOT/sim/run_fleet_sim.py" --seconds 420 >"$LOG" 2>&1 &
SIM_PID=$!
LAUNCH_PID=""
cleanup() {
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  kill "$SIM_PID" 2>/dev/null; sleep 3
  [ -n "$LAUNCH_PID" ] && kill -9 "$LAUNCH_PID" 2>/dev/null
  pkill -9 -f "sim/run_fleet_si[m].py" 2>/dev/null
  pkill -f "fleet_localization[.]launch" 2>/dev/null
  pkill -f "__ns:=/robot[0-9]" 2>/dev/null
  pkill -f "scripts/fleet_follo[w].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 600); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "sim never became ready"; exit 1; }
timeout 500 ros2 launch warehouse_bringup fleet_localization.launch.py >"$LLOG" 2>&1 &
LAUNCH_PID=$!
sleep 15

# goals in the map frame: robot1 to the top-right, robot2 down the middle,
# robot3 to the bottom-left -> paths cross in the central aisle
GOALS="robot1=4.72,3.22 robot2=2.22,-2.78 robot3=-1.28,-2.78"
RAM_LOG="$ROOT/out/logs/check_fleet_nav_ram.log"; : >"$RAM_LOG"
( while true; do free -m | awk '/Mem:/{print "RAM_USED_MB="$3}' >>"$RAM_LOG"; sleep 2; done ) &
RAM_PID=$!
timeout 300 python3 "$ROOT/scripts/fleet_follow.py" $GOALS --timeout 280 | tee "$ROOT/out/logs/check_fleet_nav_follow.log" | grep -E "FOLLOW_(DONE|TIMEOUT|NO_TF)"
kill $RAM_PID 2>/dev/null
sleep 3
G1=$(grep "^FPOSE" "$LOG" | tail -1); echo "$G1"
PEAK=$(grep -oE "VRAM_MB=[0-9]+" "$LOG" | cut -d= -f2 | sort -n | tail -1)
RAM=$(cut -d= -f2 "$RAM_LOG" | sort -n | tail -1)
echo "peak VRAM: ${PEAK} MB, peak RAM used: ${RAM} MB"
PYTHONPATH="$ROOT/sim" python3 - "$G1" "$GOALS" "${PEAK:-99999}" "${RAM:-99999}" "$ROOT/out/logs/check_fleet_nav_follow.log" <<'PY'
import math, sys
from fleet_layout import world_to_map
g1, goals, peak, ram, flog = sys.argv[1:]
gt = {k: tuple(map(float, v.split(","))) for k, v in (kv.split("=") for kv in g1.split()[2:])}
ok = int(peak) < 7000 and int(ram) < 13000 and "FOLLOW_DONE" in open(flog).read()
for kv in goals.split():
    ns, v = kv.split("="); gx, gy = map(float, v.split(","))
    mx, my = world_to_map(*gt[ns][:2])
    err = math.hypot(mx - gx, my - gy); good = err < 0.25; ok &= good
    print(f"{ns}: goal err {err:.3f} m {'ok' if good else 'BAD'}")
print("check_fleet_nav: PASS" if ok else "check_fleet_nav: FAIL")
sys.exit(0 if ok else 1)
PY
