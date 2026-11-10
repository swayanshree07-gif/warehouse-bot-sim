#!/usr/bin/env bash
# Day 24 check: traffic reservations in the Isaac run. robot1 and robot3 swap
# places head-on along the y=0 aisle (a guaranteed deadlock that the traffic
# manager must resolve by priority yield) while robot2 crosses down the middle.
# Each robot's next cell is only entered once it is reserved (FleetFollower).
# PASS = FOLLOW_DONE (no unresolved deadlock), >= 1 deadlock resolved, every
# robot within 0.25 m of its goal, ground-truth min pairwise separation over
# all FPOSE samples >= 0.45 m, peak VRAM < 7000 MB.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
source "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_fleet_traffic.log"
LLOG="$ROOT/out/logs/check_fleet_traffic_launch.log"

timeout --kill-after=20 600 python "$ROOT/sim/run_fleet_sim.py" --seconds 420 --pose-every 0.2 >"$LOG" 2>&1 &
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

GOALS="robot1=5.22,0.22 robot2=2.22,-2.78 robot3=-0.78,0.22"
RAM_LOG="$ROOT/out/logs/check_fleet_traffic_ram.log"; : >"$RAM_LOG"
( while true; do free -m | awk '/Mem:/{print "RAM_USED_MB="$3}' >>"$RAM_LOG"; sleep 2; done ) &
RAM_PID=$!
FLOG="$ROOT/out/logs/check_fleet_traffic_follow.log"
timeout 360 python3 "$ROOT/scripts/fleet_follow.py" $GOALS --timeout 330 | tee "$FLOG" | grep -E "FOLLOW_(DONE|TIMEOUT|NO_TF)"
kill $RAM_PID 2>/dev/null
sleep 3
PEAK=$(grep -oE "VRAM_MB=[0-9]+" "$LOG" | cut -d= -f2 | sort -n | tail -1)
RAM=$(cut -d= -f2 "$RAM_LOG" | sort -n | tail -1)
echo "peak VRAM: ${PEAK} MB, peak RAM used: ${RAM} MB"
PYTHONPATH="$ROOT/sim" python3 "$ROOT/scripts/eval_fleet_traffic.py" "$LOG" "$FLOG" "$GOALS" "${PEAK:-99999}" "${RAM:-99999}"
