#!/usr/bin/env bash
# Day 1 check: headless Isaac Sim publishes /clock on ROS_DOMAIN_ID=21.
# Starts the sim (under timeout), waits for SMOKE_READY, then receives /clock
# messages with the ROS 2 CLI. Exit 0 = PASS. Log: out/logs/smoke_clock.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/smoke_clock.log"
SIM_SECONDS=${SIM_SECONDS:-60}

timeout --kill-after=20 900 python "$ROOT/sim/smoke_clock.py" --seconds "$SIM_SECONDS" >"$LOG" 2>&1 &
SIM_PID=$!
cleanup() {
  kill "$SIM_PID" 2>/dev/null; sleep 2
  pkill -9 -f "sim/smoke_cloc[k].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

# wait up to 12 min for the app to start (first run compiles shaders)
for _ in $(seq 1 720); do
  grep -q "SMOKE_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SMOKE_READY" "$LOG" || { echo "sim never became ready"; exit 1; }

echo "sim ready; listening for /clock on domain $ROS_DOMAIN_ID"
MSGS=$(timeout 30 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock 2>&1)
echo "$MSGS"
HZ=$(timeout 15 ros2 topic hz /clock --window 50 2>&1 | grep -m1 "average rate")
echo "${HZ:-no rate measured}"

wait "$SIM_PID"; RC=$?
grep -E "SMOKE_(STATS|OK|FAIL)" "$LOG"
# app.close() may exit with rc 0 before sys.exit, so also require SMOKE_OK
if echo "$MSGS" | grep -q "sec:" && [ $RC -eq 0 ] && grep -q "^SMOKE_OK" "$LOG"; then
  echo "smoke_clock: PASS"; exit 0
fi
echo "smoke_clock: FAIL (sim rc=$RC)"; tail -20 "$LOG"; exit 1
