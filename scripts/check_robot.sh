#!/usr/bin/env bash
# Day 3 check: diff-drive robot drives 1 m, turns 90 deg, lifts, never flips.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_robot.log"
timeout --kill-after=20 600 python "$ROOT/sim/check_robot.py" "$@" >"$LOG" 2>&1
RC=$?
kill_kit_leftovers
grep -E "TRACE|ROBOT_|Traceback|Error:" "$LOG" | head -60
# app.close() can end the process with rc 0 before sys.exit: also require the OK marker
grep -q "^ROBOT_OK" "$LOG" || [ $RC -ne 0 ] || RC=1
[ $RC -eq 0 ] && echo "check_robot: PASS" || { echo "check_robot: FAIL (rc=$RC)"; tail -15 "$LOG"; }
exit $RC
