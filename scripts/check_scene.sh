#!/usr/bin/env bash
# Day 2 check: build + save warehouse USD headless, boxes must stay at rest.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_scene.log"
timeout --kill-after=20 600 python "$ROOT/sim/check_scene.py" "$@" >"$LOG" 2>&1
RC=$?
kill_kit_leftovers
grep -E "SCENE_|Traceback|Error:" "$LOG" | head -20
# app.close() can end the process with rc 0 before sys.exit: also require the OK marker
grep -q "^SCENE_OK" "$LOG" || [ $RC -ne 0 ] || RC=1
[ $RC -eq 0 ] && echo "check_scene: PASS" || { echo "check_scene: FAIL (rc=$RC)"; tail -15 "$LOG"; }
exit $RC
