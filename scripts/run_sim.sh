#!/usr/bin/env bash
# One-shot sim launcher (Day 7): warehouse + robot + ROS 2 bridge, headless.
#
#   scripts/run_sim.sh [--seconds N] [--start=x,y,yaw_deg] [other sim/run_sim.py args]
#
# Sources env.sh (domain 21, localhost), starts sim/run_sim.py under `timeout`
# (hard cap 900 s), logs to out/logs/run_sim.log, prints LAUNCH_READY when the
# sim plays, and on Ctrl-C / SIGTERM / exit stops the sim gracefully (SIGTERM
# to the sim, SIGKILL after 30 s) and removes Kit leftovers. Topics: /clock /odom /tf
# /tf_static /joint_states /scan (pub), /cmd_vel (sub).
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="${RUN_SIM_LOG:-$ROOT/out/logs/run_sim.log}"

SECONDS_ARG=600
PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --seconds) SECONDS_ARG="$2"; shift 2 ;;
    --seconds=*) SECONDS_ARG="${1#*=}"; shift ;;
    *) PASS+=("$1"); shift ;;
  esac
done
# CLAUDE.md: every sim run <= 15 min. Leave 60 s for startup + shutdown.
[ "$SECONDS_ARG" -gt 840 ] 2>/dev/null && SECONDS_ARG=840

timeout --kill-after=20 900 python "$ROOT/sim/run_sim.py" --seconds "$SECONDS_ARG" "${PASS[@]}" >"$LOG" 2>&1 &
SIM_PID=$!

STOPPING=0
stop_sim() {
  [ "$STOPPING" = 1 ] && return; STOPPING=1
  if kill -0 "$SIM_PID" 2>/dev/null; then
    echo "LAUNCH stopping sim (pid $SIM_PID)"
    # SIGTERM, not SIGINT: a backgrounded process may have SIGINT ignored.
    # timeout forwards it to python, whose handler leaves the loop cleanly.
    kill -TERM "$SIM_PID" 2>/dev/null
    for _ in $(seq 1 30); do kill -0 "$SIM_PID" 2>/dev/null || break; sleep 1; done
    kill -9 "$SIM_PID" 2>/dev/null
  fi
  pkill -9 -f "warehouse-bot-sim/sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
  echo "LAUNCH stopped ($(grep -o 'SIM_DONE.*' "$LOG" | tail -1))"
}
trap stop_sim EXIT
trap 'exit 130' INT TERM

echo "LAUNCH sim pid $SIM_PID, log $LOG, run time ${SECONDS_ARG}s, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
for _ in $(seq 1 300); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "LAUNCH sim exited early:"; tail -20 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "LAUNCH sim never became ready"; exit 1; }
echo "LAUNCH_READY"
wait "$SIM_PID"
RC=$?
echo "LAUNCH sim exited rc=$RC"
exit "$RC"
