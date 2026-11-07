#!/usr/bin/env bash
# Day 26 check: headless camera capture -> PNG frames -> mp4.
#   scripts/check_media.sh
# Starts the sim with --capture out/media/check_media (overview + chase camera,
# 480x270, one frame per 0.5 sim-s), drives the robot over /cmd_vel (forward
# 1 m, turn, forward), stops the sim, then
#  1. the sim log has CAPTURE_DONE with no blank frames,
#  2. scripts/make_video.sh builds overview.mp4, chase.mp4 and demo.mp4,
#  3. scripts/check_media.py passes (>= 20 frames per camera, no blank ones,
#     first/last frame differ, videos readable by ffprobe with the right size),
#  4. no sim/Kit processes are left.
# Exit 0 = PASS. Log: out/logs/check_media.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_media.log"
OUT="$ROOT/out/media/check_media"
rm -rf "$OUT"   # previous run of this check (always under out/media/)

FAILS=0
ok() { echo "ok   $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }

timeout --kill-after=20 300 python "$ROOT/sim/run_sim.py" --seconds 60 --start=-2,0,0 \
  --capture "$OUT" --capture-every 0.5 >"$LOG" 2>&1 &
SIM_PID=$!
cleanup() {
  kill -TERM "$SIM_PID" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "$SIM_PID" 2>/dev/null || break; sleep 1; done
  pkill -9 -f "warehouse-bot-sim/sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 300); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "sim never became ready"; exit 1; }
grep -q "^CAPTURE_ON" "$LOG" && ok "$(grep '^CAPTURE_ON' "$LOG")" || fail "no CAPTURE_ON line"
sleep 2
timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" 0.5,0.0,2.0 0.0,1.0,2.0 0.4,0.0,3.0 >/dev/null 2>&1 \
  && ok "drove forward, turned, drove forward" || fail "cmd_vel_driver"
sleep 2
cleanup
trap - EXIT
DONE=$(grep '^CAPTURE_DONE' "$LOG")
if [ -n "$DONE" ] && echo "$DONE" | grep -q "blank=0"; then ok "$DONE"; else fail "capture: ${DONE:-no CAPTURE_DONE line}"; fi
LEFT="$(list_sim_leftovers)"
[ -z "$LEFT" ] && ok "no sim/Kit processes left" || { fail "leftover processes:"; echo "$LEFT"; }

bash "$ROOT/scripts/make_video.sh" "$OUT" >"$ROOT/out/logs/check_media_video.log" 2>&1 \
  && ok "$(tail -1 "$ROOT/out/logs/check_media_video.log")" \
  || fail "make_video: $(tail -1 "$ROOT/out/logs/check_media_video.log")"
if /usr/bin/python3 "$ROOT/scripts/check_media.py" "$OUT" --min-frames 20 --need-video \
    >"$ROOT/out/logs/check_media_frames.log" 2>&1; then
  ok "$(tail -1 "$ROOT/out/logs/check_media_frames.log")"
else
  fail "check_media.py:"; cat "$ROOT/out/logs/check_media_frames.log"
fi
[ $FAILS -eq 0 ] && { echo "check_media: PASS"; exit 0; }
echo "check_media: FAIL ($FAILS)"; exit 1
