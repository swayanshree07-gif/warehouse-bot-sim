#!/usr/bin/env bash
# Day 24 check: the single launch script brings the whole stack up.
#   scripts/check_launch_all.sh [--slam | --supervise]
#  1. scripts/launch_all.sh prints ALL_READY (its own health check passed)
#  2. scripts/health_check.py passes again, run from outside
#  3. one MoveBox task (box_09 -> shelf_D, auto slot 1) succeeds with
#     place_error < 0.05 m, and /boxes shows box_09 on shelf_D:1
#  4. SIGTERM to launch_all.sh -> ALL_STOPPED within 120 s, and no sim, Kit,
#     Nav2, slam_toolbox, task_manager or rosbag processes are left
#  5. (Day 25) the run was recorded with --record into out/bags/<check name>:
#     scripts/check_bag.py passes (< 300 MB, all key topics have messages)
#     and `ros2 bag info` can read it
#  6. (Day 26) camera frames were captured with --media into
#     out/media/<check name>: scripts/check_media.py passes (frames, not blank,
#     robot moved, overview/chase/demo.mp4 readable)
# --supervise instead checks the supervision path: launch_all.sh with a 45 s
# sim run time must notice the sim exiting by itself, stop Nav2 and the
# task_manager, print ALL_STOPPED and leave no processes behind.
# Exit 0 = PASS. Logs: out/logs/check_launch_all*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
L="$ROOT/out/logs"; mkdir -p "$L"
NAME=check_launch_all
SLAM=()
[ "$1" = "--slam" ] && { SLAM=(--slam); NAME=check_launch_all_slam; }
SUPERVISE=0
[ "$1" = "--supervise" ] && { SUPERVISE=1; NAME=check_launch_all_supervise; }
ALOG="$L/$NAME.log"
export LAUNCH_ALL_LOG_PREFIX="$L/$NAME"

FAILS=0
ok() { echo "ok   $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
ros_leftovers() {   # our ROS-side processes (bracket trick: never matches pgrep/this shell)
  pgrep -af "[w]arehouse_tasks/task_manager|nav2_[a-z_]+/[a-z_]+ |[s]lam_toolbox/async|[r]obot_state_publisher|[r]os2 bag record" \
    | grep -v -E "pgrep|bash -c" || true
}

if [ $SUPERVISE = 1 ]; then
  T0=$(date +%s)
  timeout 400 bash "$ROOT/scripts/launch_all.sh" --seconds 45 >"$ALOG" 2>&1
  RC=$?
  echo "     launch_all exited rc=$RC after $(( $(date +%s) - T0 )) s"
  grep -q "^ALL_READY" "$ALOG" && ok "ALL_READY" || fail "never ready"
  grep -q "^ALL sim exited" "$ALOG" && ok "noticed the sim exiting" || fail "no 'ALL sim exited' line"
  [ $RC = 0 ] && ok "exit code 0" || fail "exit code $RC"
  grep -q "^ALL_STOPPED" "$ALOG" && ok "$(grep '^ALL_STOPPED' "$ALOG")" || fail "no ALL_STOPPED line"
  sleep 2
  LEFT="$(list_sim_leftovers)$(ros_leftovers)"
  [ -z "$LEFT" ] && ok "no sim/Kit/ROS processes left" || { fail "leftover processes:"; echo "$LEFT"; }
  [ $FAILS -eq 0 ] && { echo "$NAME: PASS"; exit 0; }
  echo "$NAME: FAIL ($FAILS)"; exit 1
fi

BAG="$ROOT/out/bags/$NAME"
MEDIA="$ROOT/out/media/$NAME"
rm -rf "$BAG" "$MEDIA"   # previous run of this check (always under out/)
T0=$(date +%s)
bash "$ROOT/scripts/launch_all.sh" "${SLAM[@]}" --seconds 500 --record "$BAG" --media "$MEDIA" >"$ALOG" 2>&1 &
APID=$!
trap 'kill -TERM $APID 2>/dev/null; wait $APID 2>/dev/null' EXIT
for _ in $(seq 1 480); do
  grep -q -E "^ALL_READY|^ALL_STOPPED" "$ALOG" && break
  kill -0 "$APID" 2>/dev/null || break
  sleep 1
done
if grep -q "^ALL_READY" "$ALOG"; then
  ok "launch_all ALL_READY after $(( $(date +%s) - T0 )) s ($(grep '^HEALTH_PASS' "$ALOG"))"
else
  fail "launch_all not ready"; tail -20 "$ALOG"
fi

if [ $FAILS -eq 0 ]; then
  timeout 60 python3 "$ROOT/scripts/health_check.py" "${SLAM[@]}" --timeout 30 >"$L/${NAME}_health2.log" 2>&1 \
    && ok "health_check again: $(tail -1 "$L/${NAME}_health2.log")" \
    || { fail "health_check again"; tail -2 "$L/${NAME}_health2.log"; }
  T1=$(date +%s)
  timeout 420 python3 "$ROOT/scripts/move_box.py" box_09 shelf_D --timeout 400 >"$L/${NAME}_move.log" 2>&1
  RC=$?
  RES=$(grep '^MOVE_BOX_RESULT' "$L/${NAME}_move.log")
  echo "     $RES ($(( $(date +%s) - T1 )) s)"
  E=$(echo "$RES" | sed -n 's/.*"place_error": \([-0-9.e]*\).*/\1/p')
  [ $RC = 0 ] && echo "$RES" | grep -q '"final_shelf": "shelf_D", "final_slot": 1' \
    && /usr/bin/python3 -c "import sys; e=float('${E:--1}'); sys.exit(0 if 0 <= e < 0.05 else 1)" \
    && ok "move_box box_09 -> shelf_D:1 succeeded, place_error $E" || fail "move_box task"
  timeout 30 python3 "$ROOT/scripts/boxes_probe.py" --expect box_09=shelf_D:1 >"$L/${NAME}_probe.log" 2>&1 \
    && ok "/boxes shows box_09 on shelf_D:1, others untouched" \
    || { fail "/boxes after the task"; grep FAIL "$L/${NAME}_probe.log" | head -5; }
fi

T2=$(date +%s)
kill -TERM "$APID" 2>/dev/null
for _ in $(seq 1 120); do kill -0 "$APID" 2>/dev/null || break; sleep 1; done
if kill -0 "$APID" 2>/dev/null; then
  fail "launch_all still running 120 s after SIGTERM"; kill -9 "$APID"
fi
trap - EXIT
grep -q "^ALL_STOPPED" "$ALOG" && ok "$(grep '^ALL_STOPPED' "$ALOG") after $(( $(date +%s) - T2 )) s" \
  || fail "no ALL_STOPPED line"
sleep 2
LEFT="$(list_sim_leftovers)$(ros_leftovers)"
[ -z "$LEFT" ] && ok "no sim/Kit/ROS processes left" || { fail "leftover processes:"; echo "$LEFT"; }
grep -q Traceback "${LAUNCH_ALL_LOG_PREFIX}_tm.log" && fail "traceback in task_manager log"
NEED=""; [ ${#SLAM[@]} -eq 0 ] && NEED="--need $(/usr/bin/python3 -c 'import sys; sys.path.insert(0, "'"$ROOT"'/scripts"); import check_bag; print(check_bag.DEFAULT_NEED)'),/amcl_pose,/plan"
# shellcheck disable=SC2086
if timeout 30 /usr/bin/python3 "$ROOT/scripts/check_bag.py" "$BAG" $NEED >"$L/${NAME}_bag_check.log" 2>&1; then
  ok "$(tail -1 "$L/${NAME}_bag_check.log")"
else fail "bag: $(tail -1 "$L/${NAME}_bag_check.log")"; fi
timeout 30 ros2 bag info "$BAG" >"$L/${NAME}_bag_info.log" 2>&1 \
  && ok "ros2 bag info reads it ($(grep -m1 'Bag size' "$L/${NAME}_bag_info.log" | tr -s ' '))" \
  || fail "ros2 bag info"
if timeout 120 /usr/bin/python3 "$ROOT/scripts/check_media.py" "$MEDIA" --min-frames 40 --need-video \
    >"$L/${NAME}_media_check.log" 2>&1; then
  ok "$(tail -1 "$L/${NAME}_media_check.log")"
else fail "media:"; cat "$L/${NAME}_media_check.log"; fi
timeout 10 ros2 daemon stop >/dev/null 2>&1
[ $FAILS -eq 0 ] && { echo "$NAME: PASS"; exit 0; }
echo "$NAME: FAIL ($FAILS)"; exit 1
