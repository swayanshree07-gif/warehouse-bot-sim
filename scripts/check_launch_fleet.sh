#!/usr/bin/env bash
# Day 26 check: one command runs the whole fleet demo.
#   scripts/launch_all.sh --fleet --tasks --record DIR --media DIR
#  1. FLEET_READY, then the 6 demo tasks finish (FLEET_TASKS rc=0)
#  2. ALL_STOPPED and no sim / Kit / ROS processes of this project are left
#  3. bag < 300 MB with namespaced topics for all 3 robots + /fleet/status, /boxes
#  4. overview + chase frames and mp4s pass scripts/check_media.py
# Exit 0 = PASS. Bounded by timeout 900. Logs: out/logs/check_launch_fleet*.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh" >/dev/null
source "$ROOT/scripts/lib.sh"
L="$ROOT/out/logs"; mkdir -p "$L"
BAG="$ROOT/out/bags/check_launch_fleet"; MEDIA="$ROOT/out/media/check_launch_fleet"
rm -rf "$BAG" "$MEDIA"
FAILS=0
ok() { echo "ok   $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
timeout --kill-after=30 870 bash "$ROOT/scripts/launch_all.sh" --fleet --seconds 780 --tasks \
  --record "$BAG" --media "$MEDIA" >"$L/check_launch_fleet.log" 2>&1
grep -E "^(ALL|FLEET)" "$L/check_launch_fleet.log"
grep -q "FLEET_READY" "$L/check_launch_fleet.log" && ok "fleet ready" || fail "no FLEET_READY"
grep -q "FLEET_TASKS rc=0" "$L/check_launch_fleet.log" && ok "6 tasks done" || fail "tasks not all done"
grep -q "ALL_STOPPED" "$L/check_launch_fleet.log" && ok "stopped" || fail "no ALL_STOPPED"
sleep 2
LEFT=$( (list_sim_leftovers; pgrep -af "fleet_localization[.]launch|scripts/fleet_isaa[c][.]py|scripts/gripper_cm[d][.]py|ros2 bag recor[d]|__ns:=/robot" | grep -v -E "pgrep|bash -c") )
[ -z "$LEFT" ] && ok "no leftover processes" || { fail "leftovers:"; echo "$LEFT"; }
NEED=/clock,/tf,/map,/boxes,/fleet/status
for r in robot1 robot2 robot3; do NEED="$NEED,/$r/odom,/$r/scan,/$r/cmd_vel,/$r/amcl_pose"; done
if /usr/bin/python3 "$ROOT/scripts/check_bag.py" "$BAG" --max-mb 300 --need "$NEED" >"$L/check_launch_fleet_bag.log" 2>&1
then ok "$(tail -1 "$L/check_launch_fleet_bag.log") ($(du -sh "$BAG" | cut -f1))"
else fail "bag:"; cat "$L/check_launch_fleet_bag.log"; fi
if timeout 120 /usr/bin/python3 "$ROOT/scripts/check_media.py" "$MEDIA" --min-frames 40 --need-video \
    >"$L/check_launch_fleet_media.log" 2>&1
then ok "$(tail -1 "$L/check_launch_fleet_media.log")"
else fail "media:"; cat "$L/check_launch_fleet_media.log"; fi
[ $FAILS = 0 ] && { echo PASS; exit 0; } || { echo "FAIL ($FAILS)"; exit 1; }
