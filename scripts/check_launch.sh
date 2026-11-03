#!/usr/bin/env bash
# Day 7 check: scripts/run_sim.sh brings the sim up with all topics, the robot
# can be driven, and SIGTERM shuts everything down cleanly (graceful SIM_DONE,
# no leftover sim/Kit processes). Exit 0 = PASS.
# Logs: out/logs/check_launch.log (launcher), out/logs/check_launch_sim.log (sim)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LLOG="$ROOT/out/logs/check_launch.log"
export RUN_SIM_LOG="$ROOT/out/logs/check_launch_sim.log"
rm -f "$RUN_SIM_LOG"

FAILS=0
ok()   { echo "ok   $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }

# Stopped with SIGTERM below: bash started in the background ignores SIGINT
# (and can't trap it), so Ctrl-C only works for a foreground launcher.
bash "$ROOT/scripts/run_sim.sh" --seconds 240 --start=-2,0,0 >"$LLOG" 2>&1 &
LPID=$!
cleanup() {
  kill -TERM "$LPID" 2>/dev/null; sleep 5
  pkill -9 -f "warehouse-bot-sim/sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 300); do
  grep -q "LAUNCH_READY" "$LLOG" && break
  kill -0 "$LPID" 2>/dev/null || break
  sleep 1
done
grep -q "LAUNCH_READY" "$LLOG" && ok "launcher ready" || { fail "launcher not ready"; cat "$LLOG"; exit 1; }
sleep 3

TOPICS=$(timeout 30 ros2 topic list --no-daemon --spin-time 3 2>&1)
for t in /clock /odom /tf /tf_static /joint_states /scan /cmd_vel; do
  echo "$TOPICS" | grep -qx "$t" && ok "topic $t listed" || fail "topic $t missing"
done
echo "$TOPICS" | grep -qx /scan || echo "ros2 topic list said: $TOPICS"
for t in /clock /odom /joint_states /scan; do
  timeout 20 ros2 topic echo --once --no-daemon "$t" >/dev/null 2>&1 && ok "message on $t" || fail "no message on $t"
done
timeout 20 ros2 topic echo --once --no-daemon --qos-durability transient_local --qos-reliability reliable /tf_static >/dev/null 2>&1 \
  && ok "message on /tf_static" || fail "no message on /tf_static"

# robot_state_publisher (Day 8) on top of the sim: wheel/lift TFs from the
# sim's /joint_states must resolve all the way from odom.
if [ -f "$ROOT/ros_ws/install/local_setup.bash" ]; then
  setsid timeout 120 ros2 launch warehouse_bringup description.launch.py >"$ROOT/out/logs/check_launch_rsp.log" 2>&1 &
  RSP=$!
  sleep 4
  for f in left_wheel_link right_wheel_link lift_link laser base_footprint; do
    timeout 15 python3 - "$f" <<'PY' && ok "TF odom->$f resolves (sim + robot_state_publisher)" || fail "TF odom->$f missing"
import sys, time, rclpy, tf2_ros
from rclpy.time import Time
rclpy.init(); n = rclpy.create_node("tf_probe"); b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)
    if b.can_transform("odom", sys.argv[1], Time()):
        sys.exit(0)
sys.exit(1)
PY
  done
  kill -INT -- -"$RSP" 2>/dev/null; sleep 2; kill -9 -- -"$RSP" 2>/dev/null
else
  fail "ros_ws not built (run scripts/build_ws.sh)"
fi

P0=$(grep "^POSE" "$RUN_SIM_LOG" | tail -1)
timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" 0.4,0.0,1.5 >/dev/null
sleep 1
P1=$(grep "^POSE" "$RUN_SIM_LOG" | tail -1)
python3 - "$P0" "$P1" <<'PY' && ok "teleop moved robot: $P0 -> $P1" || fail "robot did not move: $P0 -> $P1"
import math, sys
g = [dict(kv.split("=") for kv in s.split()[1:]) for s in sys.argv[1:3]]
d = math.hypot(float(g[1]["x"]) - float(g[0]["x"]), float(g[1]["y"]) - float(g[0]["y"]))
sys.exit(0 if 0.45 < d < 0.75 else 1)
PY

T0=$(date +%s)
kill -TERM "$LPID"
for _ in $(seq 1 90); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
DT=$(( $(date +%s) - T0 ))
kill -0 "$LPID" 2>/dev/null && fail "launcher still running ${DT}s after SIGTERM" || ok "launcher exited ${DT}s after SIGTERM"
grep -q "SIM_DONE reason=signal" "$RUN_SIM_LOG" && ok "sim stopped gracefully (SIM_DONE reason=signal)" \
  || fail "no graceful SIM_DONE in sim log"
LEFT=$(list_sim_leftovers)
[ -z "$LEFT" ] && ok "no leftover sim processes" || fail "leftovers: $LEFT"

[ "$FAILS" = 0 ] && echo "check_launch: PASS" || echo "check_launch: FAIL ($FAILS)"
[ "$FAILS" = 0 ]
