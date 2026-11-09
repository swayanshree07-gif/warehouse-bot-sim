#!/usr/bin/env bash
# Day 19 check: Isaac scene with 3 namespaced robots on domain 21.
# PASS = all 9 /robotN/{cmd_vel,odom,scan} topics present, robotN TF frames on /tf,
# driving /robot2/cmd_vel moves robot2 >= 0.5 m while robot1/robot3 move < 0.05 m,
# and peak GPU memory < 7000 MB. Log: out/logs/check_fleet_sim.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_fleet_sim.log"

timeout --kill-after=20 600 python "$ROOT/sim/run_fleet_sim.py" --seconds 150 >"$LOG" 2>&1 &
SIM_PID=$!
cleanup() {
  kill "$SIM_PID" 2>/dev/null; sleep 2
  pkill -9 -f "sim/run_fleet_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 600); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "sim never became ready"; exit 1; }
sleep 5
TOPICS=$(timeout 20 ros2 topic list --no-daemon 2>/dev/null)
echo "$TOPICS" | grep robot
TF=$(timeout 15 ros2 topic echo --once --no-daemon /tf tf2_msgs/msg/TFMessage 2>/dev/null; \
     timeout 15 ros2 topic echo --once --no-daemon /tf tf2_msgs/msg/TFMessage 2>/dev/null; \
     timeout 15 ros2 topic echo --once --no-daemon /tf tf2_msgs/msg/TFMessage 2>/dev/null)
FRAMES=$(echo "$TF" | grep -oE "robot[0-9]/[a-z_]+" | sort -u | tr '\n' ' ')
echo "tf frames seen: $FRAMES"
P0=$(grep "^FPOSE" "$LOG" | tail -1); echo "before: $P0"
CMD_VEL_TOPIC=/robot2/cmd_vel timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" 0.5,0.0,2.0
sleep 2
P1=$(grep "^FPOSE" "$LOG" | tail -1); echo "after:  $P1"
PEAK=$(grep -oE "VRAM_MB=[0-9]+" "$LOG" | cut -d= -f2 | sort -n | tail -1)
echo "peak VRAM: ${PEAK} MB"

python3 - "$P0" "$P1" "${PEAK:-99999}" "$TOPICS" "$FRAMES" <<'PY'
import math, sys
p0, p1, peak, topics, frames = sys.argv[1:]
def parse(line):
    return {k: tuple(map(float, v.split(","))) for k, v in (kv.split("=") for kv in line.split()[2:])}
a, b = parse(p0), parse(p1)
moved = {r: math.hypot(b[r][0] - a[r][0], b[r][1] - a[r][1]) for r in a}
want = [f"/robot{i}/{t}" for i in (1, 2, 3) for t in ("cmd_vel", "odom", "scan")]
tl = set(topics.split())
missing = [t for t in want if t not in tl]
fr = set(frames.split())
missing_tf = [f"robot{i}/base_link" for i in (1, 2, 3) if f"robot{i}/base_link" not in fr]
ok = (not missing and not missing_tf and moved["robot2"] >= 0.5
      and moved["robot1"] < 0.05 and moved["robot3"] < 0.05 and int(peak) < 7000)
print(f"FLEET topics 9/9={not missing} missing={missing} tf_missing={missing_tf} "
      f"moved={ {k: round(v, 3) for k, v in moved.items()} } peak_vram_mb={peak}")
print("check_fleet_sim: PASS" if ok else "check_fleet_sim: FAIL")
sys.exit(0 if ok else 1)
PY
RC=$?
[ $RC -ne 0 ] && { grep -E "Error|Traceback" "$LOG" | head -10; }
exit $RC
