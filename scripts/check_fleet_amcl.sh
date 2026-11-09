#!/usr/bin/env bash
# Day 20 check: AMCL x3 on the saved map for the 3-robot Isaac scene.
# Starts sim/run_fleet_sim.py + fleet_localization.launch.py, reads
# /robotN/amcl_pose at start, then drives robot2 1 m forward and robot3 0.6 m
# backward and reads them again. PASS = every robot's AMCL pose is within 0.2 m
# (and 15 deg) of the sim ground truth (FPOSE, world -> map = +2 m in x) both
# times, and peak VRAM < 7000 MB. Logs: out/logs/check_fleet_amcl{,_launch}.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
source "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_fleet_amcl.log"
LLOG="$ROOT/out/logs/check_fleet_amcl_launch.log"

timeout --kill-after=20 600 python "$ROOT/sim/run_fleet_sim.py" --seconds 200 >"$LOG" 2>&1 &
SIM_PID=$!
LAUNCH_PID=""
cleanup() {
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  kill "$SIM_PID" 2>/dev/null; sleep 3
  [ -n "$LAUNCH_PID" ] && kill -9 "$LAUNCH_PID" 2>/dev/null
  pkill -9 -f "sim/run_fleet_si[m].py" 2>/dev/null
  pkill -f "fleet_localization[.]launch" 2>/dev/null
  pkill -f "__ns:=/robot[0-9]" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 600); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "sim never became ready"; exit 1; }
timeout 300 ros2 launch warehouse_bringup fleet_localization.launch.py >"$LLOG" 2>&1 &
LAUNCH_PID=$!
sleep 15

A0=$(timeout 40 python3 "$ROOT/scripts/amcl_probe.py" 30); echo "$A0"
G0=$(grep "^FPOSE" "$LOG" | tail -1); echo "$G0"
CMD_VEL_TOPIC=/robot2/cmd_vel timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" 0.5,0.0,2.0
CMD_VEL_TOPIC=/robot3/cmd_vel timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" -0.3,0.0,2.0
sleep 3
A1=$(timeout 40 python3 "$ROOT/scripts/amcl_probe.py" 30); echo "$A1"
G1=$(grep "^FPOSE" "$LOG" | tail -1); echo "$G1"
PEAK=$(grep -oE "VRAM_MB=[0-9]+" "$LOG" | cut -d= -f2 | sort -n | tail -1)
echo "peak VRAM: ${PEAK} MB"

PYTHONPATH="$ROOT/sim" python3 - "$A0" "$G0" "$A1" "$G1" "${PEAK:-99999}" <<'PY'
import math, sys
from fleet_layout import world_to_map
a0, g0, a1, g1, peak = sys.argv[1:]
def parse(line, skip):
    return {k: tuple(map(float, v.split(","))) for k, v in (kv.split("=") for kv in line.split()[skip:])}
ok = int(peak) < 7000
for label, a, g in (("start", a0, g0), ("after_drive", a1, g1)):
    am, gt = parse(a, 1), parse(g, 2)
    for ns in ("robot1", "robot2", "robot3"):
        if ns not in am:
            print(f"{label} {ns}: no amcl pose"); ok = False; continue
        mx, my = world_to_map(gt[ns][0], gt[ns][1])
        err = math.hypot(am[ns][0] - mx, am[ns][1] - my)
        dyaw = abs((am[ns][2] - gt[ns][2] + 180) % 360 - 180)
        good = err <= 0.2 and dyaw <= 15
        ok &= good
        print(f"{label} {ns}: err={err:.3f} m dyaw={dyaw:.1f} deg {'ok' if good else 'BAD'}")
print("check_fleet_amcl: PASS" if ok else "check_fleet_amcl: FAIL")
sys.exit(0 if ok else 1)
PY
RC=$?
[ $RC -ne 0 ] && grep -iE "error|warn" "$LLOG" | sort | uniq -c | head -15
exit $RC
