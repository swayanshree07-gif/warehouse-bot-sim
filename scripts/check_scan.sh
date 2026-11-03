#!/usr/bin/env bash
# Day 6 check: the PhysX raycast lidar publishes /scan (+ static TF
# base_link->laser) and its ranges match a 2D ground-truth raycast of the
# layout (sim/geometry2d.py) within 5 cm, at the start pose and after driving.
# Exit 0 = PASS. Log: out/logs/check_scan.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_scan.log"
START="${START:--2,0,0}"

timeout --kill-after=20 600 python "$ROOT/sim/run_sim.py" --seconds 150 --start="$START" >"$LOG" 2>&1 &
SIM_PID=$!
cleanup() {
  kill "$SIM_PID" 2>/dev/null; sleep 2
  pkill -9 -f "sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 600); do
  grep -q "SIM_READY" "$LOG" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
grep -q "SIM_READY" "$LOG" || { echo "sim never became ready"; exit 1; }
sleep 3
last_pose() { grep "^POSE" "$LOG" | tail -1; }
probe() { timeout 60 python3 "$ROOT/scripts/scan_probe.py" --seconds 2 | grep "^SCAN_PROBE" | cut -d' ' -f2-; }
SA=$(probe); PA=$(last_pose)
timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" 0.5,0.0,2.0 0.0,1.0,1.0
sleep 1
SB=$(probe); PB=$(last_pose)
echo "gt A: $PA"; echo "gt B: $PB"

python3 - "$SA" "$SB" "$PA" "$PB" <<'PY'
import json, math, sys
sys.path.insert(0, "sim")
import geometry2d as G
import robot_params as P
fails = []
def need(cond, msg):
    print(("ok   " if cond else "FAIL ") + msg)
    if not cond: fails.append(msg)
def gt(line):
    d = dict(kv.split("=") for kv in line.split()[1:])
    return float(d["x"]), float(d["y"]), math.radians(float(d["yaw"]))
for tag, raw, pose in (("A", sys.argv[1], sys.argv[3]), ("B", sys.argv[2], sys.argv[4])):
    s = json.loads(raw or "{}")
    if not s.get("ranges"):
        need(False, f"{tag}: got a /scan message"); continue
    n = len(s["ranges"])
    need(n == len(P.laser_angles_deg()), f"{tag}: {n} beams, frame={s['frame']}")
    need(s["frame"] == "laser", f"{tag}: frame_id laser")
    need(s["hz"] >= 5, f"{tag}: scan rate {s['hz']} Hz (target {P.LASER_RATE_HZ})")
    need(abs(s["angle_min"] + math.pi) < 0.02 and abs(s["angle_increment"] - math.radians(P.LASER_RES_DEG)) < 1e-4,
         f"{tag}: angle_min={s['angle_min']:.4f} inc={s['angle_increment']:.5f}")
    tf = s.get("tf_base_laser")
    need(isinstance(tf, list) and all(abs(a - b) < 1e-3 for a, b in zip(tf, P.LASER_POS)), f"{tag}: tf base_link->laser {tf}")
    x, y, yaw = gt(pose)
    lx = x + P.LASER_POS[0] * math.cos(yaw) - P.LASER_POS[1] * math.sin(yaw)
    ly = y + P.LASER_POS[0] * math.sin(yaw) + P.LASER_POS[1] * math.cos(yaw)
    angles = [s["angle_min"] + i * s["angle_increment"] for i in range(n)]
    exp = G.expected_scan(lx, ly, yaw, [math.degrees(a) for a in angles], P.laser_height(), P.LASER_RANGE[1])
    errs = [abs(r - e) for r, e in zip(s["ranges"], exp) if r is not None]
    good = sum(e < 0.05 for e in errs)
    frac = good / n
    errs.sort()
    med = errs[len(errs) // 2] if errs else 9
    worst = max(range(n), key=lambda i: abs((s["ranges"][i] or 99) - exp[i]))
    print(f"   {tag}: laser at ({lx:.2f},{ly:.2f}) yaw={math.degrees(yaw):.1f}; median err {med*100:.2f} cm; "
          f"worst beam {math.degrees(angles[worst]):.0f} deg got {s['ranges'][worst]} exp {exp[worst]:.3f}")
    for deg in (0, 90, -90, 180):
        i = min(range(n), key=lambda k: abs(math.degrees(angles[k]) - deg) if deg != 180 else abs(abs(math.degrees(angles[k])) - 180))
        print(f"   {tag}: beam {deg:4d} deg: got {s['ranges'][i]} expected {exp[i]:.3f}")
    need(frac >= 0.95 and med < 0.02, f"{tag}: {good}/{n} beams within 5 cm of ground truth")
print("check_scan: PASS" if not fails else "check_scan: FAIL")
sys.exit(1 if fails else 0)
PY
RC=$?
[ $RC -ne 0 ] && { grep -E "Error|Traceback|error" "$LOG" | head -10; }
exit $RC
