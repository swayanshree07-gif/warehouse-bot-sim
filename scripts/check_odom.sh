#!/usr/bin/env bash
# Day 5 check: the sim publishes /odom, TF odom->base_link and /joint_states.
# Probes the topics while standing (rates >= 20 Hz), drives 0.5 m/s x 2 s then
# 1 rad/s x 1.5 s, probes again, and compares odom/TF/wheel joint deltas with the
# ground-truth POSE lines from sim/run_sim.py. Exit 0 = PASS.
# Log: out/logs/check_odom.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_odom.log"

timeout --kill-after=20 600 python "$ROOT/sim/run_sim.py" --seconds 150 --start=-2,0,0 >"$LOG" 2>&1 &
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
probe() { timeout 60 python3 "$ROOT/scripts/odom_probe.py" --seconds "$1" | grep "^ODOM_PROBE" | cut -d' ' -f2-; }

A=$(probe 3); P0=$(last_pose)
echo "probe A: $A"; echo "gt A: $P0"
# probe B listens during the whole drive (wheel joint travel is accumulated)
BF=$(mktemp); probe 12 >"$BF" &
PROBE_PID=$!
sleep 2
timeout 60 python3 "$ROOT/scripts/cmd_vel_driver.py" 0.5,0.0,2.0 0.0,1.0,1.5
wait "$PROBE_PID"; B=$(cat "$BF"); rm -f "$BF"; P1=$(last_pose)
echo "probe B: $B"; echo "gt B: $P1"

python3 - "$A" "$B" "$P0" "$P1" <<'PY'
import json, math, sys
sys.path.insert(0, "sim")
import robot_params as P
A, B = json.loads(sys.argv[1] or "{}"), json.loads(sys.argv[2] or "{}")
def gt(line):
    d = dict(kv.split("=") for kv in line.split()[1:])
    return float(d["x"]), float(d["y"]), float(d["yaw"])
g0, g1 = gt(sys.argv[3]), gt(sys.argv[4])
wrap = lambda a: (a + 180) % 360 - 180
fails = []
def need(cond, msg):
    print(("ok   " if cond else "FAIL ") + msg)
    if not cond: fails.append(msg)

for name, pr in (("A", A), ("B", B)):
    hz = pr.get("hz", {})
    need(all(hz.get(k, 0) >= 20 for k in ("odom", "tf", "joints")), f"rates {name} >= 20 Hz: {hz}")
oa, ob = A.get("odom", {}), B.get("odom", {})
jb = B.get("joints", {})
need(ob.get("frame") == "odom" and ob.get("child") == "base_link", f"odom frames {ob.get('frame')}->{ob.get('child')}")
need(math.hypot(oa.get("x", 9), oa.get("y", 9)) < 0.02, f"odom starts at origin: x={oa.get('x')} y={oa.get('y')}")
odx, ody = ob["x"] - oa["x"], ob["y"] - oa["y"]
gdx, gdy = g1[0] - g0[0], g1[1] - g0[1]
need(math.hypot(odx - gdx, ody - gdy) < 0.05, f"odom disp ({odx:.3f},{ody:.3f}) vs gt ({gdx:.3f},{gdy:.3f})")
odyaw, gdyaw = wrap(ob["yaw"] - oa["yaw"]), wrap(g1[2] - g0[2])
need(abs(odyaw - gdyaw) < 3, f"odom dyaw {odyaw:.1f} vs gt {gdyaw:.1f} deg")
need(math.hypot(odx, ody) > 0.8, f"robot actually moved {math.hypot(odx, ody):.3f} m")
tl = B.get("tf_lookup", {})
need("x" in tl and math.hypot(tl["x"] - ob["x"], tl["y"] - ob["y"]) < 0.02 and abs(wrap(tl["yaw"] - ob["yaw"])) < 1,
     f"tf2 odom->base_link {tl} matches odom")
need({"left_wheel_joint", "right_wheel_joint", "lift_joint"} <= set(jb), f"joint names {sorted(jb)}")
jt = B.get("joint_travel", {})
need("left_wheel_joint" in jt and "right_wheel_joint" in jt, "wheel joint travel recorded")
if "left_wheel_joint" in jt and "right_wheel_joint" in jt:
    dl, dr = jt["left_wheel_joint"], jt["right_wheel_joint"]
    # straight 1 m + in-place turn: mean wheel travel ~ 1 m, differential ~ dyaw
    travel = (dl + dr) * P.WHEEL_RADIUS / 2
    wyaw = math.degrees((dr - dl) * P.WHEEL_RADIUS / P.WHEEL_SEPARATION)
    need(0.8 < travel < 1.2, f"wheel joints mean travel {travel:.3f} m (dl={dl:.2f} dr={dr:.2f} rad)")
    need(abs(wyaw - gdyaw) < 20, f"wheel joints dyaw {wyaw:.1f} vs gt {gdyaw:.1f} deg (slip allowed)")
print("check_odom: PASS" if not fails else "check_odom: FAIL")
sys.exit(1 if fails else 0)
PY
RC=$?
[ $RC -ne 0 ] && { grep -E "Error|Traceback|error" "$LOG" | head -10; }
exit $RC
