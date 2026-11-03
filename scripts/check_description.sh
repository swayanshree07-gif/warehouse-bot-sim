#!/usr/bin/env bash
# Day 8 check: ros_ws builds, `ros2 launch warehouse_bringup description.launch.py`
# publishes /robot_description and the static TFs of the URDF (base_link->laser
# at LASER_POS, base_link->base_footprint). No sim needed. Exit 0 = PASS.
# Log: out/logs/check_description.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_description.log"
/usr/bin/python3 "$ROOT/sim/urdf_gen.py" --check || exit 1
timeout 300 bash "$ROOT/scripts/build_ws.sh" >"$LOG" 2>&1 || { echo "colcon build FAILED"; tail -30 "$LOG"; exit 1; }
echo "ok   colcon build"
source "$ROOT/env.sh"   # also sources ros_ws/install
setsid timeout 120 ros2 launch warehouse_bringup description.launch.py use_sim_time:=false >>"$LOG" 2>&1 &
LPID=$!
cleanup() { kill -INT -- -"$LPID" 2>/dev/null; sleep 2; kill -9 -- -"$LPID" 2>/dev/null; true; }
trap cleanup EXIT
sleep 4
timeout 60 python3 - <<'PY'
import math, sys, time
sys.path.insert(0, "sim")
import rclpy, tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import robot_params as P
rclpy.init()
n = rclpy.create_node("description_probe")
desc = []
n.create_subscription(String, "robot_description", lambda m: desc.append(m.data),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
fails = []
def need(c, msg):
    print(("ok   " if c else "FAIL ") + msg)
    if not c: fails.append(msg)
need(bool(desc) and 'robot name="warehouse_bot"' in desc[-1], f"/robot_description received ({len(desc[-1]) if desc else 0} chars)")
def tr(a, b):
    try:
        t = buf.lookup_transform(a, b, Time()).transform.translation
        return (t.x, t.y, t.z)
    except Exception as e:
        return str(e)
for (a, b), exp in ((("base_link", "laser"), P.LASER_POS),
                    (("base_link", "base_footprint"), (0, 0, -P.chassis_centre_z())),
                    (("base_link", "caster_front_link"), (P.CASTER_X, 0, P.CASTER_RADIUS - P.chassis_centre_z()))):
    got = tr(a, b)
    need(isinstance(got, tuple) and all(abs(g - e) < 1e-4 for g, e in zip(got, exp)), f"TF {a}->{b} = {got} (expect {exp})")
print("check_description: PASS" if not fails else "check_description: FAIL")
sys.exit(1 if fails else 0)
PY
