"""Listen to /odom, /tf and /joint_states for a while and print one JSON line.

  python3 scripts/odom_probe.py [--seconds 3]
Output: `ODOM_PROBE {...}` with message rates (per wall-clock second), the last
odom pose, the odom->base_link transform looked up through tf2 and the last
joint positions, plus `joint_travel`: per-joint position change over the
window, unwrapped (PhysX reports revolute positions wrapped into [-2pi, 2pi]).
Used by scripts/check_odom.sh (Day 5).
"""
import argparse
import json
import math
import sys
import time

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--wait", type=float, default=20.0, help="max wait for first /odom")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("odom_probe")
    st = {"odom": 0, "tf": 0, "joints": 0}
    counting = [False]
    last = {}

    def on_odom(m):
        st["odom"] += 1
        p, q = m.pose.pose.position, m.pose.pose.orientation
        last["odom"] = {"x": p.x, "y": p.y, "yaw": yaw_of(q), "frame": m.header.frame_id,
                        "child": m.child_frame_id, "vx": m.twist.twist.linear.x,
                        "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}

    def on_tf(m):
        if any(t.child_frame_id == "base_link" for t in m.transforms):
            st["tf"] += 1

    travel = {}

    def on_js(m):
        st["joints"] += 1
        cur = dict(zip(m.name, m.position))
        prev = last.get("joints")
        if prev is not None and counting[0]:
            for n, v in cur.items():
                if n in prev:
                    d = v - prev[n]
                    if "wheel" in n:  # strip wrap jumps (multiples of 2pi)
                        d -= round(d / (2 * math.pi)) * 2 * math.pi
                    travel[n] = travel.get(n, 0.0) + d
        last["joints"] = cur

    node.create_subscription(Odometry, "odom", on_odom, 10)
    node.create_subscription(TFMessage, "tf", on_tf, 50)
    node.create_subscription(JointState, "joint_states", on_js, 10)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    t0 = time.time()
    while st["odom"] == 0 and time.time() - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    for k in st:
        st[k] = 0
    counting[0] = True
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    dt = time.time() - t0
    out = {"hz": {k: round(v / dt, 1) for k, v in st.items()}, **last, "joint_travel": travel}
    try:
        tr = buf.lookup_transform("odom", "base_link", Time()).transform
        out["tf_lookup"] = {"x": tr.translation.x, "y": tr.translation.y, "yaw": yaw_of(tr.rotation)}
    except Exception as e:  # noqa: BLE001
        out["tf_lookup"] = {"error": str(e)}
    print("ODOM_PROBE " + json.dumps(out), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if st["odom"] else 1


if __name__ == "__main__":
    sys.exit(main())
