"""Grab /scan for a while and print one JSON line (`SCAN_PROBE {...}`).

  python3 scripts/scan_probe.py [--seconds 2]
Reports the scan rate, frame, angle_min/increment, range limits, the last
ranges and the static TF base_link->laser (via tf2). Used by check_scan.sh.
"""
import argparse
import json
import math
import sys
import time

import rclpy
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--wait", type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("scan_probe")
    st = {"n": 0, "msg": None}

    def on_scan(m):
        st["n"] += 1
        st["msg"] = m

    node.create_subscription(LaserScan, "scan", on_scan, qos_profile_sensor_data)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    while st["msg"] is None and time.time() - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    st["n"] = 0
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    m = st["msg"]
    if m is None:
        print("SCAN_PROBE {}", flush=True)
        return 1
    out = {
        "hz": round(st["n"] / (time.time() - t0), 1),
        "frame": m.header.frame_id,
        "angle_min": m.angle_min, "angle_max": m.angle_max, "angle_increment": m.angle_increment,
        "range_min": m.range_min, "range_max": m.range_max,
        "ranges": [r if math.isfinite(r) else None for r in m.ranges],
    }
    try:
        tr = buf.lookup_transform("base_link", "laser", Time()).transform.translation
        out["tf_base_laser"] = [tr.x, tr.y, tr.z]
    except Exception as e:  # noqa: BLE001
        out["tf_base_laser"] = str(e)
    print("SCAN_PROBE " + json.dumps(out), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
