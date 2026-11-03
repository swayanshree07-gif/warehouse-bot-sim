"""Grab the latest /map (+ TF map->odom) and print one JSON line (`MAP_PROBE {...}`).

  python3 scripts/map_probe.py [--wait 30]
Reports size, resolution, origin, counts of free (0) / occupied (>=65) /
unknown (-1) cells, the map stamp and the map->odom transform. Used by
scripts/check_slam.sh (Day 9).
"""
import argparse
import json
import sys
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=30.0)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("map_probe")
    maps = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "map", maps.append, qos)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    while not maps and time.time() - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(20):   # let TF arrive too
        rclpy.spin_once(node, timeout_sec=0.05)
    if not maps:
        print("MAP_PROBE {}", flush=True)
        return 1
    m = maps[-1]
    d = m.data
    out = {
        "width": m.info.width, "height": m.info.height, "resolution": round(m.info.resolution, 4),
        "origin": [m.info.origin.position.x, m.info.origin.position.y],
        "free": sum(1 for v in d if v == 0), "occupied": sum(1 for v in d if v >= 65),
        "unknown": sum(1 for v in d if v < 0), "frame": m.header.frame_id,
        "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
    }
    try:
        t = buf.lookup_transform("map", "odom", Time()).transform.translation
        out["tf_map_odom"] = [t.x, t.y, t.z]
    except Exception as e:  # noqa: BLE001
        out["tf_map_odom"] = str(e)
    print("MAP_PROBE " + json.dumps(out), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
