#!/usr/bin/env python3
"""Send tasks to /fleet/tasks and wait on /fleet/status (Day 23).

Usage: python3 scripts/fleet_client.py [--timeout 90] "box_00 zone_E" "box_04 zone_E" ...
Prints FLEET_STATUS <json> (the final status) and exits 0 when every sent
task is done (or rejected, for --expect-rejected ones), 1 on timeout/failure.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tasks", nargs="+")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--expect-rejected", type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("fleet_client")
    pub = node.create_publisher(String, "/fleet/tasks", QoSProfile(depth=50, durability=DurabilityPolicy.VOLATILE))
    last = {}
    node.create_subscription(String, "/fleet/status", lambda m: last.update(json.loads(m.data)), 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    for t in a.tasks:
        pub.publish(String(data=t))
        rclpy.spin_once(node, timeout_sec=0.05)
    want_done = len(a.tasks) - a.expect_rejected
    ok = False
    while time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (last and last["done"] + last["failed"] >= want_done
                and len(last["rejected"]) >= a.expect_rejected and not last["pending"]
                and all(r["state"] == "idle" for r in last["robots"].values())):
            ok = last["failed"] == 0 and last["done"] == want_done
            break
    print("FLEET_STATUS " + json.dumps(last))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
