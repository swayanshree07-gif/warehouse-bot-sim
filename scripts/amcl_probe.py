#!/usr/bin/env python3
"""Print the latest /robotN/amcl_pose of each fleet robot (Day 20).

  python3 scripts/amcl_probe.py [timeout_s]
Prints `AMCL robot1=x,y,yaw_deg robot2=... robot3=...` (map frame) and exits 0,
or exits 1 if some robot published no pose within the timeout.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

NS = ("robot1", "robot2", "robot3")


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = rclpy.create_node("amcl_probe")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    poses = {}

    def cb(ns):
        def f(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            poses[ns] = (p.x, p.y, yaw)
        return f
    subs = [node.create_subscription(PoseWithCovarianceStamped, f"/{ns}/amcl_pose", cb(ns), qos) for ns in NS]
    end = time.time() + timeout
    while time.time() < end and len(poses) < len(NS):
        rclpy.spin_once(node, timeout_sec=0.2)
    # a little longer to get the freshest pose
    t2 = time.time() + 1.0
    while time.time() < t2:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("AMCL " + " ".join("{}={:.3f},{:.3f},{:.1f}".format(ns, *poses[ns]) for ns in NS if ns in poses),
          flush=True)
    del subs
    node.destroy_node()
    rclpy.shutdown()
    return 0 if len(poses) == len(NS) else 1


if __name__ == "__main__":
    sys.exit(main())
