#!/usr/bin/env python3
"""Day 21: drive the 3 Isaac robots to goals with the traffic manager.

  python3 scripts/fleet_follow.py robot1=x,y robot2=x,y robot3=x,y [--timeout 240]
Goals are in the map frame. Poses come from TF map -> robotN/base_footprint
(AMCL map->odom + sim odom->base). Commands go to /robotN/cmd_vel at 10 Hz.
Prints FOLLOW_DONE ticks=.. secs=.. and exits 0, or FOLLOW_TIMEOUT and exits 1.
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros_ws/src/warehouse_tasks"))
from warehouse_tasks.path_follower import FleetFollower  # noqa: E402
from warehouse_tasks.traffic import Roadmap  # noqa: E402

MAP = os.path.join(ROOT, "ros_ws/src/warehouse_bringup/maps/warehouse.yaml")


def main():
    args = [a for a in sys.argv[1:] if "=" in a]
    timeout = float(sys.argv[sys.argv.index("--timeout") + 1]) if "--timeout" in sys.argv else 240.0
    goals = {k: tuple(map(float, v.split(","))) for k, v in (a.split("=") for a in args)}
    rclpy.init()
    node = rclpy.create_node("fleet_follow")
    buf = Buffer()
    TransformListener(buf, node)
    pubs = {n: node.create_publisher(Twist, f"/{n}/cmd_vel", 10) for n in goals}

    def poses():
        out = {}
        for n in goals:
            try:
                t = buf.lookup_transform("map", f"{n}/base_footprint", Time())
            except Exception:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            out[n] = (t.transform.translation.x, t.transform.translation.y, yaw)
        return out

    def send(cmds):
        for n, (v, w) in cmds.items():
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            pubs[n].publish(m)

    t0 = time.time()
    p = None
    while p is None and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.1)
        p = poses()
    if p is None:
        print("FOLLOW_NO_TF", flush=True)
        return 1
    ff = FleetFollower(Roadmap.from_map(MAP), {n: xy[:2] for n, xy in p.items()}, tol=0.1)
    for n, g in goals.items():
        ff.set_goal(n, g)
    rc, last = 1, 0.0
    while time.time() - t0 < timeout:
        end = time.time() + 0.1
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        p = poses() or p
        send(ff.step(p))
        if time.time() - last > 5:
            last = time.time()
            print("FOLLOW t=%.0f tick=%d " % (time.time() - t0, ff.ticks) + " ".join(
                "%s=%.2f,%.2f" % (n, x, y) for n, (x, y, _) in p.items()), flush=True)
        if ff.done():
            rc = 0
            break
    send({n: (0.0, 0.0) for n in goals})
    print(("FOLLOW_DONE" if rc == 0 else "FOLLOW_TIMEOUT") + " ticks=%d secs=%.1f deadlocks=%d" % (
        ff.ticks, time.time() - t0, ff.tm.deadlocks), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
