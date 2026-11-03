"""Closed-loop waypoint driver on /odom -> /cmd_vel (rotate, then go).

  python3 scripts/waypoint_driver.py STEP [STEP ...]
  STEP = "x,y"  drive to that point in the odom frame (turn towards it first)
         "h:deg" turn in place to that absolute odom heading
Odom starts at the robot's pose at sim start (x forward). Prints
`WAYPOINT_DONE x= y= yaw=` at the end; exit 1 on timeout. Speeds stay well
below robot limits so the in-place turns don't slip much. Used by
scripts/check_slam.sh (Day 9) and the exploration route (Day 10).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

V_MAX, W_MAX = 0.4, 0.8
POS_TOL, YAW_TOL = 0.08, math.radians(3)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main(argv):
    steps = []
    for a in argv:
        if a.startswith("h:"):
            steps.append(("h", math.radians(float(a[2:]))))
        else:
            x, y = (float(v) for v in a.split(","))
            steps.append(("p", (x, y)))
    rclpy.init()
    node = rclpy.create_node("waypoint_driver")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    st = {}

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        st["pose"] = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    node.create_subscription(Odometry, "odom", on_odom, 10)

    def cmd(v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        pub.publish(t)

    def turn_to(target, deadline):
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if "pose" not in st:
                continue
            e = wrap(target - st["pose"][2])
            if abs(e) < YAW_TOL:
                cmd(0, 0)
                return True
            cmd(0, max(-W_MAX, min(W_MAX, 2.0 * e)) if abs(e) > 0.15 else math.copysign(0.15, e))
        return False

    ok = True
    t_start = time.time()
    while "pose" not in st and time.time() - t_start < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "pose" not in st:
        print("WAYPOINT no /odom", flush=True)
        return 1
    for kind, val in steps:
        deadline = time.time() + 60
        if kind == "h":
            ok = turn_to(val, deadline) and ok
            continue
        gx, gy = val
        x, y, yaw = st["pose"]
        ok = turn_to(math.atan2(gy - y, gx - x), deadline) and ok
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            x, y, yaw = st["pose"]
            d = math.hypot(gx - x, gy - y)
            if d < POS_TOL:
                break
            e = wrap(math.atan2(gy - y, gx - x) - yaw)
            v = min(V_MAX, 0.8 * d) * max(0.0, math.cos(e))
            cmd(max(v, 0.05), max(-W_MAX, min(W_MAX, 1.5 * e)))
        else:
            ok = False
        cmd(0, 0)
        print(f"WAYPOINT reached ({gx:.2f},{gy:.2f}) at ({x:.2f},{y:.2f})", flush=True)
    for _ in range(5):
        cmd(0, 0)
        time.sleep(0.05)
    x, y, yaw = st["pose"]
    print(f"WAYPOINT_DONE ok={ok} x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
