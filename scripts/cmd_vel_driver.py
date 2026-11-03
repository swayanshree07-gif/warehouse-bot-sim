"""Publish a timed sequence of Twist commands on /cmd_vel, then stop.

  python3 scripts/cmd_vel_driver.py vx,wz,seconds [vx,wz,seconds ...]
Waits for a subscriber, publishes each segment at 20 Hz (wall clock) and sends
zero twists at the end. Used by scripts/check_cmd_vel.sh (Day 4).
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist


def main(argv):
    segs = [tuple(float(v) for v in a.split(",")) for a in argv]
    rclpy.init()
    node = rclpy.create_node("cmd_vel_driver")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0:
        if time.time() - t0 > 20:
            print("DRIVER no /cmd_vel subscriber", flush=True)
            return 1
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.5)

    def send(vx, wz, seconds):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(msg)
            time.sleep(0.05)

    for vx, wz, secs in segs:
        print(f"DRIVER segment vx={vx} wz={wz} for {secs}s", flush=True)
        send(vx, wz, secs)
    send(0.0, 0.0, 0.5)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
