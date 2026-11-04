"""Command the sim lift and virtual gripper over ROS 2 (Days 16-17).

  python3 scripts/gripper_cmd.py lift 0.25 [--tol 0.01] [--timeout 15]
      publish /lift/command and wait until /lift/height is within --tol;
      prints `LIFT_REACHED target= height=` (exit 0) or `LIFT_TIMEOUT` (exit 1)
  python3 scripts/gripper_cmd.py attach | detach
      call /gripper/attach or /gripper/detach (std_srvs/Trigger); prints
      `GRIPPER <verb> success=<0|1> message=<...>`, exit 0 only on success
Needs `source env.sh` (domain 21) and the sim running (sim/run_sim.py).
"""
import argparse
import sys
import time

import rclpy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


def do_lift(node, target, tol, timeout):
    pub = node.create_publisher(Float64, "lift/command", 10)
    st = {"h": None}
    node.create_subscription(Float64, "lift/height", lambda m: st.__setitem__("h", m.data), 10)
    t0 = time.time()
    last_pub = 0.0
    inside_since = None
    while time.time() - t0 < timeout:
        if time.time() - last_pub > 1.0:   # republish until the sim has it
            pub.publish(Float64(data=target))
            last_pub = time.time()
        rclpy.spin_once(node, timeout_sec=0.05)
        h = st["h"]
        if h is not None and abs(h - target) <= tol:
            inside_since = inside_since or time.time()
            if time.time() - inside_since > 0.5:   # settled, not just passing through
                print(f"LIFT_REACHED target={target:.3f} height={h:.4f} err={h - target:+.4f}")
                return 0
        else:
            inside_since = None
    print(f"LIFT_TIMEOUT target={target:.3f} height={st['h']}")
    return 1


def do_trigger(node, verb, timeout):
    cli = node.create_client(Trigger, f"gripper/{verb}")
    if not cli.wait_for_service(timeout_sec=timeout):
        print(f"GRIPPER {verb} success=0 message=service_unavailable")
        return 1
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None:
        print(f"GRIPPER {verb} success=0 message=timeout")
        return 1
    print(f"GRIPPER {verb} success={int(res.success)} message={res.message}")
    return 0 if res.success else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("verb", choices=["lift", "attach", "detach"])
    ap.add_argument("value", nargs="?", type=float)
    ap.add_argument("--tol", type=float, default=0.01)
    ap.add_argument("--timeout", type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("gripper_cmd")
    try:
        if a.verb == "lift":
            if a.value is None:
                ap.error("lift needs a height")
            return do_lift(node, a.value, a.tol, a.timeout)
        return do_trigger(node, a.verb, a.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
