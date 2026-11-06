"""System health check for the full stack (Day 24).

  python3 scripts/health_check.py [--slam] [--timeout 120] [--no-tasks]

Polls until everything below is healthy or the timeout runs out:
  - topics deliver messages: /clock /odom /scan /joint_states /lift/height
    /boxes /map (/map is transient-local)
  - TF map -> base_link and odom -> base_link resolve
  - Nav2 lifecycle nodes are `active` (map_server + amcl, or slam_toolbox with
    --slam, plus controller/planner/behavior/bt_navigator/waypoint_follower)
  - services /gripper/attach and /gripper/detach exist
  - the /move_box action server is up (unless --no-tasks)
Prints one `HEALTH {json}` line with every item and `HEALTH_PASS` or
`HEALTH_FAIL <items>`. Exit 0 on pass. Needs `source env.sh` and a built ros_ws.
"""
import argparse
import json
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

NAVIGATION = ["controller_server", "planner_server", "behavior_server",
              "bt_navigator", "waypoint_follower"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slam", action="store_true")
    ap.add_argument("--no-tasks", action="store_true")
    ap.add_argument("--timeout", type=float, default=120.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("health_check", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    status = {}
    counts = {}

    def counter(name):
        counts[name] = 0

        def cb(_m):
            counts[name] += 1
        return cb

    topics = {"/clock": Clock, "/odom": Odometry, "/scan": LaserScan,
              "/joint_states": JointState, "/lift/height": Float64}
    for t, ty in topics.items():
        node.create_subscription(ty, t, counter(t), 10)
    from warehouse_interfaces.msg import BoxStates
    node.create_subscription(BoxStates, "/boxes", counter("/boxes"), 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", counter("/map"), latched)
    buf = Buffer()
    TransformListener(buf, node)

    lifecycle = (["slam_toolbox"] if a.slam else ["map_server", "amcl"]) + NAVIGATION
    lc_clients = {n: node.create_client(GetState, f"/{n}/get_state") for n in lifecycle}
    srv_clients = {s: node.create_client(Trigger, s) for s in ("/gripper/attach", "/gripper/detach")}
    act = None
    if not a.no_tasks:
        from warehouse_interfaces.action import MoveBox
        act = ActionClient(node, MoveBox, "/move_box")
    pending = {}

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def evaluate():
        for t in counts:
            status[f"topic {t}"] = counts[t] > 0
        for parent in ("map", "odom"):
            try:
                ok = buf.can_transform(parent, "base_link", Time())
            except Exception:  # noqa: BLE001
                ok = False
            status[f"tf {parent}->base_link"] = bool(ok)
        for n, cli in lc_clients.items():
            key = f"lifecycle {n}"
            if status.get(key) == "active":
                continue
            fut = pending.get(n)
            if fut is not None and fut.done():
                res = fut.result()
                status[key] = res.current_state.label if res else "no_response"
                pending.pop(n)
            elif fut is None:
                if cli.service_is_ready():
                    pending[n] = cli.call_async(GetState.Request())
                else:
                    status.setdefault(key, "no_service")
        for s, cli in srv_clients.items():
            status[f"service {s}"] = cli.service_is_ready()
        if act is not None:
            status["action /move_box"] = act.server_is_ready()

    def bad():
        return [k for k, v in status.items() if not (v is True or v == "active")]

    t0 = time.time()
    try:
        while True:
            spin(1.0)
            evaluate()
            if not bad():
                break
            if time.time() - t0 > a.timeout:
                break
        print("HEALTH " + json.dumps({"elapsed_s": round(time.time() - t0, 1), **status}), flush=True)
        b = bad()
        if b:
            print("HEALTH_FAIL " + ", ".join(b), flush=True)
            return 1
        print(f"HEALTH_PASS {len(status)} items in {time.time() - t0:.1f} s", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
