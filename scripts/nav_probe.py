"""Nav2 probe (Days 12-13): lifecycle states, AMCL vs ground truth, goals.

  python3 scripts/nav_probe.py --sim-log out/logs/x_sim.log [--start=-2,0,0]
      [--goal x,y,yaw_deg ...] [--max-amcl-err 0.2] [--max-goal-err 0.25]
      [--localizer amcl|slam]

Needs the sim (run_sim.py prints ground-truth `POSE` lines into --sim-log) and
warehouse_bringup nav2.launch.py. Goals are in the map frame, whose origin is
the sim start pose. Prints `NAV_PROBE {...}` JSON lines plus ok/FAIL lines and
exits 0 only if every check passes:
  - all Nav2 lifecycle nodes report `active`
  - AMCL pose within --max-amcl-err of ground truth (at start and after each goal)
  - each goal SUCCEEDED and the true final position is within --max-goal-err
With --localizer slam (Day 14: nav2.launch.py slam:=true) slam_toolbox replaces
map_server + AMCL, the localisation estimate is TF map->base_link, and the
known area of /map must grow during the goals.
"""
import argparse
import json
import math
import re
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

NODES = ["map_server", "amcl", "controller_server", "planner_server",
         "behavior_server", "bt_navigator", "waypoint_follower"]
POSE_RE = re.compile(r"^POSE t=(\S+) x=(\S+) y=(\S+) yaw=(\S+)")

fails = []


def need(cond, msg):
    print(("ok   " if cond else "FAIL ") + msg, flush=True)
    if not cond:
        fails.append(msg)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def gt_map_pose(sim_log, start):
    """Last ground-truth pose from the sim log, converted to the map frame."""
    last = None
    with open(sim_log) as f:
        for line in f:
            m = POSE_RE.match(line)
            if m:
                last = m
    if last is None:
        return None
    x, y, yaw = float(last.group(2)), float(last.group(3)), math.radians(float(last.group(4)))
    sx, sy, syaw = start
    dx, dy = x - sx, y - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    return (c * dx - s * dy, s * dx + c * dy, wrap(yaw - syaw))


class Probe(BasicNavigator):
    def __init__(self, localizer):
        super().__init__("nav_probe")
        self.localizer = localizer
        self._amcl = None
        self.map_known = None
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_amcl, qos)
        self.create_subscription(OccupancyGrid, "map", self._on_map, qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_map(self, m):
        self.map_known = sum(1 for v in m.data if v >= 0) * m.info.resolution ** 2

    @property
    def amcl(self):
        """Localisation estimate in the map frame (AMCL pose or TF map->base_link)."""
        if self.localizer == "amcl":
            return self._amcl
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def _on_amcl(self, m):
        p = m.pose.pose
        self._amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def lifecycle_state(self, name, timeout=5.0):
        cli = self.create_client(GetState, f"{name}/get_state")
        if not cli.wait_for_service(timeout_sec=timeout):
            return "no_service"
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result().current_state.label if fut.result() else "timeout"

    def settle(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def pose_msg(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def check_amcl(nav, args, start, label):
    nav.settle(args.settle)
    gt = gt_map_pose(args.sim_log, start)
    if nav.amcl is None or gt is None:
        need(False, f"{label}: amcl_pose {nav.amcl} / ground truth {gt} available")
        return
    err = math.hypot(nav.amcl[0] - gt[0], nav.amcl[1] - gt[1])
    yerr = math.degrees(abs(wrap(nav.amcl[2] - gt[2])))
    print("NAV_PROBE " + json.dumps({"step": label, "amcl": [round(v, 3) for v in nav.amcl],
                                     "gt": [round(v, 3) for v in gt], "amcl_err_m": round(err, 3),
                                     "amcl_err_deg": round(yerr, 1)}), flush=True)
    need(err < args.max_amcl_err, f"{label}: {nav.localizer} pose within {args.max_amcl_err} m of ground truth "
         f"(err {err:.3f} m, {yerr:.1f} deg)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim-log", required=True)
    ap.add_argument("--start", default="-2,0,0")
    ap.add_argument("--goal", action="append", default=[], help="x,y,yaw_deg in the map frame")
    ap.add_argument("--max-amcl-err", type=float, default=0.2)
    ap.add_argument("--max-goal-err", type=float, default=0.25)
    ap.add_argument("--goal-timeout", type=float, default=120.0)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--localizer", choices=["amcl", "slam"], default="amcl")
    args = ap.parse_args()
    sx, sy, syaw = (float(v) for v in args.start.split(","))
    start = (sx, sy, math.radians(syaw))

    rclpy.init()
    nav = Probe(args.localizer)
    nav.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    t0 = time.time()
    if args.localizer == "amcl":
        nav.setInitialPose(pose_msg(nav, 0.0, 0.0, 0.0))
        nav.waitUntilNav2Active()
        nodes = NODES
    else:
        nav.waitUntilNav2Active(localizer="slam_toolbox")
        nodes = ["slam_toolbox"] + NODES[2:]
    print(f"NAV_PROBE {json.dumps({'step': 'active', 'wait_s': round(time.time() - t0, 1)})}", flush=True)

    states = {n: nav.lifecycle_state(n) for n in nodes}
    print("NAV_PROBE " + json.dumps({"step": "lifecycle", "states": states}), flush=True)
    need(all(s == "active" for s in states.values()), f"all {len(nodes)} Nav2 lifecycle nodes active")
    check_amcl(nav, args, start, "start")
    known0 = nav.map_known

    for g in args.goal:
        gx, gy, gyaw = (float(v) for v in g.split(","))
        t1 = time.time()
        nav.goToPose(pose_msg(nav, gx, gy, math.radians(gyaw)))
        while not nav.isTaskComplete():
            if time.time() - t1 > args.goal_timeout:
                nav.cancelTask()
                break
        res = nav.getResult()
        dt = time.time() - t1
        nav.settle(args.settle)
        gt = gt_map_pose(args.sim_log, start)
        err = math.hypot(gt[0] - gx, gt[1] - gy)
        yerr = math.degrees(abs(wrap(gt[2] - math.radians(gyaw))))
        print("NAV_PROBE " + json.dumps({"step": f"goal {g}", "result": str(res), "seconds": round(dt, 1),
                                         "gt": [round(v, 3) for v in gt], "goal_err_m": round(err, 3),
                                         "goal_err_deg": round(yerr, 1)}), flush=True)
        need(res == TaskResult.SUCCEEDED, f"goal {g}: result {res} after {dt:.1f} s")
        need(err < args.max_goal_err, f"goal {g}: true position within {args.max_goal_err} m "
             f"(err {err:.3f} m, yaw err {yerr:.1f} deg)")
        check_amcl(nav, args, start, f"after goal {g}")

    if args.localizer == "slam":
        nav.settle(3.0)
        print("NAV_PROBE " + json.dumps({"step": "map", "known_m2_start": known0,
                                         "known_m2_end": nav.map_known}), flush=True)
        need(known0 is not None and nav.map_known is not None and nav.map_known > known0 + 5.0,
             f"/map grew while navigating ({known0} -> {nav.map_known} m^2)")
    nav.destroy_node()
    rclpy.shutdown()
    print("NAV_PROBE_DONE " + ("PASS" if not fails else f"FAIL ({len(fails)})"), flush=True)
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
