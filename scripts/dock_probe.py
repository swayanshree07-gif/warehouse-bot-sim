"""Docking accuracy trials (plan Day 20).

  python3 scripts/dock_probe.py --sim-log out/logs/x_sim.log [--start=-2,0,0]
      [--slot shelf_A:1 ...] [--max-pos-err 0.03] [--max-yaw-err 3.0]

For each slot: Nav2 to the slot's approach pose, dock with the same RosRobot
code the task_manager uses (warehouse_tasks.ros_robot), settle 1 s, then compare
the sim's ground-truth pose (last `POSE` line in --sim-log, converted to the
map frame) with the registry dock pose. Then undock back to the approach pose.
No box is handled. Prints one `DOCK_TRIAL {json}` line per trial with the true
along-track / lateral / yaw errors and the robot's own estimate (TF
map->base_link, i.e. AMCL), plus ok/FAIL lines. Exit 0 only if every trial
docked and its true position error < --max-pos-err and yaw error < --max-yaw-err.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_probe import gt_map_pose, wrap  # noqa: E402
from warehouse_tasks.line_track import errors  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402
from warehouse_tasks.ros_robot import RosRobot  # noqa: E402
from warehouse_tasks.task_manager import default_registry  # noqa: E402

DEFAULT_SLOTS = ["shelf_A:1", "shelf_C:0", "shelf_B:2", "shelf_D:0", "shelf_A:2"]


def trial_errors(pose, dock):
    """True errors of pose (x, y, yaw_rad) w.r.t. dock (x, y, yaw_deg):
    (pos, along, lateral, yaw_deg). along > 0 = stopped short."""
    yaw = math.radians(dock[2])
    along, lat = errors(pose, dock[:2], yaw)
    return (math.hypot(pose[0] - dock[0], pose[1] - dock[1]), along, lat,
            math.degrees(wrap(pose[2] - yaw)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim-log", required=True)
    ap.add_argument("--start", default="-2,0,0")
    ap.add_argument("--slot", action="append", default=[])
    ap.add_argument("--max-pos-err", type=float, default=0.03)
    ap.add_argument("--max-yaw-err", type=float, default=3.0)
    ap.add_argument("--registry", default="")
    ap.add_argument("--no-lidar", action="store_true", help="AMCL-only dock (Day 20 baseline)")
    a = ap.parse_args()
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    start = (sx, sy, math.radians(syaw))
    reg = Registry.load(a.registry or default_registry())
    slots = a.slot or DEFAULT_SLOTS

    rclpy.init()
    node = Node("dock_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
    robot = RosRobot(node, registry=reg, lidar_dock=not a.no_lidar)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    fails = 0
    for _ in range(100):           # wait for AMCL's map->base_link
        if robot.pose() is not None:
            break
        time.sleep(0.1)
    for spec in slots:
        shelf, idx = spec.split(":")
        slot = [s for s in reg.shelves[shelf]["slots"] if s["index"] == int(idx)][0]
        approach, dock = slot["approach"], slot["dock"]
        t0 = time.time()
        rec = {"slot": spec, "dock": dock}
        ok, msg = robot.navigate(approach)
        if ok:
            ok, msg = robot.dock(dock)
        rec.update(docked=ok, msg=msg)
        if robot.last_dock:
            rec["lidar"] = {k: (round(v, 4) if isinstance(v, float) else v)
                            for k, v in robot.last_dock.items() if k in ("lateral", "along")}
            rec["lidar"]["yaw_deg"] = round(math.degrees(robot.last_dock["yaw"]), 2)
        if ok:
            time.sleep(1.0)
            gt = gt_map_pose(a.sim_log, start)
            est = robot.pose()
            pos, along, lat, dyaw = trial_errors(gt, dock)
            rec.update(pos_err=round(pos, 4), along=round(along, 4), lateral=round(lat, 4),
                       yaw_err_deg=round(dyaw, 2))
            if est is not None:
                rec["est_minus_gt"] = [round(est[0] - gt[0], 4), round(est[1] - gt[1], 4),
                                       round(math.degrees(wrap(est[2] - gt[2])), 2)]
            good = pos < a.max_pos_err and abs(dyaw) < a.max_yaw_err
            print(("ok   " if good else "FAIL ") + f"{spec}: true pos err {pos * 100:.1f} cm "
                  f"(along {along * 100:+.1f}, lat {lat * 100:+.1f}), yaw {dyaw:+.2f} deg", flush=True)
            fails += not good
            uok, umsg = robot.undock(approach)
            if not uok:
                print(f"FAIL {spec}: undock {umsg}", flush=True)
                fails += 1
        else:
            print(f"FAIL {spec}: {msg}", flush=True)
            fails += 1
        rec["secs"] = round(time.time() - t0, 1)
        print("DOCK_TRIAL " + json.dumps(rec), flush=True)
    print(f"dock trials: {len(slots) - fails}/{len(slots)} within "
          f"{a.max_pos_err * 100:.0f} cm / {a.max_yaw_err:.0f} deg", flush=True)
    ex.shutdown(timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if fails == 0 else 1)


if __name__ == "__main__":
    main()
