#!/usr/bin/env python3
"""Fleet manager, Isaac backend (Day 25).

  python3 scripts/fleet_isaac.py [--out out/fleet_isaac_report.json] [--timeout 700]
Same interface as the light fleet_manager node (Day 23):
  /fleet/tasks  (sub, String "box_id shelf_id")   /fleet/status (pub, String JSON, 2 Hz)
Poses: TF map -> robotN/base_footprint (AMCL). Commands: /robotN/cmd_vel at 10 Hz.
Docking: approach pose, face the rack, lidar rack-face measurement (rack_align)
gives the dock pose (AMCL alone is ~0.1 m off; attach needs < 0.1 m).
Pick/drop run scripts/gripper_cmd.py in a subprocess per robot (non-blocking):
  pick = attach (must return the task's box) + lift LIFT_CLEARANCE
  drop = lift 0.0 + detach
The report JSON (FleetExecutor.report() + wall time) is rewritten whenever a task
finishes. Exits after --timeout wall seconds or on SIGINT; robots are stopped.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from warehouse_interfaces.msg import BoxStates
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros_ws/src/warehouse_tasks"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
from warehouse_tasks.fleet_exec import FleetExecutor  # noqa: E402
from warehouse_tasks.fleet_manager import parse_task, status_dict  # noqa: E402
from warehouse_tasks import rack_align  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402
from warehouse_tasks.traffic import Roadmap  # noqa: E402
import robot_params  # noqa: E402

BR = os.path.join(ROOT, "ros_ws/src/warehouse_bringup")
NAMES = ("robot1", "robot2", "robot3")
G = f"{sys.executable} {ROOT}/scripts/gripper_cmd.py"


def action_cmd(ns, kind):
    if kind == "pick":
        return f"{G} attach --ns {ns} && {G} lift {robot_params.LIFT_CLEARANCE} --ns {ns}"
    return f"{G} lift 0.0 --ns {ns} && {G} detach --ns {ns}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "out/fleet_isaac_report.json"))
    ap.add_argument("--timeout", type=float, default=700.0)
    a = ap.parse_args()
    stop = {"s": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(s=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(s=True))
    rclpy.init()
    node = rclpy.create_node("fleet_manager")
    buf = Buffer()
    TransformListener(buf, node)
    pubs = {n: node.create_publisher(Twist, f"/{n}/cmd_vel", 10) for n in NAMES}
    spub = node.create_publisher(String, "/fleet/status", 10)
    inbox = []
    node.create_subscription(String, "/fleet/tasks", lambda m: inbox.append(m.data), 50)

    scans = {}
    for n in NAMES:
        node.create_subscription(LaserScan, f"/{n}/scan", lambda m, n=n: scans.__setitem__(n, m), 5)

    def measure(n, shelf, slot):
        """Lidar dock pose in robot n's base frame (rack_align, Day 20) or None."""
        scan = scans.get(n)
        if scan is None:
            return None
        try:
            t = buf.lookup_transform(f"{n}/base_link", f"{n}/laser", Time()).transform.translation
            laser = (t.x, t.y, 0.0)
        except Exception:
            laser = (0.2, 0.0, 0.0)
        c = slot["center"]
        approach_off = math.hypot(slot["approach"][0] - c[0], slot["approach"][1] - c[1])
        dock_off = math.hypot(slot["dock"][0] - c[0], slot["dock"][1] - c[1])
        lat, depth, flen = rack_align.slot_geometry(shelf, slot, slot["dock"][2])
        meas, why = rack_align.measure_face(
            list(scan.ranges), scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
            laser=laser, expect_dist=approach_off - depth / 2, face_len=flen)
        if meas is None:
            print(f"RACK_NOT_FOUND {n}: {why}", flush=True)
            return None
        rel, _ = rack_align.dock_in_base(meas, lat, depth, dock_off)
        return rel

    boxes = {}
    node.create_subscription(BoxStates, "/boxes", lambda m: boxes.update(
        {b.id: (b.position.x, b.position.y, b.held) for b in m.boxes}), 5)

    def place_error(n, box, slot):
        """Held box offset from the slot centre (map frame) from /boxes (sim)."""
        if box not in boxes:
            return None
        mx, my = boxes[box][:2]          # /boxes is already in the map frame
        return mx - slot["center"][0], my - slot["center"][1]

    def poses():
        out = {}
        for n in NAMES:
            try:
                t = buf.lookup_transform("map", f"{n}/base_footprint", Time())
            except Exception:
                return None
            q = t.transform.rotation
            out[n] = (t.transform.translation.x, t.transform.translation.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
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
        print("FLEET_NO_TF", flush=True)
        return 1
    ex = FleetExecutor(Roadmap.from_map(os.path.join(BR, "maps/warehouse.yaml")),
                       Registry.load(os.path.join(BR, "config/registry.yaml")), p,
                       measure_fn=measure, place_error_fn=place_error)
    print("fleet_manager ready: backend=isaac robots=" + ",".join(NAMES), flush=True)
    procs = {}           # ns -> (Popen, kind, box)
    n_ev, last_pub, last_done, prev = 0, 0.0, -1, time.time()

    def write_report():
        rep = ex.report()
        rep["wall_s"] = round(time.time() - t0, 1)
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(rep, f, indent=1)

    while not stop["s"] and time.time() - t0 < a.timeout:
        end = time.time() + 0.1
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        while inbox:
            txt = inbox.pop(0)
            try:
                ex.submit(*parse_task(txt))
            except ValueError as e:
                ex.rejected.append((txt, "", str(e)))
        p = poses() or p
        now = time.time()
        send(ex.step(p, now - prev) if ex.busy() else {n: (0.0, 0.0) for n in NAMES})
        prev = now
        while ex.requests:
            ns, kind, box = ex.requests.pop(0)
            procs[ns] = (subprocess.Popen(["bash", "-c", action_cmd(ns, kind)], stdout=subprocess.PIPE,
                                          stderr=subprocess.STDOUT, text=True), kind, box)
        for ns, (pr, kind, box) in list(procs.items()):
            if pr.poll() is None:
                continue
            out = pr.stdout.read()
            ok = pr.returncode == 0 and (kind == "drop" or f"message={box}" in out)
            if not ok:
                print(f"ACTION_FAIL {ns} {kind} {box}: {out.strip()[-300:]}", flush=True)
            del procs[ns]
            ex.action_done(ns, ok)
        for t, m in ex.events[n_ev:]:
            print(f"[{t:6.1f}] {m}", flush=True)
        n_ev = len(ex.events)
        st = ex.fleet.status()
        if st["done"] + st["failed"] != last_done:
            last_done = st["done"] + st["failed"]
            write_report()
        if now - last_pub > 0.5:
            last_pub = now
            spub.publish(String(data=json.dumps(status_dict(ex))))
            if int(now - t0) % 10 == 0:
                print("FLEET t=%.0f " % (now - t0) + " ".join(
                    f"{n}={ex.phase[n][0]}@{p[n][0]:.2f},{p[n][1]:.2f}" for n in NAMES), flush=True)
    send({n: (0.0, 0.0) for n in NAMES})
    for pr, _, _ in procs.values():
        pr.kill()
    write_report()
    print("FLEET_EXIT " + json.dumps({k: v for k, v in ex.report().items() if k != "tasks"}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
