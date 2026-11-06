"""Read one /boxes (warehouse_interfaces/BoxStates) message and compare it with
the expected placement (Day 23).

  python3 scripts/boxes_probe.py [--expect box_09=shelf_D:1,box_03=-] [--rate]
Expected = the registry's initial placement, overridden by --expect
(`-` = not on any slot). Prints `BOXES_GT {json}` and ok/FAIL lines; exit 0
if every box matches, the frame is "map", the stamp is set, nothing is held
and (with --rate) /boxes arrives at >= 5 Hz. Needs `source env.sh`.
"""
import argparse
import json
import os
import sys
import time

import rclpy
import yaml
from warehouse_interfaces.msg import BoxStates

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
REG = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup", "config", "registry.yaml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--expect", default="")
    ap.add_argument("--rate", action="store_true")
    ap.add_argument("--timeout", type=float, default=15.0)
    a = ap.parse_args()
    reg = yaml.safe_load(open(REG))
    want = {b: f"{v['shelf']}:{v['slot']}" for b, v in reg["boxes"].items()}
    for kv in filter(None, a.expect.split(",")):
        k, v = kv.split("=")
        want[k] = v
    rclpy.init()
    node = rclpy.create_node("boxes_probe")
    msgs = []
    node.create_subscription(BoxStates, "boxes", msgs.append, 10)
    t0 = time.time()
    need = 11 if a.rate else 1
    while len(msgs) < need and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    ok = True

    def chk(c, m):
        nonlocal ok
        print(("ok   " if c else "FAIL ") + m, flush=True)
        ok &= bool(c)
    try:
        if not msgs:
            chk(False, "no /boxes message")
            return 1
        m = msgs[-1]
        got = {b.id: (f"{b.shelf}:{b.slot}" if b.shelf else "-") for b in m.boxes}
        print("BOXES_GT " + json.dumps({b.id: {"at": got[b.id], "held": b.held,
                                               "pos": [round(b.position.x, 3), round(b.position.y, 3),
                                                       round(b.position.z, 3)]} for b in m.boxes}), flush=True)
        chk(m.header.frame_id == "map", f"frame_id {m.header.frame_id!r}")
        chk(m.header.stamp.sec > 0 or m.header.stamp.nanosec > 0, "header stamp set (sim time)")
        chk(not any(b.held for b in m.boxes), "no box held")
        bad = {b: (got.get(b), w) for b, w in want.items() if got.get(b) != w}
        chk(not bad, f"{len(want)} boxes at expected slots" + (f", mismatches {bad}" if bad else ""))
        if a.rate:
            # rate from the header stamps (sim time)
            s = [x.header.stamp.sec + x.header.stamp.nanosec * 1e-9 for x in msgs]
            hz = (len(s) - 1) / (s[-1] - s[0]) if s[-1] > s[0] else 0.0
            chk(hz >= 5.0, f"/boxes rate {hz:.1f} Hz (sim time)")
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
