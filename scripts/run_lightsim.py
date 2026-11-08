#!/usr/bin/env python3
"""Run the light 2D fleet scenario headless (Day 17). Writes a JSON report.

Usage: timeout 120 python3 scripts/run_lightsim.py [--out out/fleet/lightsim.json]
Exit 0 only if every task is done, no collisions and min separation >= 0.5 m.
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "warehouse_tasks"))
from warehouse_tasks.lightsim import DEFAULT_SCENARIO, LightSim  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402
from warehouse_tasks.traffic import Roadmap  # noqa: E402

BR = os.path.join(ROOT, "ros_ws", "src", "warehouse_bringup")
STARTS = {"robot1": (-2.0, 0.0), "robot2": (2.0, 0.0), "robot3": (6.0, 0.0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "fleet", "lightsim.json"))
    ap.add_argument("--min-sep", type=float, default=0.5)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    t0 = time.time()
    rm = Roadmap.from_map(os.path.join(BR, "maps", "warehouse.yaml"))
    sim = LightSim(rm, Registry.load(os.path.join(BR, "config", "registry.yaml")), STARTS)
    for box, shelf in DEFAULT_SCENARIO:
        sim.submit(box, shelf)
    rep = sim.run()
    rep["wall_time_s"] = round(time.time() - t0, 2)
    rep["n_scenario_tasks"] = len(DEFAULT_SCENARIO)
    if args.verbose:
        for t, m in sim.events:
            print(f"{t:7.1f}  {m}")
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(rep, f, indent=2)
    print(json.dumps({k: v for k, v in rep.items() if k != "boxes"}))
    ok = (rep["tasks_done"] == len(DEFAULT_SCENARIO) and rep["tasks_failed"] == 0
          and rep["rejected"] == 0 and rep["collisions"] == 0
          and rep["min_separation_m"] >= args.min_sep)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
