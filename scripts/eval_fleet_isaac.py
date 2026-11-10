#!/usr/bin/env python3
"""Day 25: evaluate a check_fleet_isaac run.

  eval_fleet_isaac.py SIM_LOG REPORT_JSON CLIENT_RC PEAK_VRAM RAM_MB
PASS iff: client saw every task done (rc 0), report has 6 done / 0 failed,
every moved box's last ground-truth FBOXES position (world -> map) classifies
(Registry.slot_at) as the task's target slot and is not held, ground-truth min
pairwise robot separation over all FPOSE samples >= 0.45 m, metrics present,
peak VRAM < 7000 MB, RAM < 13000 MB.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, os.path.join(ROOT, "sim"), os.path.join(ROOT, "ros_ws/src/warehouse_tasks")]
from eval_fleet_traffic import min_separation, parse_fpose  # noqa: E402
from fleet_boxes import parse_fboxes  # noqa: E402
from fleet_layout import world_to_map  # noqa: E402
from warehouse_tasks.registry import Registry  # noqa: E402

N_TASKS, MIN_SEP = 6, 0.45
REG = os.path.join(ROOT, "ros_ws/src/warehouse_bringup/config/registry.yaml")


def evaluate(sim_text, report, client_rc, peak, ram, registry):
    lines = sim_text.splitlines()
    samples = [parse_fpose(ln) for ln in lines if ln.startswith("FPOSE")]
    fb = [ln for ln in lines if ln.startswith("FBOXES")]
    last = parse_fboxes(fb[-1]) if fb else {"boxes": {}}
    checks = []

    def chk(ok, msg):
        checks.append((bool(ok), msg))

    chk(client_rc == 0, f"client: all tasks done (rc={client_rc})")
    chk(report.get("tasks_done") == N_TASKS and report.get("tasks_failed") == 0,
        f"report: {report.get('tasks_done')} done, {report.get('tasks_failed')} failed")
    for t in report.get("tasks", []):
        if t["state"] != "done":
            continue
        b = last["boxes"].get(t["box"])
        if b is None:
            chk(False, f"{t['box']} missing from FBOXES")
            continue
        (x, y, z), holder = b
        mx, my = world_to_map(x, y)
        got = registry.slot_at((mx, my, z))
        chk(list(got) == t["dst"] and holder == "-",
            f"{t['box']} by {t['robot']}: at {got[0]}[{got[1]}] want {t['dst'][0]}[{t['dst'][1]}] holder={holder}")
    sep = min_separation(samples)
    chk(len(samples) > 10 and sep >= MIN_SEP, f"ground-truth min separation {sep:.3f} m ({len(samples)} samples)")
    m = report.get("metrics", {})
    chk(m.get("tasks_per_hour", 0) > 0 and len(m.get("per_robot", {})) == 3,
        f"metrics: {m.get('tasks_per_hour')} tasks/h, idle {m.get('mean_idle_pct')} %, "
        f"traffic wait {m.get('total_traffic_wait_s')} s")
    chk(peak < 7000 and ram < 13000, f"peak VRAM {peak} MB, RAM {ram} MB")
    return checks, sep


def main():
    sim_log, rep, rc, peak, ram = sys.argv[1:6]
    report = json.load(open(rep)) if os.path.exists(rep) else {}
    checks, _ = evaluate(open(sim_log).read(), report, int(rc), int(peak), int(ram), Registry.load(REG))
    for ok, msg in checks:
        print(("ok   " if ok else "FAIL ") + msg)
    ok = all(c for c, _ in checks)
    print("check_fleet_isaac: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
