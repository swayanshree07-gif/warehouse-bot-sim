#!/usr/bin/env python3
"""Day 24: evaluate a check_fleet_traffic run from its logs.

  eval_fleet_traffic.py SIM_LOG FOLLOW_LOG "robot1=x,y ..." PEAK_VRAM RAM_MB
Ground-truth min pairwise separation over every FPOSE line, final goal error
(map frame), FOLLOW_DONE with deadlocks >= 1. Prints PASS/FAIL, exit 0/1.
"""
import itertools
import math
import re
import sys

MIN_SEP, GOAL_TOL = 0.45, 0.25


def parse_fpose(line):
    out = {}
    for kv in line.split()[2:]:
        ns, v = kv.split("=")
        out[ns] = tuple(float(a) for a in v.split(","))
    return out


def min_separation(samples):
    best = math.inf
    for s in samples:
        for a, b in itertools.combinations(sorted(s), 2):
            best = min(best, math.hypot(s[a][0] - s[b][0], s[a][1] - s[b][1]))
    return best


def evaluate(sim_text, follow_text, goals, peak, ram, to_map):
    samples = [parse_fpose(ln) for ln in sim_text.splitlines() if ln.startswith("FPOSE")]
    m = re.search(r"FOLLOW_DONE .*deadlocks=(\d+)", follow_text)
    res = {"samples": len(samples), "done": bool(m),
           "deadlocks": int(m.group(1)) if m else -1,
           "min_sep": min_separation(samples), "goal_err": {}}
    if samples:
        for ns, (gx, gy) in goals.items():
            mx, my = to_map(*samples[-1][ns][:2])
            res["goal_err"][ns] = math.hypot(mx - gx, my - gy)
    res["ok"] = (res["done"] and res["deadlocks"] >= 1 and len(samples) > 10
                 and res["min_sep"] >= MIN_SEP and peak < 7000 and ram < 13000
                 and len(res["goal_err"]) == len(goals)
                 and all(e < GOAL_TOL for e in res["goal_err"].values()))
    return res


def main():
    from fleet_layout import world_to_map
    sim_log, flog, goals_s, peak, ram = sys.argv[1:6]
    goals = {k: tuple(map(float, v.split(","))) for k, v in (kv.split("=") for kv in goals_s.split())}
    r = evaluate(open(sim_log).read(), open(flog).read(), goals, int(peak), int(ram), world_to_map)
    print(f"FPOSE samples: {r['samples']}, follow done: {r['done']}, deadlocks resolved: {r['deadlocks']}")
    print(f"ground-truth min separation: {r['min_sep']:.3f} m (need >= {MIN_SEP})")
    for ns, e in sorted(r["goal_err"].items()):
        print(f"{ns}: goal err {e:.3f} m {'ok' if e < GOAL_TOL else 'BAD'}")
    print("check_fleet_traffic: PASS" if r["ok"] else "check_fleet_traffic: FAIL")
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
