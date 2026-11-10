"""Day 27: run the FleetExecutor on kinematic robots (see tests/test_fleet_exec.py).
--perm i rotates the start poses among the robots. Exit 0 iff 6/6 tasks done,
min separation >= 0.45 m and never two robots docked at one shelf."""
import argparse
import itertools
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "tests"))
import test_fleet_exec as T  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--perm", type=int, default=0)
a = ap.parse_args()
names = sorted(T.STARTS)
perm = list(itertools.permutations(names))[a.perm % 6]
T.STARTS = {n: T.STARTS[p] for n, p in zip(names, perm)}
ex = T.make()
for b, s in T.TASKS:
    ex.submit(b, s)
worst = [0]
orig = ex.step


def step(p, dt):
    c = orig(p, dt)
    per = {}
    for n, (ph, tid, sub) in ex.phase.items():
        if ph in ("dock_in", "pick", "drop", "dock_out"):
            pick = ph == "pick" or (ph == "dock_in" and sub < 10) or (ph == "dock_out" and sub == 1)
            per.setdefault(ex._leg_shelf(tid, pick), set()).add(n)
    worst[0] = max([worst[0]] + [len(v) for v in per.values()])
    return c


ex.step = step
rep = T.run(ex, dict(T.STARTS))
ok = rep["tasks_done"] == 6 and rep["min_separation_m"] >= 0.45 and worst[0] <= 1
print(f"perm {perm}: done {rep['tasks_done']}/6 sep {rep['min_separation_m']} "
      f"lock_waits {rep['lock_waits']} sim {rep['sim_time_s']} s -> {'OK' if ok else 'BAD'}")
sys.exit(0 if ok else 1)
