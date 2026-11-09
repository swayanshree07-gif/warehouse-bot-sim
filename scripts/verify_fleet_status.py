#!/usr/bin/env python3
"""Verify a FLEET_STATUS JSON (stdin) for check_fleet_manager.sh (Day 23)."""
import json
import sys

s = json.load(sys.stdin)
t = s["tasks"]
robots = {v["robot"] for v in t.values()}
for k, v in sorted(t.items(), key=lambda kv: int(kv[0])):
    print(f"     task {k}: {v['box']} -> {v['dst']}[{v['slot']}] {v['state']} by {v['robot']}")
ok = (len(t) == 6 and all(v["state"] == "done" and v["robot"] for v in t.values())
      and len(robots) >= 2 and len(s["rejected"]) == 1 and s["collisions"] == 0
      and s["min_separation_m"] >= 0.5 and "tasks_per_hour" in s["metrics"])
print(f"     robots used {sorted(robots)}, collisions {s['collisions']}, "
      f"min sep {s['min_separation_m']} m, {s['metrics'].get('tasks_per_hour')} tasks/h (sim time)")
sys.exit(0 if ok else 1)
