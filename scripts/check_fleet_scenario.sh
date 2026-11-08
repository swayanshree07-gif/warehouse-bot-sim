#!/usr/bin/env bash
# Day 18: headless light-sim fleet scenario + metrics check.
# Exit 0 iff 9/9 tasks done, 0 collisions, min sep >= 0.5 m, tasks/hour >= floor,
# and the JSON report has every metric key.
set -euo pipefail
cd "$(dirname "$0")/.."
FLOOR="${MIN_TPH:-200}"   # measured 337.5/h on Day 18; floor leaves ~40 % margin
OUT="${OUT:-out/fleet/scenario_report.json}"
timeout 120 /usr/bin/python3 scripts/run_lightsim.py --out "$OUT" --min-tph "$FLOOR"
/usr/bin/python3 - "$OUT" <<'PY'
import json, sys
sys.path.insert(0, "ros_ws/src/warehouse_tasks")
from warehouse_tasks.metrics import METRIC_KEYS
m = json.load(open(sys.argv[1]))["metrics"]
missing = [k for k in METRIC_KEYS if k not in m]
assert not missing, f"missing metrics: {missing}"
for r, v in m["per_robot"].items():
    assert {"idle_pct", "traffic_wait_s", "tasks_done"} <= v.keys(), r
print(f"metrics OK: {m['tasks_per_hour']} tasks/h, idle {m['mean_idle_pct']} %, "
      f"task {m['mean_task_time_s']} s, queue wait {m['mean_queue_wait_s']} s")
PY
echo "check_fleet_scenario: PASS"
