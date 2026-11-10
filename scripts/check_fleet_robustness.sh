#!/usr/bin/env bash
# Day 27: fleet robustness without the GPU (CLAUDE.md fallback to the light sim).
# 3 runs in a row of (a) the light-sim scenario (check_fleet_scenario.sh) and
# (b) the FleetExecutor (the Isaac backend's brain) on kinematic robots with a
# different start permutation each run, 6 chained tasks, shelf lock checked.
# PASS = 3/3 for both. No GPU, no ROS; each run < 10 s.
set -uo pipefail
cd "$(dirname "$0")/.."
ok=0
for i in 1 2 3; do
  if OUT="out/fleet/robust_scenario_$i.json" bash scripts/check_fleet_scenario.sh >/dev/null 2>&1 \
     && timeout 120 /usr/bin/python3 scripts/fleet_exec_kinematic.py --perm "$i"; then
    ok=$((ok + 1)); echo "run $i: PASS"
  else
    echo "run $i: FAIL"
  fi
done
echo "check_fleet_robustness: $ok/3"
[ "$ok" -eq 3 ] && echo "check_fleet_robustness: PASS" || { echo "check_fleet_robustness: FAIL"; exit 1; }
