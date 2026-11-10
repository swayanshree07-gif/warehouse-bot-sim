#!/usr/bin/env bash
# Day 28: lint check. flake8 (config in .flake8) on all project Python,
# bash -n on all shell scripts. PASS if both are clean.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 1
rc=0
/usr/bin/python3 -m flake8 sim scripts tests ros_ws/src || rc=1
for f in scripts/*.sh env.sh; do bash -n "$f" || { echo "bash -n failed: $f"; rc=1; }; done
[ $rc -eq 0 ] && echo "LINT PASS" || echo "LINT FAIL"
exit $rc
