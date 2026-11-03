#!/usr/bin/env bash
# Day 11: map quality check without the sim. Compares a saved map
# (default out/maps/warehouse.yaml, made by scripts/map_warehouse.sh) with the
# scene geometry: free-area coverage >= 80 % and occupied-cell IoU >= 0.7
# (2-cell tolerance; see sim/map_eval.py). Exit 0 = PASS.
#   scripts/check_map.sh [map.yaml] [--start=x,y,yaw_deg]
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MAP="${1:-$ROOT/out/maps/warehouse.yaml}"
shift || true
[ -f "$MAP" ] || { echo "no map $MAP (run scripts/map_warehouse.sh)"; echo "check_map: FAIL"; exit 1; }
python3 "$ROOT/sim/map_eval.py" "$MAP" --start=-2,0,0 --min-coverage 0.8 --min-iou 0.7 "$@" \
  && echo "check_map: PASS" || { echo "check_map: FAIL"; exit 1; }
