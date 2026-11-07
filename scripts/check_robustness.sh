#!/usr/bin/env bash
# Day 27 check: the full demo works N times in a row within the resource budget.
#   scripts/check_robustness.sh [N] [--slam]      (default N=3, ~4 min per run)
# Runs scripts/check_launch_all.sh N times back to back (each run: launch_all
# with bag + media, health check, one MoveBox task, clean stop, bag + media
# checks, no leftovers). Meanwhile samples every 2 s:
#   - GPU memory used (nvidia-smi, whole GPU; baseline taken before run 1)
#   - system RAM used (free -m) and swap used
# PASS = N/N runs passed, peak VRAM < 7000 MiB (CLAUDE.md: 8 GB card) and
# peak RAM used < 14500 MiB (16 GB machine). Prints a ROBUST {json} summary.
# Logs: out/logs/check_robustness*.log (each run's own logs are overwritten
# by the next run; the per-run check output is kept as _run<i>.log).
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
L="$ROOT/out/logs"; mkdir -p "$L"
N=3; MODE=()
for a in "$@"; do
  case "$a" in
    --slam) MODE=(--slam) ;;
    [0-9]*) N="$a" ;;
    *) echo "usage: $0 [N] [--slam]"; exit 2 ;;
  esac
done
VRAM_MAX=7000; RAM_MAX=14500
SAMPLES="$L/check_robustness_samples.log"
: >"$SAMPLES"

sample() {
  local v r s
  v=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
  r=$(free -m | awk '/^Mem:/{print $3}'); s=$(free -m | awk '/^Swap:/{print $3}')
  echo "$(date +%s) ${v:-0} $r $s"
}
BASE=$(sample)
echo "     baseline (epoch vram_MiB ram_MiB swap_MiB): $BASE"
( while true; do sample; sleep 2; done ) >>"$SAMPLES" 2>/dev/null &
SPID=$!
trap 'kill $SPID 2>/dev/null' EXIT

PASSED=0; TIMES=()
for i in $(seq 1 "$N"); do
  T0=$(date +%s)
  if timeout 600 bash "$ROOT/scripts/check_launch_all.sh" "${MODE[@]}" >"$L/check_robustness_run$i.log" 2>&1; then
    PASSED=$((PASSED + 1)); R=PASS
  else R=FAIL; fi
  TIMES+=($(( $(date +%s) - T0 )))
  echo "     run $i/$N: $R in ${TIMES[-1]} s ($(grep -o 'place_error [0-9.]*' "$L/check_robustness_run$i.log" | head -1))"
  [ $R = FAIL ] && grep '^FAIL' "$L/check_robustness_run$i.log" | head -5
done
kill $SPID 2>/dev/null; trap - EXIT
LEFT="$(list_sim_leftovers)"

/usr/bin/python3 - "$SAMPLES" "$BASE" "$N" "$PASSED" "$VRAM_MAX" "$RAM_MAX" "${TIMES[*]}" "$LEFT" <<'PY'
import json, sys
path, base, n, passed, vmax, rmax, times, left = sys.argv[1:9]
rows = [list(map(int, l.split())) for l in open(path) if len(l.split()) == 4]
b = list(map(int, base.split()))
peak_v = max((r[1] for r in rows), default=0)
peak_r = max((r[2] for r in rows), default=0)
peak_s = max((r[3] for r in rows), default=0)
res = {"runs": int(n), "passed": int(passed), "run_s": [int(t) for t in times.split()],
       "vram_base_MiB": b[1], "vram_peak_MiB": peak_v, "ram_base_MiB": b[2], "ram_peak_MiB": peak_r,
       "swap_base_MiB": b[3], "swap_peak_MiB": peak_s, "samples": len(rows), "leftovers": bool(left.strip())}
ok = (res["passed"] == res["runs"] and peak_v < int(vmax) and peak_r < int(rmax)
      and rows and not res["leftovers"])
print("ROBUST " + json.dumps(res))
print(f"check_robustness: {'PASS' if ok else 'FAIL'} ({passed}/{n} runs, VRAM peak {peak_v} MiB "
      f"< {vmax}, RAM peak {peak_r} MiB < {rmax})")
sys.exit(0 if ok else 1)
PY
