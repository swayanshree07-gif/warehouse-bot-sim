#!/usr/bin/env bash
# Day 9 check: with the sim running, slam_toolbox (warehouse_bringup
# slam.launch.py) publishes /map and TF map->odom, and the map grows while the
# robot drives a scripted route down the central aisle and back.
# Exit 0 = PASS. Logs: out/logs/check_slam*.log. Map snapshots: out/maps/check_slam_*.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/env.sh"
source "$ROOT/scripts/lib.sh"
mkdir -p "$ROOT/out/logs" "$ROOT/out/maps"
LLOG="$ROOT/out/logs/check_slam.log"
export RUN_SIM_LOG="$ROOT/out/logs/check_slam_sim.log"
SLOG="$ROOT/out/logs/check_slam_toolbox.log"
[ -f "$ROOT/ros_ws/install/local_setup.bash" ] || { echo "ros_ws not built: run scripts/build_ws.sh"; exit 1; }

bash "$ROOT/scripts/run_sim.sh" --seconds 300 --start=-2,0,0 >"$LLOG" 2>&1 &
LPID=$!
SPID=""
cleanup() {
  [ -n "$SPID" ] && { kill -INT -- -"$SPID" 2>/dev/null; sleep 3; kill -9 -- -"$SPID" 2>/dev/null; }
  kill -TERM "$LPID" 2>/dev/null
  for _ in $(seq 1 60); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  pkill -9 -f "warehouse-bot-sim/sim/run_si[m].py" 2>/dev/null
  kill_kit_leftovers
}
trap cleanup EXIT

for _ in $(seq 1 300); do
  grep -q "LAUNCH_READY" "$LLOG" && break
  kill -0 "$LPID" 2>/dev/null || break
  sleep 1
done
grep -q "LAUNCH_READY" "$LLOG" || { echo "sim not ready"; cat "$LLOG"; exit 1; }
setsid timeout 280 ros2 launch warehouse_bringup slam.launch.py >"$SLOG" 2>&1 &
SPID=$!
sleep 8
probe() { timeout 60 python3 "$ROOT/scripts/map_probe.py" | grep "^MAP_PROBE" | cut -d' ' -f2-; }
drive() { timeout 180 python3 "$ROOT/scripts/waypoint_driver.py" "$@" | grep WAYPOINT_DONE; }

M0=$(probe); echo "map at start: $M0"
drive h:90 h:180 h:-90 h:0       # one full turn in place (odom frame)
sleep 3; M1=$(probe); echo "map after spin: $M1"
drive 3.6,0 h:180 0.5,0 h:0      # down the central aisle (world x -2 -> 1.6) and back
sleep 3; M2=$(probe); echo "map after route: $M2"
echo "gt: $(grep '^POSE' "$RUN_SIM_LOG" | tail -1)"
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$ROOT/out/maps/check_slam_map" \
  --ros-args -p save_map_timeout:=10.0 >"$ROOT/out/logs/check_slam_saver.log" 2>&1 && echo "saved out/maps/check_slam_map.{pgm,yaml}" || echo "map_saver failed (see out/logs/check_slam_saver.log)"

python3 - "$M0" "$M1" "$M2" <<'PY'
import json, sys
fails = []
def need(c, msg):
    print(("ok   " if c else "FAIL ") + msg)
    if not c: fails.append(msg)
m = [json.loads(a or "{}") for a in sys.argv[1:4]]
need(all(x.get("width") for x in m), "/map received at start, after spin, after route")
if all(x.get("width") for x in m):
    need(m[2]["frame"] == "map" and abs(m[2]["resolution"] - 0.05) < 1e-6, f"frame {m[2]['frame']}, resolution {m[2]['resolution']}")
    need(isinstance(m[2].get("tf_map_odom"), list), f"TF map->odom {m[2].get('tf_map_odom')}")
    known = [x["free"] + x["occupied"] for x in m]
    area = [k * 0.05 ** 2 for k in known]
    print(f"   known area m^2: start {area[0]:.1f}, after spin {area[1]:.1f}, after route {area[2]:.1f} (room 80)")
    need(m[2]["stamp"] > m[0]["stamp"], f"map updated (stamp {m[0]['stamp']:.1f} -> {m[2]['stamp']:.1f})")
    need(known[2] > known[0] * 1.1 and known[2] >= known[1], "map grows while driving")
    need(area[2] > 30, f"mapped area after route {area[2]:.1f} m^2 > 30")
    need(m[2]["occupied"] > 500, f"occupied cells {m[2]['occupied']} (walls + racks)")
print("check_slam: PASS" if not fails else "check_slam: FAIL")
sys.exit(1 if fails else 0)
PY
