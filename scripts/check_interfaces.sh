#!/usr/bin/env bash
# Day 18 check: warehouse_interfaces builds, `ros2 interface show` prints the
# definitions and the generated Python types import and round-trip.
# No sim needed. Exit 0 = PASS. Log: out/logs/check_interfaces.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/out/logs"
LOG="$ROOT/out/logs/check_interfaces.log"
timeout 300 "$ROOT/scripts/build_ws.sh" --packages-select warehouse_interfaces >"$LOG" 2>&1 \
  || { echo "FAIL colcon build"; tail -20 "$LOG"; echo "check_interfaces: FAIL"; exit 1; }
echo "ok   colcon build warehouse_interfaces"
source "$ROOT/env.sh"
FAILS=0
expect() {   # expect <interface> <text that must appear>
  local out; out=$(timeout 30 ros2 interface show "$1" 2>&1)
  if echo "$out" | grep -q -- "$2"; then echo "ok   ros2 interface show $1 ($2)";
  else echo "FAIL ros2 interface show $1 lacks '$2'"; echo "$out" | head; FAILS=$((FAILS + 1)); fi
}
expect warehouse_interfaces/action/MoveBox "string target_shelf"
expect warehouse_interfaces/action/MoveBox "float32 place_error"
expect warehouse_interfaces/msg/BoxState "geometry_msgs/Point position"
expect warehouse_interfaces/msg/BoxStates "BoxState\[\] boxes"
timeout 30 /usr/bin/python3 - <<'PY' || FAILS=$((FAILS + 1))
from rclpy.serialization import serialize_message, deserialize_message
from warehouse_interfaces.action import MoveBox
from warehouse_interfaces.msg import BoxState, BoxStates
g = MoveBox.Goal(box_id="box_03", target_shelf="shelf_D")
assert g.target_slot == -1, "default target_slot must be -1"
m = BoxStates(boxes=[BoxState(id="box_03", shelf="shelf_B", slot=0)])
m2 = deserialize_message(serialize_message(m), BoxStates)
assert m2.boxes[0].id == "box_03" and m2.boxes[0].slot == 0
print("ok   python types import, MoveBox default slot -1, BoxStates round-trip")
PY
[ $FAILS -eq 0 ] && { echo "check_interfaces: PASS"; exit 0; }
echo "check_interfaces: FAIL"; exit 1
