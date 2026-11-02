#!/usr/bin/env bash
# Verify the project environment is isolated and complete. Exit 0 = OK.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/env.sh"
fail=0
ok()  { echo "  OK   $*"; }
bad() { echo "  FAIL $*"; fail=1; }

[ "${ROS_DOMAIN_ID:-}" = "21" ] && ok "ROS_DOMAIN_ID=21" || bad "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
[ "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" = "LOCALHOST" ] && ok "discovery LOCALHOST" || bad "discovery range"
for v in ROS_NAMESPACE PAC_WS; do
  [ -z "${!v:-}" ] && ok "$v unset" || bad "$v still set (${!v})"
done
for v in PATH AMENT_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH GZ_SIM_RESOURCE_PATH; do
  if echo "${!v:-}" | grep -qE "pac_ws|hexapod|tiburon"; then bad "$v contains other-project paths"; else ok "$v clean"; fi
done
[ "${ROS_DISTRO:-}" = "jazzy" ] && ok "ROS_DISTRO=jazzy" || bad "ROS_DISTRO"
python -c "import isaacsim" 2>/dev/null && ok "isaacsim importable ($(command -v python))" || bad "isaacsim import"
[ -d "$HOME/omniverse/isaacsim-env/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.bridge" ] \
  && ok "isaacsim.ros2.bridge extension present" || bad "ros2 bridge extension missing"
for p in slam_toolbox nav2_bringup nav2_simple_commander rosbag2; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && ok "ros pkg $p" || bad "ros pkg $p missing"
done
command -v timeout >/dev/null && ok "timeout available" || bad "timeout missing"
du_mb=$(du -sm "$ROOT" | cut -f1); [ "$du_mb" -lt 4000 ] && ok "project size ${du_mb} MB < 4000" || bad "project size ${du_mb} MB"
[ $fail -eq 0 ] && echo "check_env: PASS" || echo "check_env: FAIL"
exit $fail
