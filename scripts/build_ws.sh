#!/usr/bin/env bash
# Build this project's colcon workspace (ros_ws/) in a clean shell: only
# /opt/ros/jazzy, system python3 (NOT the Isaac Sim venv, so ament_python
# packages get /usr/bin/python3 shebangs and nothing touches the shared venv).
# Regenerates the URDF from sim/robot_params.py first.
#   scripts/build_ws.sh [extra colcon args]
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
set -e
/usr/bin/python3 "$ROOT/sim/urdf_gen.py"
cd "$ROOT/ros_ws"
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG="${LANG:-C.UTF-8}" \
  ROS_DOMAIN_ID=21 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install "$@"' _ "$@"
