# Source this before anything in this project:  source ./env.sh
# Modelled on ~/isaac_ws/env.sh. Isolation: drop everything other projects
# (pac_ws, hexapod_ws, tiburon_ws) put into the shell via ~/.bashrc.

WAREHOUSE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WAREHOUSE_ROOT

unset ROS_NAMESPACE PAC_WS AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
      PYTHONPATH LD_LIBRARY_PATH ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS \
      ROS_DOMAIN_ID ROS_LOCALHOST_ONLY GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH \
      GZ_CONFIG_PATH VIRTUAL_ENV
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -E "pac_ws|hexapod|tiburon|/opt/ros/|isaacsim-env" \
       | awk 'NF && !seen[$0]++' | paste -sd:)
export PATH

export OMNI_KIT_ACCEPT_EULA=YES
# Isaac Sim 6.2 shared install: use it, never pip install into it.
source "$HOME/omniverse/isaacsim-env/bin/activate"
# Hybrid Intel+NVIDIA laptop: force rendering on the RTX GPU
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only

# System ROS 2 Jazzy (read-only) + Isaac Sim bridge workspace (read-only)
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -f "$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash" ] && \
    source "$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash"
# This project's own ROS 2 workspace, if built
[ -f "$WAREHOUSE_ROOT/ros_ws/install/local_setup.bash" ] && \
    source "$WAREHOUSE_ROOT/ros_ws/install/local_setup.bash"

# Own ROS network: pac_ws = 10, isaac_ws = 20, this project = 21. Localhost only.
export ROS_DOMAIN_ID=21
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
