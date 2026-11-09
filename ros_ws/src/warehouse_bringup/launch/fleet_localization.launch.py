"""Localization for the 3-robot fleet on the saved map (Day 20).

  ros2 launch warehouse_bringup fleet_localization.launch.py

One shared map_server (/map, frame "map") plus, per namespace robotN:
robot_state_publisher (frame_prefix "robotN/", reads /robotN/joint_states) and
AMCL (scan /robotN/scan, map /map, frames robotN/odom + robotN/base_footprint,
publishes /robotN/amcl_pose and TF map->robotN/odom on the shared /tf).
Needs sim/run_fleet_sim.py running. Initial poses = sim/fleet_layout.map_starts()
(kept in sync by tests/test_fleet_layout.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

# map frame poses (x, y, yaw); must equal sim/fleet_layout.map_starts()
MAP_STARTS = {
    "robot1": (-1.0, 0.0, 0.0),
    "robot2": (2.0, 0.0, 0.0),
    "robot3": (5.0, 0.0, 0.0),
}


def generate_launch_description():
    share = get_package_share_directory("warehouse_bringup")
    with open(os.path.join(share, "urdf", "warehouse_bot.urdf")) as f:
        description = f.read()
    sim_time = LaunchConfiguration("use_sim_time")
    params = os.path.join(share, "config", "nav2.yaml")
    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "warehouse.yaml")),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[{"use_sim_time": sim_time, "yaml_filename": LaunchConfiguration("map")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_map", output="screen",
             parameters=[{"use_sim_time": sim_time, "autostart": True, "node_names": ["map_server"]}]),
    ]
    for ns, (x, y, yaw) in MAP_STARTS.items():
        actions.append(GroupAction([
            PushRosNamespace(ns),
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 name="robot_state_publisher", output="screen",
                 parameters=[{"robot_description": ParameterValue(description, value_type=str),
                              "frame_prefix": f"{ns}/",
                              "use_sim_time": ParameterValue(sim_time, value_type=bool)}]),
            Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                 parameters=[params, {
                     "use_sim_time": sim_time,
                     "base_frame_id": f"{ns}/base_footprint",
                     "odom_frame_id": f"{ns}/odom",
                     "global_frame_id": "map",
                     "scan_topic": "scan",
                     "map_topic": "/map",
                     "set_initial_pose": True,
                     "initial_pose": {"x": x, "y": y, "z": 0.0, "yaw": yaw},
                     # fleet: 3 filters on a laptop, keep them light
                     "min_particles": 200, "max_particles": 800,
                 }]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_localization", output="screen",
                 parameters=[{"use_sim_time": sim_time, "autostart": True, "node_names": ["amcl"]}]),
        ]))
    return LaunchDescription(actions)
