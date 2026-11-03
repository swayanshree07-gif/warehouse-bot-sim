"""robot_state_publisher with the generated URDF (Day 8).

  ros2 launch warehouse_bringup description.launch.py [use_sim_time:=true]

Publishes /robot_description, static TFs (base_link->laser, base_footprint,
casters) and, once the sim publishes /joint_states, the wheel and lift TFs.
use_sim_time defaults to true (the sim publishes /clock).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory("warehouse_bringup"), "urdf", "warehouse_bot.urdf")
    with open(urdf) as f:
        description = f.read()
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(description, value_type=str),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
