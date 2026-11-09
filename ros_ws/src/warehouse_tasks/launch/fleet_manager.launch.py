"""Fleet manager (Day 23).

  ros2 launch warehouse_tasks fleet_manager.launch.py [backend:=light] [tick_period:=0.05]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument("backend", default_value="light"),
            DeclareLaunchArgument("tick_period", default_value="0.05"),
            DeclareLaunchArgument("starts", default_value="robot1=-2,0;robot2=2,0;robot3=6,0")]
    node = Node(package="warehouse_tasks", executable="fleet_manager", name="fleet_manager",
                output="screen", parameters=[{
                    "backend": LaunchConfiguration("backend"),
                    "tick_period": ParameterValue(LaunchConfiguration("tick_period"), value_type=float),
                    "starts": ParameterValue(LaunchConfiguration("starts"), value_type=str)}])
    return LaunchDescription(args + [node])
