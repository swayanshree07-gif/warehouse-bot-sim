"""MoveBox task manager (Day 19).

  ros2 launch warehouse_tasks task_manager.launch.py [backend:=mock|ros]
      [use_sim_time:=true] [mock_fail:=navigate:unreachable] [lidar_dock:=true]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("backend", default_value="mock"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("mock_step_time", default_value="0.2"),
        DeclareLaunchArgument("mock_fail", default_value=""),
        DeclareLaunchArgument("lidar_dock", default_value="true"),
    ]
    node = Node(
        package="warehouse_tasks", executable="task_manager", name="task_manager", output="screen",
        parameters=[{
            "backend": LaunchConfiguration("backend"),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            "mock_step_time": ParameterValue(LaunchConfiguration("mock_step_time"), value_type=float),
            "mock_fail": ParameterValue(LaunchConfiguration("mock_fail"), value_type=str),
            "lidar_dock": ParameterValue(LaunchConfiguration("lidar_dock"), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])
