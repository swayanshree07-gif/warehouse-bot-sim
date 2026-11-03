"""Online async SLAM (Day 9): robot description + slam_toolbox, sim time.

  ros2 launch warehouse_bringup slam.launch.py

Needs the sim running (scripts/run_sim.sh): /clock, /scan, TF odom->base_link.
Publishes /map and TF map->odom. slam_toolbox is a lifecycle node in Jazzy, so
we include its stock online_async_launch.py (which configures + activates it)
with our parameter file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("warehouse_bringup")
    slam_share = get_package_share_directory("slam_toolbox")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "description.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={"use_sim_time": use_sim_time,
                              "slam_params_file": LaunchConfiguration("slam_params"),
                              "autostart": "true"}.items(),
        ),
    ])
