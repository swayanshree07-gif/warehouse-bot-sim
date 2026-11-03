"""Nav2 on the saved warehouse map with AMCL (Day 12).

  ros2 launch warehouse_bringup nav2.launch.py [map:=/path/map.yaml]
  ros2 launch warehouse_bringup nav2.launch.py slam:=true    (Day 14)

Needs the sim running (scripts/run_sim.sh --start=-2,0,0): /clock, /scan,
/odom, TF odom->base_link. Starts robot_state_publisher (description.launch.py),
map_server + amcl (lifecycle_manager_localization) and planner, controller,
behaviors, bt_navigator, waypoint_follower (lifecycle_manager_navigation).
The controller publishes geometry_msgs/Twist on /cmd_vel directly (no
velocity smoother / collision monitor: the robot is slow and the sim is the
only consumer). AMCL starts at (0, 0, 0) in the map frame = the spawn pose.

slam:=true replaces map_server + AMCL with slam_toolbox (slam.launch.py, which
also brings the description) and adds config/nav2_slam.yaml on top of the
params (planner may plan through unknown space while the map grows).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOCALIZATION = ["map_server", "amcl"]
NAVIGATION = ["controller_server", "planner_server", "behavior_server",
              "bt_navigator", "waypoint_follower"]


def generate_launch_description():
    share = get_package_share_directory("warehouse_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params")
    log_level = LaunchConfiguration("log_level")
    slam = LaunchConfiguration("slam")
    slam_overlay = os.path.join(share, "config", "nav2_slam.yaml")
    # nav2_slam.yaml is only applied when slam:=true (a param file given as a
    # substitution that resolves to "" would fail, so pick per condition).
    common = [params, {"use_sim_time": use_sim_time}]
    args = ["--ros-args", "--log-level", log_level]

    def node(pkg, exe, name, extra=None, condition=None):
        return Node(package=pkg, executable=exe, name=name, output="screen",
                    parameters=common + (extra or []), arguments=args,
                    condition=condition)

    def nav_nodes(extra, condition):
        return GroupAction(condition=condition, actions=[
            node("nav2_controller", "controller_server", "controller_server", extra),
            node("nav2_planner", "planner_server", "planner_server", extra),
            node("nav2_behaviors", "behavior_server", "behavior_server", extra),
            node("nav2_bt_navigator", "bt_navigator", "bt_navigator", extra),
            node("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower", extra),
        ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "warehouse.yaml")),
        DeclareLaunchArgument("params", default_value=os.path.join(share, "config", "nav2.yaml")),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("slam", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "description.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
            condition=UnlessCondition(slam),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "slam.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
            condition=IfCondition(slam),
        ),
        node("nav2_map_server", "map_server", "map_server",
             extra=[{"yaml_filename": LaunchConfiguration("map")}], condition=UnlessCondition(slam)),
        node("nav2_amcl", "amcl", "amcl", condition=UnlessCondition(slam)),
        nav_nodes([], UnlessCondition(slam)),
        nav_nodes([slam_overlay], IfCondition(slam)),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                          "node_names": LOCALIZATION}], condition=UnlessCondition(slam)),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                          "node_names": NAVIGATION}]),
    ])
