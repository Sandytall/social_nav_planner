"""Bring up Nav2 with the SocialNav controller.

Mapless: a static map->odom identity transform stands in for localization, and the
global costmap uses a rolling window, so no map file / AMCL is required for the demo.

Each nav node receives the params as a SINGLE concrete file path. Passing [file, dict]
instead makes launch_ros merge/rewrite the YAML and silently drop nested plugin params
(e.g. FollowPath.plugin), which made controller_server fall back to its DWB default.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("social_nav_bringup")
    # Env-var override lets the benchmark run ablations (baseline controller / social layer
    # off) without touching launch args. Falls back to the shipped params.
    default_params = os.environ.get(
        "SOCIAL_NAV_PARAMS", os.path.join(bringup, "config", "nav2_params.yaml"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    # Pass the concrete resolved YAML path (not a LaunchConfiguration): as a single file it
    # avoids the launch_ros [file, dict] merge that silently drops nested plugin params
    # (FollowPath.plugin -> DWB fallback), and avoids the included-scope arg-default timing
    # that left the path resolving to '' (file-not-found). The YAML already sets
    # use_sim_time: true in every section (this demo is sim-only).
    configured_params = default_params

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
        ),
        Node(
            package="nav2_controller", executable="controller_server", output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_planner", executable="planner_server", output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server", output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator", output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }],
        ),
    ]

    # Declared arguments MUST come before the nodes: when this file is included by another
    # launch file the actions run in order, so LaunchConfiguration("params_file") would
    # otherwise be read before its default is set (resolving to '' -> file-not-found).
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        *nodes,
    ])
