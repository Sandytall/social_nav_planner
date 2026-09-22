"""Bring up Gazebo Classic with the SocialNav world and spawn the robot.

Headless-capable: pass gui:=false to run gzserver only (used by the smoke test and CI,
which must not need a display).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = get_package_share_directory("social_nav_description")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": verbose}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "social_bot",
            "-x", x, "-y", y, "-z", "0.12", "-Y", yaw,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("social_nav_sim"), "worlds", "social_world.world"]
            ),
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "verbose", default_value="false",
            description="gzserver --verbose logging (off by default; it slows the sim)"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x", default_value="-3.5"),
        DeclareLaunchArgument("y", default_value="-3.5"),
        DeclareLaunchArgument("yaw", default_value="0.9"),  # face the corridor gap to start
        gzserver,
        gzclient,
        rsp,
        spawn,
    ])
