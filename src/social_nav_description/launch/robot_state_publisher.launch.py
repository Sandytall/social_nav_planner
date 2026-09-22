"""Publish the SocialNav robot description + TF (MASTER_PROMPT §27)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("social_nav_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")  # xacro filename in urdf/
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(pkg, "urdf", ""), model]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "model", default_value="mir_social.urdf.xacro",
            description="Robot xacro in social_nav_description/urdf/ "
                        "(mir_social.urdf.xacro or social_bot.urdf.xacro)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        ),
    ])
