"""RL simulation stack: Gazebo + robot + pedestrians, WITHOUT the Nav2 controller.

The RL policy (via social_nav_rl's GazeboBackend) publishes /cmd_vel directly, so this launch
deliberately omits navigation.launch.py to avoid two controllers fighting over /cmd_vel. World,
robot start, pedestrian parameters and sensor profile all come from the environment/scenario
registry, matching what the RL env expects. Bring this up, then run social-nav-rl-eval
--backend gazebo (or train) in another terminal.

  ros2 launch social_nav_bringup rl_sim.launch.py environment:=factory scenario:=crossing
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from social_nav_tools.environments import generate_scenario

_SCAN_NOISE = {"clean": "0.0", "realistic": "0.01", "stress": "0.03"}


def _setup(context, *_):
    lc = LaunchConfiguration
    cfg = generate_scenario(lc("environment").perform(context), lc("scenario").perform(context),
                            lc("difficulty").perform(context), int(lc("seed").perform(context)))
    sim_pkg = get_package_share_directory("social_nav_sim")
    rviz_pkg = get_package_share_directory("social_nav_rviz")
    world = os.path.join(sim_pkg, "worlds", cfg["world"])
    sx, sy, syaw = cfg["robot_start"]
    ped = dict(cfg["pedestrian"])
    ped["use_sim_time"] = True

    return [
        SetEnvironmentVariable("SOCIAL_NAV_SCAN_NOISE",
                               _SCAN_NOISE.get(cfg["sensor_profile"], "0.01")),
        GroupAction(scoped=True, forwarding=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "sim.launch.py")),
                launch_arguments={"gui": lc("gui"), "use_sim_time": "true", "world": world,
                                  "x": str(sx), "y": str(sy), "yaw": str(syaw)}.items()),
        ]),
        Node(package="social_nav_tools", executable="pedestrian_manager",
             name="pedestrian_manager", output="screen", parameters=[ped]),
        Node(package="social_nav_tools", executable="human_markers",
             name="human_markers", output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(rviz_pkg, "rviz", "rl_sim.rviz")],
             output="log", condition=IfCondition(lc("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        DeclareLaunchArgument("environment", default_value="factory"),
        DeclareLaunchArgument("scenario", default_value="crossing"),
        DeclareLaunchArgument("difficulty", default_value="medium"),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
