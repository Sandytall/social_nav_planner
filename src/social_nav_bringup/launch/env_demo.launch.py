"""Generic SocialNav environment demo, driven by the environment/scenario registry.

One launch for every environment: it asks social_nav_tools.environments.generate_scenario for
the world, robot start/goal, sensor profile, pedestrian parameters and dynamic-obstacle/robot
counts, then brings up Gazebo + robot + Nav2 (reused, unforked) + pedestrians + optional moving
obstacles + RViz. The planner sees nothing environment-specific.

Usage:
  ros2 launch social_nav_bringup env_demo.launch.py environment:=warehouse scenario:=crossing \
      difficulty:=hard seed:=42
  ros2 launch social_nav_bringup env_demo.launch.py environment:=urban scenario:=group gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from social_nav_tools.environments import generate_scenario

_SCAN_NOISE = {"clean": "0.0", "realistic": "0.01", "stress": "0.03"}


def _setup(context, *_):
    lc = LaunchConfiguration
    environment = lc("environment").perform(context)
    scenario = lc("scenario").perform(context)
    difficulty = lc("difficulty").perform(context)
    seed = int(lc("seed").perform(context))
    sensor_override = lc("sensor").perform(context) or None
    robots_override = lc("num_robots").perform(context)

    cfg = generate_scenario(environment, scenario, difficulty, seed,
                            sensor_profile=sensor_override)
    if robots_override and int(robots_override) > 0:
        cfg["num_robots"] = int(robots_override)

    sim_pkg = get_package_share_directory("social_nav_sim")
    bringup = get_package_share_directory("social_nav_bringup")
    rviz_pkg = get_package_share_directory("social_nav_rviz")
    world = os.path.join(sim_pkg, "worlds", cfg["world"])
    sx, sy, syaw = cfg["robot_start"]

    gui = lc("gui")
    rviz = lc("rviz")
    use_sim_time = lc("use_sim_time")

    ped = dict(cfg["pedestrian"])
    ped["use_sim_time"] = True

    # AMR agents = extra robots (num_robots-1) + non-human dynamic obstacles, all as moving
    # obstacles the main robot must avoid.
    num_agents = max(0, cfg["num_robots"] - 1) + cfg["num_dynamic_obstacles"]

    actions = [
        # Sensor profile must be set before robot_state_publisher runs xacro (it reads the env).
        SetEnvironmentVariable("SOCIAL_NAV_SCAN_NOISE",
                               _SCAN_NOISE.get(cfg["sensor_profile"], "0.01")),
        GroupAction(scoped=True, forwarding=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_pkg, "launch", "sim.launch.py")),
                launch_arguments={"gui": gui, "use_sim_time": use_sim_time, "world": world,
                                  "x": str(sx), "y": str(sy), "yaw": str(syaw)}.items()),
        ]),
        GroupAction(scoped=True, forwarding=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup, "launch", "navigation.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items()),
        ]),
        Node(package="social_nav_tools", executable="human_markers",
             name="human_markers", output="screen"),
        Node(package="social_nav_tools", executable="pedestrian_manager",
             name="pedestrian_manager", output="screen", parameters=[ped]),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(rviz_pkg, "rviz", "social_nav.rviz")],
             output="log", condition=IfCondition(rviz)),
    ]
    if num_agents > 0:
        actions.append(Node(
            package="social_nav_tools", executable="robot_agent_manager",
            name="robot_agent_manager", output="screen",
            parameters=[{"use_sim_time": True, "num_agents": num_agents}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        DeclareLaunchArgument("environment", default_value="urban",
                              description="urban | factory | warehouse"),
        DeclareLaunchArgument("scenario", default_value="normal",
                              description="scenario type (see environments.SCENARIO_TYPES)"),
        DeclareLaunchArgument("difficulty", default_value="medium",
                              description="easy | medium | hard | stress"),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("sensor", default_value="",
                              description="override sensor profile: clean | realistic | stress"),
        DeclareLaunchArgument("num_robots", default_value="",
                              description="override total robot count (>=1)"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
