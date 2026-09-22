"""SocialNav urban demo: Gazebo (urban.world) + robot + Nav2 + pedestrians + RViz.

The robot is spawned at the world origin so the map, odom and Gazebo world frames coincide,
which lets the pedestrian manager use one coordinate for both the Gazebo body and the
/social_nav/humans entry. Nav2 is reused as-is from navigation.launch.py; this file does not
fork the Nav2 configuration.

Usage:
  ros2 launch social_nav_bringup urban_demo.launch.py                 # scenario:=crossing
  ros2 launch social_nav_bringup urban_demo.launch.py scenario:=walking
  ros2 launch social_nav_bringup urban_demo.launch.py gui:=false      # headless (no GUI)

Then send a goal (keep it within ~10 m of the origin start, the rolling global-costmap
half-width), e.g.:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 9.0, y: 0.0}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Each scenario is just a set of pedestrian-manager parameters; the world and Nav2 stay the
# same. Both wire end to end. "crossing" (default) guarantees a pedestrian across the robot's
# path via the "mixed" profile (even indices cross).
SCENARIOS = {
    "empty": {
        "num_humans": 0,
        "behavior_profile": "walking",
        "seed": 42,
        "spawn_region": [1.0, 12.0, -0.5, 1.4],
    },
    "single_crossing": {
        "num_humans": 1,
        "behavior_profile": "crossing",
        "seed": 42,
        "spawn_region": [1.0, 12.0, -0.5, 1.4],
    },
    "crossing": {
        "num_humans": 3,
        "behavior_profile": "mixed",
        "seed": 42,
        "spawn_region": [1.0, 12.0, -0.5, 1.4],
    },
    "walking": {
        "num_humans": 3,
        "behavior_profile": "walking",
        "seed": 42,
        "spawn_region": [1.0, 12.0, -0.5, 1.4],
    },
    "group": {
        "num_humans": 4,
        "behavior_profile": "group",
        "group_size": 4,
        "seed": 42,
        "spawn_region": [1.0, 12.0, -0.5, 1.4],
    },
    "crowded": {
        "num_humans": 8,
        "behavior_profile": "mixed",
        "seed": 42,
        "spawn_region": [1.0, 12.0, -1.0, 1.4],
    },
    "head_on": {
        "num_humans": 1,
        "behavior_profile": "head_on",
        "seed": 42,
        "spawn_region": [1.0, 12.0, -0.5, 1.4],
    },
    "turning": {
        "num_humans": 2,
        "behavior_profile": "turning",
        "seed": 42,
        "spawn_region": [1.0, 12.0, -0.5, 1.4],
    },
    "same_direction": {"num_humans": 1, "behavior_profile": "same_direction",
                       "seed": 42, "spawn_region": [1.0, 12.0, -0.5, 1.4]},
    "sudden_stop": {"num_humans": 1, "behavior_profile": "stop_go",
                    "seed": 42, "spawn_region": [1.0, 12.0, -0.5, 1.4]},
    "blocker": {"num_humans": 1, "behavior_profile": "blocker",
                "seed": 42, "spawn_region": [1.0, 12.0, -0.5, 1.4]},
    "two_crossing": {"num_humans": 2, "behavior_profile": "crossing",
                     "seed": 42, "spawn_region": [1.0, 12.0, -0.5, 1.4]},
    "group_merge": {"num_humans": 4, "behavior_profile": "merge",
                    "seed": 42, "spawn_region": [1.0, 12.0, -1.4, 1.4]},
    "high_density": {"num_humans": 12, "behavior_profile": "mixed",
                     "seed": 42, "spawn_region": [1.0, 12.0, -1.4, 1.4]},
    "mixed": {"num_humans": 5, "behavior_profile": "mixed",
              "seed": 42, "spawn_region": [1.0, 12.0, -1.0, 1.4]},
}


def _pedestrians(context, *_):
    scenario = LaunchConfiguration("scenario").perform(context)
    if scenario not in SCENARIOS:
        raise RuntimeError(
            f"unknown scenario {scenario!r}; choose one of {sorted(SCENARIOS)}")
    params = dict(SCENARIOS[scenario])
    params["use_sim_time"] = True
    override_h = int(LaunchConfiguration("num_humans").perform(context))
    if override_h >= 0:
        params["num_humans"] = override_h  # GUI/CLI override of the scenario's people count
    nodes = [Node(
        package="social_nav_tools",
        executable="pedestrian_manager",
        name="pedestrian_manager",
        output="screen",
        parameters=[params],
    )]
    # Secondary AMRs (multi-robot): num_robots-1 simple moving-obstacle agents the main robot
    # must avoid. They are plain obstacles seen by the lidar, not part of the social problem.
    num_robots = max(1, int(LaunchConfiguration("num_robots").perform(context)))
    if num_robots > 1:
        nodes.append(Node(
            package="social_nav_tools",
            executable="robot_agent_manager",
            name="robot_agent_manager",
            output="screen",
            parameters=[{"use_sim_time": True, "num_agents": num_robots - 1}],
        ))
    return nodes


def generate_launch_description():
    sim_pkg = get_package_share_directory("social_nav_sim")
    bringup = get_package_share_directory("social_nav_bringup")
    rviz_pkg = get_package_share_directory("social_nav_rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = os.path.join(sim_pkg, "worlds", "urban.world")
    rviz_cfg = os.path.join(rviz_pkg, "rviz", "social_nav.rviz")

    # Scoped so gazebo's params_file default ('') does not leak into the nav include and turn
    # a nav node's params path into '' (same guard as demo.launch.py). Robot at the origin.
    sim = GroupAction(scoped=True, forwarding=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "sim.launch.py")),
            launch_arguments={
                "gui": gui,
                "use_sim_time": use_sim_time,
                "world": world,
                "x": "0.0", "y": "0.0", "yaw": "0.0",
            }.items(),
        ),
    ])
    nav = GroupAction(scoped=True, forwarding=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ])

    markers = Node(
        package="social_nav_tools", executable="human_markers",
        name="human_markers", output="screen",
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg], output="log",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        # Proven Gazebo Classic env: only the built-in models, and no dead online DB fetch
        # (which otherwise hangs gzclient at "Preparing your world").
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "scenario", default_value="crossing",
            description="pedestrian scenario: " + ", ".join(sorted(SCENARIOS))),
        DeclareLaunchArgument(
            "num_robots", default_value="1",
            description="total robots; >1 spawns (num_robots-1) secondary AMR obstacle agents"),
        DeclareLaunchArgument(
            "num_humans", default_value="-1",
            description="override the scenario's people count; -1 keeps the scenario default"),
        sim,
        nav,
        markers,
        OpaqueFunction(function=_pedestrians),
        rviz_node,
    ])
