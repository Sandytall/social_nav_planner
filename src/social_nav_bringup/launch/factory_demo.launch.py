"""SocialNav factory demo: Gazebo (factory.world) + robot + Nav2 + workers + RViz.

The robot is spawned at the world origin so the map, odom and Gazebo world frames coincide,
which lets the worker manager use one coordinate for both the Gazebo body and the
/social_nav/humans entry. Nav2 is reused as-is from navigation.launch.py; this file does not
fork the Nav2 configuration.

Usage:
  ros2 launch social_nav_bringup factory_demo.launch.py                    # scenario:=factory_crossing
  ros2 launch social_nav_bringup factory_demo.launch.py scenario:=factory_workers
  ros2 launch social_nav_bringup factory_demo.launch.py scenario:=factory_empty
  ros2 launch social_nav_bringup factory_demo.launch.py gui:=false         # headless (no GUI)

Then send a goal (keep it within ~10 m of the origin start, the rolling global-costmap
half-width), e.g. down the main lane to the 2.0 m aisle:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 8.5, y: 0.0}}}}"
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

# --- Factory geometry (map frame), authored to match factory.world. Kept here (not in a
# shared dir) so the scenario config lives with the launch, like urban_demo.launch.py. ---

# Collision footprints as flat [xmin, ymin, xmax, ymax] groups: the racks, machines, the
# charger cabinet and the guard rail. Used only for worker spawn-safety (workers are visual).
FACTORY_OBSTACLES = [
    1.0, 1.5, 2.0, 3.2,       # rack_n_west
    2.6, 0.7, 4.0, 3.2,       # rack_n_corner (occludes the walkway)
    5.2, 1.0, 9.3, 3.2,       # rack_n_east
    0.8, -3.2, 3.0, -1.5,     # rack_s_west
    5.2, -3.2, 9.3, -1.0,     # rack_s_east
    9.5, -2.9, 10.7, -1.5,    # machine_east
    9.5, 1.6, 10.7, 3.2,      # machine_ne
    -3.85, -5.6, -3.35, -4.8,  # charger_cabinet
    -3.8, 3.29, -1.4, 3.41,   # guard_rail
]

# Dwell stations for workstation workers, all inside the open west production area so any
# assigned route stays clear of the racks. Flat [x, y] pairs.
FACTORY_WORKSTATIONS = [
    -3.2, 2.4,
    -1.4, 2.6,
    -3.2, -0.4,
    -1.6, -2.4,
    -3.2, -2.6,
    0.4, 1.9,
    0.4, -1.8,
]

# North/south endpoints of the pedestrian walkway that crosses the robot lane at x ~ 4.6.
# Flat [nx, ny, sx, sy].
FACTORY_CROSSING_ROUTES = [4.6, 4.2, 4.6, -4.2]

# Interior walkable bounds (xmin, xmax, ymin, ymax), just inside the perimeter walls.
FACTORY_WALKABLE = [-3.8, 10.8, -6.3, 6.3]

# Each scenario is a set of worker-manager counts; the world, geometry and Nav2 stay the same.
# "factory_crossing" (default) puts a worker across the robot's lane via the walkway plus two
# dwell workers near the start; "factory_workers" is dwell-only; "factory_empty" has none.
SCENARIOS = {
    "factory_empty": {
        "num_workstation": 0,
        "num_crossing": 0,
    },
    "factory_workers": {
        "num_workstation": 3,
        "num_crossing": 0,
    },
    "factory_crossing": {
        "num_workstation": 2,
        "num_crossing": 1,
    },
}


def _workers(context, *_):
    scenario = LaunchConfiguration("scenario").perform(context)
    if scenario not in SCENARIOS:
        raise RuntimeError(
            f"unknown scenario {scenario!r}; choose one of {sorted(SCENARIOS)}")
    counts = SCENARIOS[scenario]
    if counts["num_workstation"] == 0 and counts["num_crossing"] == 0:
        return []  # factory_empty: no worker node needed
    params = {
        "seed": 42,
        "num_patrol": 0,
        "obstacles": FACTORY_OBSTACLES,
        "workstations": FACTORY_WORKSTATIONS,
        "crossing_routes": FACTORY_CROSSING_ROUTES,
        "walkable": FACTORY_WALKABLE,
        "use_sim_time": True,
    }
    params.update(counts)
    return [Node(
        package="social_nav_tools",
        executable="worker_manager",
        name="worker_manager",
        output="screen",
        parameters=[params],
    )]


def generate_launch_description():
    sim_pkg = get_package_share_directory("social_nav_sim")
    bringup = get_package_share_directory("social_nav_bringup")
    rviz_pkg = get_package_share_directory("social_nav_rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = os.path.join(sim_pkg, "worlds", "factory.world")
    rviz_cfg = os.path.join(rviz_pkg, "rviz", "social_nav.rviz")

    # Scoped so gazebo's params_file default ('') does not leak into the nav include and turn
    # a nav node's params path into '' (same guard as urban_demo.launch.py). Robot at origin.
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
            "scenario", default_value="factory_crossing",
            description="factory scenario: " + ", ".join(sorted(SCENARIOS))),
        sim,
        nav,
        markers,
        OpaqueFunction(function=_workers),
        rviz_node,
    ])
