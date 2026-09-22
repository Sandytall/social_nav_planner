"""One-shot SocialNav demo (MASTER_PROMPT §41): Gazebo sim + Nav2 + our controller.

Usage:
  ros2 launch social_nav_bringup demo.launch.py            # with Gazebo GUI + RViz
  ros2 launch social_nav_bringup demo.launch.py gui:=false # headless (CI / smoke test)

Then send a goal, e.g.:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_pkg = get_package_share_directory("social_nav_sim")
    bringup = get_package_share_directory("social_nav_bringup")

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Each include is wrapped in a scoped GroupAction so the launch configurations they
    # declare (notably gazebo's params_file, default '') do not leak between the two
    # sibling includes and turn a nav node's params path into '' (file-not-found).
    sim = GroupAction(scoped=True, forwarding=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, "launch", "sim.launch.py")
            ),
            launch_arguments={"gui": gui, "use_sim_time": use_sim_time}.items(),
        ),
    ])
    nav = GroupAction(scoped=True, forwarding=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, "launch", "navigation.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        sim,
        nav,
    ])
