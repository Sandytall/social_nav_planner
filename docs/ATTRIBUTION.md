# Attribution

## MiR-100 meshes

The MiR-100 visual meshes in `social_nav_description/meshes/mir/` (base, wheel, caster,
SICK lidar) were sourced from the **ros2_mir_nav2_pick_place** project, which in turn
adapted them from the [URDF files dataset](https://github.com/Daniella1/urdf_files_dataset).
They are used here only for visualization. If you publish or redistribute this repository,
verify the upstream mesh license and preserve the original attribution.

The URDF/kinematics in `mir_social.urdf.xacro` were re-authored for Gazebo Classic
(diff-drive + ray-sensor plugins) in this project; only the mesh files are third-party.

## Everything else

All other code in this repository is original to the SocialNav Planner and licensed
Apache-2.0 (see `LICENSE`). Architectural ideas were informed by public Nav2 / social-
navigation work (ROSNavBench, NavigationAnalyzer, Social Force planners) but no code was
copied.
