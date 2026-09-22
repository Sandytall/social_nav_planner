# SocialNav Planner

A socially-aware local planner for ROS 2 Nav2. It plugs in as a Nav2 controller and a
costmap layer so the robot keeps a comfortable distance from people, yields to those
crossing or approaching, and plans a path *around* someone standing in the way instead of
stopping in front of them.

Built and tested on ROS 2 Humble with Gazebo Classic and a MiR-100 differential-drive
robot. The planner only consumes standard ROS topics (`/scan`, `/odom`, TF,
`/social_nav/humans`) and publishes `/cmd_vel`, so it isn't tied to a particular simulator
or detector.

## How it works

- **`SocialNavController`** (a `nav2_core::Controller`): follows the global plan with a
  pure-pursuit base command, then samples candidate motions around it, rejects any that
  collide, and scores the rest by an anisotropic personal-space cost, time-to-collision,
  and group intrusion. Behaviour modes (normal/cautious/crowded/emergency) scale speed as
  people get closer.
- **`SocialLayer`** (a `nav2_costmap_2d::Layer`): stamps each person's personal-space cost
  into the global costmap so the global planner routes around people. The cost stays below
  lethal so a genuinely passable gap never becomes blocked.

See [docs/architecture.md](docs/architecture.md) for details.

## Requirements

- Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11
- Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`) and gazebo_ros

```bash
./scripts/install_dependencies.sh    # or apt-install the packages above
```

## Build and run

```bash
colcon build --symlink-install
source install/setup.bash

# navigate to a goal while avoiding a person on the route (Gazebo + RViz):
HUMANS="-1.3,-0.7,0,0" ./scripts/run_demo.sh
```

`GUI=false ./scripts/run_demo.sh` runs headless and shows the robot in RViz only, which is
lighter on the GPU. Stop with Ctrl-C; `./scripts/stop.sh` clears a stuck Gazebo.

## Packages

`social_nav_core` holds the planner math (no ROS deps, unit-tested with GoogleTest);
`social_nav_controller` and `social_nav_costs` are the Nav2 plugins;
`social_nav_{sim,description,bringup}` provide the simulation, robot, and launch;
`social_nav_tools` publishes simulated pedestrians and RViz markers;
`social_nav_benchmarks` runs the evaluation scenarios;
`social_nav_msgs` defines the human/metrics messages.

## Tuning and evaluation

- [docs/TUNING.md](docs/TUNING.md) — what each parameter does; edit
  `src/social_nav_bringup/config/nav2_params.yaml` and relaunch (no rebuild with
  `--symlink-install`).
- [docs/BENCHMARKING.md](docs/BENCHMARKING.md) — run the scenario benchmark:
  `ros2 run social_nav_benchmarks social-nav-benchmark --scenario all --runs 20`.

## Notes

The MiR-100 meshes are third-party (see [docs/ATTRIBUTION.md](docs/ATTRIBUTION.md)); the
rest is Apache-2.0. The demo is mapless (rolling costmap, no AMCL). A learned pedestrian
predictor is out of scope — prediction here is classical (constant velocity/acceleration
with growing uncertainty).

## License

Apache-2.0.
