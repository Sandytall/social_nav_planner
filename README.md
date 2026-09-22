# SocialNav Planner

A research-grade, socially-aware **Nav2 local planner/controller** for ROS 2: it treats
humans differently from static obstacles and adapts the robot's motion around people —
keeping social clearance, yielding to crossers, and routing *around* people who block the
path. Built to the spec in [`MASTER_PROMPT.md`](MASTER_PROMPT.md).

> **Stack:** Ubuntu 22.04 · ROS 2 **Humble** · Gazebo Classic 11 · **MiR-100** robot.
> (The master prompt targeted Jazzy/Gazebo Harmonic; this is a documented deviation to the
> installed toolchain — the design is otherwise faithful. See `IMPLEMENTATION_STATUS.md`.)

## What problem does this solve?

A normal local planner treats a person as just another obstacle: it either stops dead or
skims past uncomfortably close. Social navigation needs the robot to **predict** where
people are going, respect their **personal space** (which is bigger in front of them than
behind), **yield** on collision courses, avoid cutting through **groups**, and still make
progress. This project does that as a drop-in Nav2 controller plus a social costmap layer.

## Architecture

```
/scan /odom  ─▶ Nav2 costmaps ─┬─▶ obstacle + inflation layers
/social_nav/humans ────────────┴─▶ SocialLayer  (anisotropic human cost)  ─▶ global planner routes AROUND people
                                                                                    │
global plan ─▶ SocialNavController (FollowPath plugin):                             ▼
   pure-pursuit base  →  candidate rollouts around it  →  hard collision reject
   →  social scoring (personal-space + TTC + group)  →  behavior mode (speed) →  /cmd_vel
```

- **Reliable path following** from a pure-pursuit base command; the social terms only
  *perturb* it, so with no people it behaves like a normal planner.
- **SocialLayer** injects each person's anisotropic cost into the global costmap so the
  **global planner** reroutes around blockers (soft cost — never makes a passable gap
  impassable).
- **Behavior modes** NORMAL/CAUTIOUS/CROWDED/EMERGENCY scale speed near people (hysteresis).

## Packages

| Package | Role |
|---|---|
| `social_nav_msgs` | HumanState/Array, PredictedHumanTrajectory, PlannerMetrics |
| `social_nav_core` | ROS-free math: anisotropic cost, TTC/closest-approach, trajectory gen, collision, scorer, modes (**68 gtests**) |
| `social_nav_prediction` | 3 human motion models (CV, CV+accel, uncertainty-aware) |
| `social_nav_human_model` | NN tracker, group detection, layered social zones |
| `social_nav_controller` | `nav2_core::Controller` plugin (pure-pursuit + social scoring) |
| `social_nav_costs` | `SocialLayer` costmap plugin (routes around people) |
| `social_nav_sim` | Gazebo Classic world + spawn |
| `social_nav_description` | MiR-100 URDF (diff-drive, SICK lidar) |
| `social_nav_bringup` | Nav2 params + launch (`demo.launch.py`) |
| `social_nav_tools` | simulated human publisher + RViz markers |
| `social_nav_rviz` | RViz config |
| `social_nav_benchmarks` | scenario benchmark framework (§29) |

## Install

```bash
sudo apt update && sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-diagnostic-updater \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-xacro \
  ros-humble-robot-state-publisher
# (or: ./scripts/install_dependencies.sh)
```

## Quick start

```bash
cd ~/Social_planner/social_nav_ws
colcon build --symlink-install
source install/setup.bash

# robot navigates to a goal, avoiding a person on its path (Gazebo + RViz):
HUMANS="-1.3,-0.7,0,0" ./scripts/run_demo.sh

# headless (RViz only, lighter on the GPU):
GUI=false HUMANS="-1.3,-0.7,0,0" ./scripts/run_demo.sh
```
Quit with **Ctrl-C**; if Gazebo sticks, `./scripts/stop.sh`.

## Tuning & benchmarking

- **Tune** every weight/speed: [`docs/TUNING.md`](docs/TUNING.md) (params live in
  `src/social_nav_bringup/config/nav2_params.yaml`; with `--symlink-install`, edit and
  relaunch — no rebuild).
- **Benchmark**: [`docs/BENCHMARKING.md`](docs/BENCHMARKING.md) —
  `ros2 run social_nav_benchmarks social-nav-benchmark --scenario all --runs 20`.

## Status, limitations, honesty

See [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md) for the phase-by-phase state and
measured results. Per the engineering rule (§62): incomplete items are marked TODO, unrun
benchmarks NOT RUN, and no performance number is fabricated. Measured controller latency:
p50 ≈ 0.08 ms, p95 ≈ 0.12 ms at 20 Hz (0 deadline misses).

**Current limitations:** full drive-*around* of a person requires the SocialLayer (done);
Isaac Sim photoreal track is deferred (GPU below spec — `docs/ISAAC_SIM_PLAN.md`); the
optional learned trajectory predictor (§50) is not implemented (classical planner first, §49).

## License
Apache-2.0.
