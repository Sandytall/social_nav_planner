# Troubleshooting

Issues actually hit while building this project, with fixes.

## Gazebo

- **`gzclient` stuck at "Preparing your world"** — it's hanging on the (dead) online model
  database. `run_demo.sh` sets `GAZEBO_MODEL_DATABASE_URI=""`; if you launch manually,
  export that too. If it still hangs, it's a GPU/OpenGL limit — run headless
  (`GUI=false ./scripts/run_demo.sh`) and watch in RViz instead.
- **Can't quit Gazebo / next launch fails "Address already in use [11345]"** — `gzserver`/
  `gzclient` ignore Ctrl-C and linger. Run `./scripts/stop.sh` (SIGKILLs them). Killing the
  `ros2 launch` process does **not** reliably reap them.
- **Robot has no floor / `Unable to find uri[model://ground_plane]`** — `GAZEBO_MODEL_PATH`
  must include `/usr/share/gazebo-11/models`. `run_demo.sh` handles this.
- **Only wheels render, no body** — `gzclient` can't resolve `package://` meshes. The MiR
  meshes use absolute `file://$(find social_nav_description)/...` paths to avoid this.

## Nav2 / controller

- **`lifecycle_manager` dies: `libdiagnostic_updater.so: cannot open shared object`** —
  install `ros-humble-diagnostic-updater`.
- **`controller_server: No critics defined for FollowPath`** — the params didn't reach the
  node and it fell back to DWB. Pass the params as a single concrete file (not `[file, dict]`
  which launch_ros merges and drops nested plugin params).
- **"Goal was rejected"** — the goal was sent before `bt_navigator` finished activating.
  Wait for `Managed nodes are active` (run_demo does).
- **Occasional `ABORTED / BtActionNode::Tick: invalid status value`** — a known Nav2-Humble
  `bt_action_node` result race (~1 in 5). The robot still reaches the goal; just re-send.
- **Robot crawls / stalls, planner very slow** — build in Release (`-O2`). The core CMake
  defaults to Release; a mixed `-O0` build makes the per-cycle planner far too slow.
- **Robot stops near a person instead of going around** — the SocialLayer must be in the
  global costmap `plugins` list (it is, by default). Local scoring alone can only stop/yield.

## Build

- **`failed to create symbolic link ... Is a directory`** — you mixed plain and
  `--symlink-install` builds. Fix: `rm -rf build install log && colcon build --symlink-install`.
- **`AMENT_TRACE_SETUP_FILES: unbound variable`** — don't use `set -u` in scripts that
  source ROS `setup.bash`. Use `set -eo pipefail`.

## Humans not affecting the robot

- **Robot drives through a person** — the human data is likely stale: publishers must stamp
  on **sim time** (`use_sim_time:=true`), or the controller/SocialLayer drop it. The
  `human_publisher` node does this.
- Also confirm `/social_nav/humans` is actually publishing (`ros2 topic hz /social_nav/humans`).
