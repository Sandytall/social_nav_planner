# MASTER PROMPT — SocialNav Planner

> **Status:** This is the canonical, binding specification for the project. It is the
> single source of truth. All implementation work must follow it. It is transcribed
> faithfully from `PROJECT Socially-Aware Local.docx`.
>
> **Engineering rule (§62) applies to this document too:** do not pretend a feature
> works. Mark incomplete work `TODO`, unrun benchmarks `NOT RUN`, simulated results
> `SIMULATED`, assumptions `ASSUMPTION`. Never fabricate performance numbers.

---

## ROLE

Act as a senior robotics software engineer, autonomous navigation researcher, ROS 2 /
Nav2 developer, and robotics R&D engineer.

You are building a serious research-grade and production-oriented open-source robotics
project called:

**SocialNav Planner**

The goal is to implement a custom Nav2 local planner/controller that treats humans
differently from static obstacles and dynamically adapts robot behavior around people.

**This is NOT a toy project.**

The final repository should demonstrate competence in:

- C++
- ROS 2
- Nav2
- plugin architecture
- local trajectory planning
- dynamic obstacle prediction
- human-aware navigation
- costmap integration
- trajectory scoring
- collision checking
- behavior around groups
- recovery behaviors
- real-time constraints
- simulation
- automated testing
- benchmarking
- Docker
- CI
- observability
- reproducible experiments

The implementation must be designed so that another engineer can clone the repository,
build it, launch simulation, run experiments, and reproduce the reported results.

---

## 1. PRIMARY OBJECTIVE

Create a custom Nav2 local planner that generates safe and socially acceptable
trajectories around humans.

The planner must distinguish at minimum:

1. Static obstacles
2. Unknown obstacles
3. Dynamic non-human obstacles
4. Individual humans
5. Groups of humans
6. Humans approaching the robot
7. Humans walking in the same direction
8. Humans crossing the robot's path
9. Stationary humans
10. Humans whose future trajectory is uncertain

The planner should optimize a trajectory using multiple costs:

```
J = w_goal      * GoalCost
  + w_path      * PathFollowingCost
  + w_obs       * StaticObstacleCost
  + w_dyn       * DynamicObstacleCost
  + w_human     * HumanSocialCost
  + w_ttc       * TimeToCollisionCost
  + w_clearance * HumanClearanceCost
  + w_smooth    * SmoothnessCost
  + w_vel       * VelocityCost
  + w_progress  * ProgressCost
  + w_group     * GroupIntrusionCost
  + w_direction * SocialDirectionCost
```

The weights must be configurable through ROS 2 parameters.

**Do NOT hard-code behavioral thresholds.**

---

## 2. TECHNOLOGY REQUIREMENTS

Use:

- Ubuntu 24.04
- ROS 2 Jazzy
- C++17 or C++20 where compatible with ROS 2 Jazzy
- Python 3 for testing, benchmarking and tooling
- Nav2
- Gazebo Harmonic
- RViz2
- TF2
- Eigen
- pluginlib
- rclcpp
- nav_msgs
- geometry_msgs
- sensor_msgs
- visualization_msgs
- tf2
- tf2_ros
- costmap_2d / Nav2 costmap interfaces
- GoogleTest
- pytest where useful
- rosbag2
- Docker
- GitHub Actions

Prefer C++ for the actual planner.

Python may be used for:

- experiment orchestration
- scenario generation
- benchmarking
- result analysis
- plotting
- dataset processing
- automated evaluation

**Do not implement the core planner in Python.**

---

## 3. REPOSITORY STRUCTURE

Create a professional ROS 2 workspace:

```
social_nav_ws/
  src/
    social_nav_msgs/
    social_nav_core/
    social_nav_controller/
    social_nav_human_model/
    social_nav_prediction/
    social_nav_costs/
    social_nav_sim/
    social_nav_bringup/
    social_nav_tools/
    social_nav_benchmarks/
    social_nav_tests/
    social_nav_description/
    social_nav_rviz/
    social_nav_docs/
  Dockerfile
  docker-compose.yml
  README.md
  LICENSE
  CONTRIBUTING.md
  CHANGELOG.md
  .gitignore
  .github/
    workflows/
      build.yml
      test.yml
      simulation.yml
      benchmark.yml
```

---

## 4. ARCHITECTURE

Use a modular architecture.

High-level system:

```
Sensor inputs
    ↓
Human Detection
    ↓
Human Tracking
    ↓
Trajectory Prediction
    ↓
Human World Model
    ↓
Social Cost Model
    ↓
Candidate Trajectory Generator
    ↓
Collision Checking
    ↓
Trajectory Scoring
    ↓
Best Safe Trajectory
    ↓
Nav2 Controller API
    ↓
cmd_vel
```

The planner must NOT directly depend on a specific human detector.
The planner should consume a generic human representation.

---

## 5. HUMAN REPRESENTATION

Create a ROS 2 message `HumanState.msg`. Suggested fields:

- `uint64 id`
- `geometry_msgs/Pose pose`
- `geometry_msgs/Twist velocity`
- `float32[] covariance`
- `float32 confidence`
- `float32 tracking_age`
- `bool stationary`
- `bool group_member`
- `int32 group_id`

Also create `PredictedHumanTrajectory.msg`. Fields:

- `human_id`
- `geometry_msgs/Pose[] predicted_poses`
- `geometry_msgs/Twist[] predicted_velocities`
- `float32[] prediction_confidence`
- `float32 prediction_horizon`
- `float32 timestep`

Create `HumanArray.msg` containing:

- `header`
- `HumanState[]`

Create `HumanPredictionArray.msg` containing:

- `header`
- `PredictedHumanTrajectory[]`

Use timestamps correctly. **Never assume sensor and planner timestamps are identical.**

---

## 6. HUMAN DETECTION INTERFACE

Create an abstract interface. The planner must work with:

- A. simulated humans
- B. externally supplied tracked humans
- C. future perception systems

Provide a simple simulated human publisher.

- Topic: `/social_nav/humans`
- Optional: `/social_nav/human_predictions`

**The system should continue operating if human detections disappear. This is critical.**

---

## 7. HUMAN TRACKING

Implement a basic tracker suitable for simulation and development. At minimum:

- nearest-neighbor association
- track creation
- track timeout
- velocity estimation
- acceleration estimation
- exponential smoothing

Optional advanced implementation: Kalman filter.

The tracker must handle:

- temporary detection loss
- ID switching
- stationary humans
- sudden direction changes
- noisy detections
- humans entering/leaving the field of view

**Never immediately delete a track because of one missing detection.**

Use configurable: `human_track_timeout`, `velocity_filter_alpha`, `max_association_distance`.

---

## 8. HUMAN MOTION PREDICTION

Implement at least three prediction models:

- **Model A — Constant Velocity:** `p(t) = p0 + v*t`
- **Model B — Constant Velocity + Acceleration:** `p(t) = p0 + v*t + 0.5*a*t²`
- **Model C — Uncertainty-Aware Prediction:** represent future position as a Gaussian
  distribution. Prediction uncertainty must increase with time. Example:
  0.5 s → low uncertainty, 1 s → moderate, 2 s → larger, 3 s → high.

The planner should become more conservative as uncertainty increases.

The prediction horizon must be configurable. Example: `prediction_horizon: 3.0`,
`prediction_dt: 0.1`.

---

## 9. HUMAN SOCIAL ZONE

Do NOT treat humans as simple circles. Implement layered social zones. At minimum:

1. Personal zone
2. Comfort zone
3. Caution zone

Make these anisotropic. The zone should depend on human heading. A human walking
forward should have a different cost distribution in front versus behind. For example:
`front clearance > side clearance > rear clearance`.

Do not use these values blindly. Make every parameter configurable.

---

## 10. ANISOTROPIC SOCIAL COST

Implement a cost function based on the human's orientation. For a candidate robot
position `p`: transform `p` into the human's local coordinate frame. Calculate:

- `x` = longitudinal distance
- `y` = lateral distance

Then calculate an anisotropic Gaussian or elliptical cost:

```
C = exp( -(x² / 2σx²) -(y² / 2σy²) )
```

where `σx` and `σy` depend on: human velocity, human direction, uncertainty, and
whether the robot is approaching from front/rear/side.

The system should not force the robot to avoid every human from every direction equally.

---

## 11. APPROACHING HUMAN DETECTION

Calculate relative velocity. Given robot position `R`, human position `H`:

- relative position: `d = H - R`
- relative velocity: `v_rel = v_human - v_robot`

Calculate **Time To Closest Approach** and **Distance At Closest Approach**.

If `TCA < threshold` and `DCA < threshold`, increase social collision cost.

**Do NOT rely solely on Euclidean distance.** A human 1.5 m away moving away from the
robot is different from a human 1.5 m away moving directly toward it.

---

## 12. CROSSING HUMAN DETECTION

Detect situations where the robot trajectory intersects or approaches the predicted
human trajectory. Calculate:

- intersection time
- minimum separation
- relative velocity

Increase cost for trajectories that create uncomfortable or dangerous interactions.

---

## 13. GROUP DETECTION

Implement basic human grouping. Humans close together and moving similarly can be
classified as a group. Group properties:

- group centroid
- group velocity
- group bounding area
- group members

The robot should not attempt to drive through the middle of a group simply because each
individual is technically outside the robot footprint.

Create `GroupIntrusionCost`. The planner should generally prefer going around a group.

---

## 14. STATIC OBSTACLE HANDLING

Integrate properly with Nav2 costmaps. **Do NOT replace Nav2's static obstacle
handling.** The social planner must add human-aware reasoning on top of standard
obstacle avoidance.

Architecture: `Nav2 costmap + dynamic human layer / social layer + planner scoring`.

The planner must still correctly handle: walls, tables, boxes, narrow corridors,
unknown space, inflation, robot footprint.

---

## 15. SOCIAL COSTMAP LAYER

Create an optional custom Nav2 costmap layer `SocialLayer`. It should:

1. Subscribe to human states
2. Predict human positions
3. Generate social cost fields
4. Insert them into the Nav2 layered costmap
5. Remove stale human costs
6. Update at configurable frequency

Parameters: `enabled`, `update_frequency`, `human_radius`, `comfort_radius`,
`caution_radius`, `prediction_horizon`, `prediction_dt`, `decay_rate`, `front_sigma`,
`side_sigma`, `rear_sigma`, `group_radius`.

---

## 16. LOCAL TRAJECTORY GENERATION

Generate candidate trajectories using velocity samples. Sample `vx`, `vy`, `omega`
where supported by robot type. For differential drive: `vx`, `omega`.

For each sample, simulate forward. Example: trajectory duration = 2.5 s, dt = 0.1 s.

Each candidate contains: `x`, `y`, `theta`, `velocity`, `angular velocity`, `timestamp`.

---

## 17. TRAJECTORY SIMULATION

For each candidate trajectory:

1. Integrate robot motion.
2. Check static collision.
3. Check dynamic collision.
4. Check human social zones.
5. Check predicted human trajectories.
6. Check robot footprint.
7. Calculate all costs.
8. Reject unsafe trajectories.
9. Score remaining trajectories.

**The planner must NEVER select an unsafe trajectory simply because its total cost is
lower. Safety is a hard constraint. Social preference is a soft optimization objective.**

---

## 18. HARD SAFETY CONSTRAINTS

A trajectory must be rejected if:

- robot footprint collides with obstacle
- predicted human collision occurs
- robot exceeds maximum velocity
- robot exceeds maximum acceleration
- robot exceeds maximum angular velocity
- trajectory leaves valid costmap region
- trajectory enters lethal obstacle cost
- TF is invalid
- command becomes stale
- planner computation exceeds its deadline

**Never trade collision safety for goal progress.**

---

## 19. SOCIAL TRAJECTORY SCORING

Implement separate cost terms. For each trajectory: `GoalCost`, `PathCost`,
`ObstacleCost`, `HumanClearanceCost`, `HumanPredictionCost`, `TimeToCollisionCost`,
`GroupIntrusionCost`, `DirectionCost`, `SmoothnessCost`, `VelocityCost`, `ProgressCost`.

Each term should be independently measurable. Example output:

```
Trajectory #37:
  Goal:        0.31
  Path:        0.12
  Obstacle:    0.00
  Human:       0.43
  TTC:         0.71
  Group:       0.02
  Smoothness:  0.08
  Total:       1.67
```

This is important for debugging.

---

## 20. SOCIAL BEHAVIOR MODES

Implement modes: `NORMAL`, `CAUTIOUS`, `CROWDED`, `EMERGENCY`.

The mode can depend on: number of humans, minimum human clearance, uncertainty, TTC,
environment density. Example:

- NORMAL: normal speed
- CAUTIOUS: reduced speed
- CROWDED: lower speed + increased clearance
- EMERGENCY: stop / controlled braking

**Make transitions hysteresis-based to prevent mode oscillation.**

---

## 21. EDGE CASES

Explicitly handle all of these:

- **Human suddenly stops** — Prediction must adapt.
- **Human suddenly changes direction** — Increase uncertainty.
- **Human disappears** — Maintain track temporarily, then decay it.
- **Human detection flickers** — Do not create/delete tracks every frame.
- **Two humans cross** — Maintain separate IDs.
- **Two humans merge into a group** — Detect grouping.
- **Group splits** — Remove group membership appropriately.
- **Robot is surrounded** — Planner must slow/stop safely.
- **Narrow corridor** — Do not attempt impossible social clearance. Recognize when no
  socially comfortable trajectory exists and select the safest feasible trajectory.
- **Human approaches robot head-on** — Calculate relative velocity and TTC.
- **Human walking behind robot** — Do not unnecessarily block the robot.
- **Human walking same direction** — Prefer safe overtaking or following behavior.
- **Human standing near goal** — Do not blindly drive into them to reach the goal.
- **Human blocks goal** — Trigger replanning / wait / alternate approach.
- **No humans detected** — Planner should behave like a normal local planner.
- **Sensor failure** — Planner must fail safely.
- **TF failure** — Do not generate stale commands.
- **Costmap unavailable** — Stop safely.
- **Planner timeout** — Return failure to Nav2 and allow recovery behavior.

---

## 22. NARROW CORRIDOR BEHAVIOR

Explicitly test: robot width = 0.6 m; corridor width 1.2 / 1.5 / 2.0 / 2.5 m.

The robot must determine whether passing is feasible. Do not blindly maintain a fixed
social radius that makes navigation impossible.

Implement **Feasible clearance** vs **Preferred clearance**. This distinction is important.

---

## 23. DEADLOCK HANDLING

Handle situations like: Human A approaches from left, Human B approaches from right,
robot cannot pass.

The planner should not oscillate left → right → left → right. Implement hysteresis and
short-term commitment. Potentially use: trajectory commitment, direction persistence,
hysteresis penalty, previous-command penalty. **Measure oscillation frequency.**

---

## 24. RECOVERY BEHAVIOR

If no valid trajectory exists:

1. Reduce speed.
2. Recalculate.
3. Attempt controlled stop.
4. Wait for humans to move if appropriate.
5. Trigger Nav2 recovery behavior.
6. Replan global path if necessary.

**Never blindly rotate in a crowded environment.**

---

## 25. NAV2 PLUGIN

Implement a proper Nav2 controller plugin. Use the correct Nav2 controller interface for
the target ROS 2 distribution. The plugin should expose: `configure()`, `activate()`,
`deactivate()`, `cleanup()`, `setPlan()`, `computeVelocityCommands()`, `setSpeedLimit()`.

Use pluginlib correctly. Register the plugin. **Do not create a fake interface merely to
demonstrate the concept. It must be loadable by Nav2.**

---

## 26. PARAMETERS

Create a YAML configuration. Example:

```yaml
social_nav_controller:
  ros__parameters:
    prediction_horizon: 3.0
    prediction_dt: 0.1
    max_linear_velocity: 0.8
    max_angular_velocity: 1.2
    max_linear_acceleration: 0.5
    max_angular_acceleration: 1.0
    preferred_human_clearance: 1.5
    minimum_human_clearance: 0.8
    front_social_sigma: 1.5
    side_social_sigma: 1.0
    rear_social_sigma: 0.7
    human_track_timeout: 1.0
    group_detection_radius: 1.5
    weights:
      goal: 1.0
      path: 2.0
      obstacle: 10.0
      human: 5.0
      ttc: 20.0
      group: 6.0
      smoothness: 1.0
      progress: 2.0
```

All parameters should be validated at startup. Invalid parameters must produce clear errors.

---

## 27. SIMULATION

Use Gazebo (target: Harmonic). Create a simulated differential-drive robot. The
environment should contain: rooms, corridors, doors, tables, walls, intersections.

Create simulated humans supporting: walking, stopping, turning, random paths, crossing,
group walking.

Create at least **10 deterministic scenarios**.

---

## 28. REQUIRED TEST SCENARIOS

- **01** Empty environment → Planner behaves like conventional local planner.
- **02** One stationary human → Robot passes with social clearance.
- **03** Human crossing robot → Robot slows or changes path.
- **04** Human walking toward robot → Robot predicts collision and reacts.
- **05** Human walking same direction → Robot does not behave excessively conservatively.
- **06** Two humans walking together → Robot treats them as a group.
- **07** Crowded corridor → Robot slows and avoids oscillation.
- **08** Human suddenly changes direction → Robot increases caution.
- **09** Human detection disappears → Track decays safely.
- **10** Impossible passage → Robot stops/replans rather than forcing passage.
- **11** Sensor timestamp delay → Planner detects stale human data.
- **12** TF failure → Safe stop.
- **13** Planner computation overload → Watchdog catches deadline violation.
- **14** Multiple humans crossing simultaneously.
- **15** Human standing at goal.

---

## 29. BENCHMARKING

Create an automated benchmarking framework. Run every scenario against:

1. Baseline Nav2 controller
2. SocialNav controller

Compare: success rate, collision rate, minimum human distance, average human distance,
time to goal, path length, path efficiency, average velocity, number of stops, number of
replans, oscillations, planner computation time, CPU usage, memory, TTC violations,
social-zone intrusions.

**Do not cherry-pick successful runs.** Run at least 20 trials per stochastic scenario.
Use deterministic random seeds when reproducibility is required.

---

## 30. PERFORMANCE REQUIREMENTS

The planner should report: planning frequency, average computation time, p50, p95, p99,
maximum computation time, deadline misses. Example:

```
Planner:
  20 Hz target
  p50 = 8 ms
  p95 = 15 ms
  p99 = 22 ms
  deadline misses = 0
```

**Do not claim real-time performance without measuring it.**

---

## 31. VISUALIZATION

Create RViz markers for: humans, human IDs, human velocity vectors, predicted
trajectories, social zones, groups, candidate trajectories, selected trajectory,
collision points, closest approach point, TTC, planner mode.

Use different marker types and namespaces. Provide an RViz configuration file.

---

## 32. DEBUG MODE

Add `debug_mode: true`. When enabled, publish:

- `/social_nav/debug/candidate_trajectories`
- `/social_nav/debug/selected_trajectory`
- `/social_nav/debug/human_predictions`
- `/social_nav/debug/social_cost`
- `/social_nav/debug/collision_points`
- `/social_nav/debug/metrics`

When disabled, avoid expensive debug calculations wherever possible.

---

## 33. LOGGING

Use structured ROS logging. Example:

```
[INFO]  Social planner activated.
[WARN]  Human prediction stale: ID=14, age=0.42s
[WARN]  No socially preferred trajectory found.
[ERROR] Controller computation exceeded deadline: 74ms / 50ms
```

**Do not spam logs every cycle.** Use throttled logging where appropriate.

---

## 34. FAILURE MODES

Define explicit failure states: `NO_HUMAN_DATA`, `STALE_HUMAN_DATA`,
`NO_VALID_TRAJECTORY`, `TF_FAILURE`, `COSTMAP_FAILURE`, `PLANNER_TIMEOUT`,
`INVALID_PLAN`, `EMERGENCY_STOP`.

For each state define: detection condition, safe behavior, recovery behavior, logging,
metric.

---

## 35. UNIT TESTS

Use GoogleTest. Test: human prediction, social cost, TTC, trajectory collision, group
detection, parameter validation, track timeout, direction classification, trajectory
scoring, mode switching, hysteresis, planner timeout, edge cases.

**At least 50 meaningful unit tests.** Do not create fake tests that only test
getters/setters.

---

## 36. INTEGRATION TESTS

Create ROS 2 integration tests that launch: Nav2, robot, simulator, human publisher,
SocialNav controller.

Verify: plugin loads, topics exist, commands are produced, planner reacts to humans,
recovery works, stale data causes safe behavior.

---

## 37. SYSTEM TESTS

Create end-to-end tests. Example: launch simulation → spawn robot → spawn human → send
navigation goal → human crosses robot → planner reacts → robot reaches goal → metrics
generated.

**The test should return a non-zero exit code if the result violates safety criteria.**

---

## 38. BENCHMARK CLI

Create `social-nav-benchmark`. Example:

```
social-nav-benchmark --scenario crossing_human --controller social --runs 20
social-nav-benchmark --scenario all --runs 20
```

Output:

```
results/
  run_001.json
  run_002.json
  summary.csv
  report.html
```

Generate plots: success rate, clearance distribution, trajectory length, goal time,
computation latency, TTC, oscillations.

---

## 39. REPRODUCIBILITY

Every experiment must save: git commit, ROS version, Ubuntu version, planner parameters,
scenario, random seed, timestamp, robot configuration, software version. Example
`experiment_metadata.json`:

```json
{ "git_commit": "...", "ros_distribution": "jazzy", "scenario": "crossing_human", "seed": 42, "planner_version": "0.1.0" }
```

---

## 40. DOCKER

Create a Docker image that contains the complete project. The user must be able to run
`docker build -t social-nav:jazzy .` then `docker run ...`. Prefer a Docker Compose setup
for simulation.

Document: GPU configuration if available, X11/display configuration, networking,
`ROS_DOMAIN_ID`, volumes, simulation.

**The project must also support native installation without Docker.**

---

## 41. ONE-COMMAND DEMO

Create `./scripts/run_demo.sh`. It should: build workspace if required, source ROS, start
Gazebo, start Nav2, start robot, start human simulator, start RViz, start SocialNav, send
a navigation goal, spawn humans, produce visible social navigation behavior.

The README should tell the user exactly what command to execute.

---

## 42. QUICK START

The README must contain a copy-paste workflow. Example:

```bash
git clone <repo>
cd social_nav_ws
./scripts/install_dependencies.sh
./scripts/build.sh
source install/setup.bash
./scripts/run_demo.sh
```

**Do not leave placeholder commands.** If a command differs depending on environment,
document both options.

---

## 43. ROS 2 COMMAND EXAMPLES

Document useful commands (and explain what each is useful for): `ros2 topic list`,
`ros2 topic echo /social_nav/humans`, `ros2 topic echo /cmd_vel`, `ros2 param list`,
`ros2 service list`, `ros2 node list`, `ros2 control list_controllers`,
`ros2 action list`, `ros2 bag record ...`.

---

## 44. PARAMETER TUNING TOOL

Create `social-nav-tune`. It should allow experiments with different weights. Example:

```
social-nav-tune --human-weight 5 --ttc-weight 20 --clearance 1.5
```

Run benchmark, compare results, generate `parameter_sweep.csv` and plots.

---

## 45. ABLATION STUDY

This is important for making the project research-grade. Run: Baseline Nav2, Social
clearance only, Human prediction, TTC, Group modeling, Full model. Compare performance:

```
Method            Success   TTC   Clearance
Baseline          ...
Social Cost       ...
Prediction        ...
TTC               ...
Group Model       ...
Full SocialNav    ...
```

**Do not manufacture results. Actually run experiments.**

---

## 46. RESEARCH QUESTIONS

The project should answer:

- **RQ1:** Does human trajectory prediction reduce near-collision events?
- **RQ2:** Does anisotropic social cost improve human clearance?
- **RQ3:** Does group modeling reduce uncomfortable group intrusion?
- **RQ4:** What is the computational cost of social reasoning?
- **RQ5:** How does performance degrade as human density increases?
- **RQ6:** What happens when human predictions become uncertain?
- **RQ7:** Does social navigation significantly increase time-to-goal?
- **RQ8:** What parameter combinations provide a reasonable safety/performance tradeoff?

---

## 47. EXPERIMENT MATRIX

Run experiments across:

- Human density: 0, 1, 5, 10, 20, 30 humans
- Prediction horizon: 0.5, 1.0, 2.0, 3.0, 5.0 sec
- Clearance: 0.8, 1.0, 1.2, 1.5, 2.0 m
- Robot speed: 0.3, 0.5, 0.8, 1.0 m/s

Measure the resulting behavior.

---

## 48. SAFETY PHILOSOPHY

Use this hierarchy:

- **LEVEL 1:** Physical collision avoidance
- **LEVEL 2:** Dynamic collision prediction
- **LEVEL 3:** Human comfort
- **LEVEL 4:** Path efficiency
- **LEVEL 5:** Goal efficiency

**Never allow a lower-priority objective to override a higher-priority safety constraint.**

---

## 49. NO FAKE AI

Do not add an LLM simply because it looks impressive. Do not claim "AI-powered social
navigation" unless an actual learned model is implemented and benchmarked.

The initial planner should be explainable and deterministic. Optional future module:
learned human trajectory predictor. Keep it modular.

---

## 50. OPTIONAL ADVANCED ML MODULE

After the classical planner works, implement LSTM / Transformer-based trajectory
prediction.

- Input: past human positions, past velocity, heading, nearby humans, robot state.
- Output: future trajectory distribution.

Compare: Constant Velocity vs Kalman vs Learned Predictor. Measure: ADE, FDE, collision
prediction accuracy, planning performance.

**Do not make the ML model mandatory for the core system.**

---

## 51. CODE QUALITY

Follow modern C++ practices. Use: RAII, smart pointers, const correctness, namespaces,
header/source separation, clear interfaces, minimal global state, thread safety, mutexes
where required, atomics where appropriate, lifecycle management.

Use clang-format, clang-tidy, and compiler warnings `-Wall -Wextra -Wpedantic`. Treat
important warnings as errors.

---

## 52. THREADING

Do not block the ROS executor unnecessarily. Separate: sensor callbacks, tracking,
prediction, planning, visualization, benchmarking where appropriate.

Use callback groups and executors correctly. Avoid unnecessary mutex contention. Document
thread ownership.

---

## 53. REAL-TIME CONSIDERATIONS

Do not allocate large amounts of memory inside the high-frequency planning loop.
Preallocate where practical. Avoid unnecessary copies.

Measure: allocation, planning latency, callback latency, queue delays, TF lookup time,
costmap access time. Document any remaining non-real-time behavior honestly.

---

## 54. SECURITY / ROBUSTNESS

Treat external sensor input as untrusted. Handle: NaN, Inf, invalid timestamps, negative
time deltas, impossible velocities, invalid frame IDs, empty arrays, duplicate IDs, future
timestamps, stale timestamps, out-of-range coordinates.

Reject invalid data safely. **Never let malformed human data crash the planner.**

---

## 55. DOCUMENTATION

Create `docs/`: `architecture.md`, `human_model.md`, `prediction.md`, `social_cost.md`,
`planner.md`, `safety.md`, `simulation.md`, `benchmarking.md`, `performance.md`,
`troubleshooting.md`, `parameter_reference.md`, `research_results.md`.

Create architecture diagrams using Mermaid.

---

## 56. TROUBLESHOOTING GUIDE

Document solutions for: Gazebo not launching, RViz not displaying TF, Nav2 plugin not
loading, pluginlib errors, DDS discovery problems, Docker networking, GPU unavailable,
missing packages, TF extrapolation errors, costmap not updating, planner timeout, no
cmd_vel, robot not moving, simulation time mismatch, stale human tracks.

---

## 57. CI/CD

GitHub Actions must run: dependency installation, build, clang-format check, clang-tidy,
unit tests, integration tests, package tests.

A separate workflow can run simulation tests. **Do not make CI depend on a physical robot.**

---

## 58. RELEASES

Create:

- v0.1.0 — Core planner
- v0.2.0 — Human tracking
- v0.3.0 — Prediction
- v0.4.0 — Social cost
- v0.5.0 — Simulation benchmark
- v1.0.0 — Stable research release

Maintain `CHANGELOG.md`.

---

## 59. FINAL DEMO

The final demonstration should show a robot navigating toward a goal while several humans
move through the environment. The robot: detects humans, predicts trajectories,
visualizes social zones, slows down, chooses an alternate path, avoids groups, avoids
head-on collisions, does not oscillate, reaches the goal.

At the same time RViz displays: human predictions, social cost fields, candidate
trajectories, selected trajectory, TTC, planner mode, robot velocity, goal.

---

## 60. FINAL VIDEO

Produce a 3–5 minute technical demonstration:

```
00:00–00:20  Problem.
00:20–00:50  Architecture.
00:50–01:30  Human prediction.
01:30–02:10  Social cost.
02:10–02:50  Planner behavior.
02:50–03:30  Benchmark.
03:30–04:00  Performance.
04:00–05:00  Failure/recovery scenarios.
```

**Do not use cinematic footage instead of technical evidence. Show actual RViz/Gazebo
behavior.**

---

## 61. FINAL README

The README must immediately communicate: What problem does this solve? Why normal local
planners are insufficient for social navigation? Architecture, Algorithms, ROS 2
integration, Installation, Docker, Quick start, Demo, Parameters, Benchmarking, Results,
Limitations, Future work, Research references, License.

---

## 62. IMPORTANT ENGINEERING RULE

Do not pretend that a feature works.

- If something is incomplete: mark it `TODO`.
- If a benchmark has not been run: say `NOT RUN`.
- If a result is simulated: label it `SIMULATED`.
- If a value is an assumption: label it `ASSUMPTION`.

**Never fabricate performance numbers.**

---

## 63. DEVELOPMENT PROCESS

Implement incrementally.

- **PHASE 1:** Create ROS 2 packages. Verify build.
- **PHASE 2:** Create basic Nav2 controller plugin. Verify robot moves.
- **PHASE 3:** Implement trajectory generation.
- **PHASE 4:** Implement collision checking.
- **PHASE 5:** Implement human state messages.
- **PHASE 6:** Implement human tracking.
- **PHASE 7:** Implement prediction.
- **PHASE 8:** Implement social cost.
- **PHASE 9:** Implement group reasoning.
- **PHASE 10:** Implement recovery.
- **PHASE 11:** Create simulation scenarios.
- **PHASE 12:** Create automated benchmarks.
- **PHASE 13:** Optimize performance.
- **PHASE 14:** Add CI.
- **PHASE 15:** Write documentation.
- **PHASE 16:** Run final experiments.

**Do not jump directly to advanced ML. The classical system must work first.**

---

## 64. DEFINITION OF DONE

The project is complete only when:

- [ ] ROS 2 builds successfully.
- [ ] Nav2 plugin loads successfully.
- [ ] Robot navigates without humans.
- [ ] Human tracking works.
- [ ] Human prediction works.
- [ ] Social cost works.
- [ ] Dynamic human collision checking works.
- [ ] Group detection works.
- [ ] Narrow corridors are handled.
- [ ] No-human case works.
- [ ] Human-loss case works.
- [ ] TF failure is handled.
- [ ] Planner timeout is handled.
- [ ] Emergency stop works.
- [ ] At least 50 unit tests exist.
- [ ] Integration tests pass.
- [ ] Simulation scenarios run automatically.
- [ ] Benchmarking CLI works.
- [ ] Baseline comparison exists.
- [ ] Performance metrics are collected.
- [ ] Docker build works.
- [ ] One-command demo works.
- [ ] CI works.
- [ ] Documentation is complete.
- [ ] Results are reproducible.
- [ ] No fabricated benchmark results exist.

---

## 65. FIRST TASK

Do not immediately dump the entire project into one enormous response.

First inspect the development environment. Determine: Ubuntu version, ROS 2 version,
Gazebo version, available GPU, NVIDIA driver, Docker, workspace location.

Then propose the exact package architecture. After that, implement the project
incrementally. At the end of every implementation phase: Build. Run tests. Report
failures. Fix failures. Verify again. Show the exact command used to run the next stage.

**Do not move forward while a foundational component is broken.**

The final output should be a complete, runnable, documented robotics software project,
not merely example code.
