# Future Work — R&D Evaluation / Telemetry / Benchmarking System

**Status: PLANNED (not implemented).** Captured from a requirements spec (2026-09-22).
Goal: elevate the project from a demo to a serious R&D validation / deployment tool —
experiment runner, reproducible configs, rich metrics, event + trajectory logging, rosbag
offline analysis, failure classification, multi-planner benchmarking, statistics, scenario
matrix, and an HTML dashboard/report.

## Principle (spec §16): REUSE, don't rewrite

The planner, perception, prediction, sim, and Nav2 integration are existing functionality —
do not restructure them. Build the eval system as an additive layer that consumes ROS 2
topics. Much of this **already exists** in `social_nav_benchmarks`; the "later" work is a
delta, not a new project.

## Already have (reuse)
- Experiment runner + CLI (`social-nav-benchmark`), per-run dir, summary.csv, report.md.
- Reproducibility: `experiment_metadata.json` (git commit, ROS distro, seeds, params path).
- Metrics: success, time-to-goal, path length/efficiency, min/avg human clearance,
  collision, social-intrusion ratio, stops, oscillations, planner p50/p95/p99 + deadline
  misses + actual Hz (§30 `PlannerMetrics`).
- Failure/status states (`/social_nav/debug/status`: EMPTY_PLAN / NO_VALID_TRAJECTORY /
  STALE_HUMAN_DATA / EMERGENCY_STOP).
- Deterministic scenarios (`scenarios.py`), ablation via `SOCIAL_NAV_PARAMS`.
- Debug topics: selected trajectory, planner mode, RViz markers.
- Docker + CI, reproducible.

## Delta to build later (the new pieces)
1. **Config-driven experiment/benchmark scripts** — `scripts/run_experiment.sh` /
   `run_benchmark.sh` with `--planners rpp,social_force,social_nav`, `--pedestrians 1,3,5`,
   `--seeds 1-20` sweeps; write `results/<id>/config.yaml` (full config) per run.
2. **rosbag2 recording + OFFLINE analyzer** — record a bag per run; analyze after the sim
   ends (bag → metrics/events/report), so real-robot rosbags can be analyzed the same way.
   This is the key architectural addition (decouples analysis from Gazebo).
3. **Structured event logger** — HUMAN_DETECTED, CROSSING_PREDICTED, PASSING_STARTED/…,
   NEAR_COLLISION, RECOVERY_STARTED → `events.json` (timestamp/type/severity/robot_state/
   human_ids/metadata). Emit from the controller/human_model where the state is known.
3b. **Trajectory logging** to `trajectory.csv` (robot pose/vel, global path, local traj,
   pedestrian pos/vel + predictions) — ROS-level, sim-agnostic.
4. **Failure classifier** — COLLISION/STUCK/OSCILLATION/DEADLOCK/PLANNER_TIMEOUT/
   TF_FAILURE/RECOVERY_LOOP/GOAL_TIMEOUT with evidence window; never claim a cause the data
   doesn't support (§62).
5. **Baseline planners for comparison** — RPP (`nav2_regulated_pure_pursuit_controller`,
   installed), DWB/MPPI where available, and a **Social Force** baseline; same scenario/
   seed/goal for fair comparison (extend the `SOCIAL_NAV_PARAMS` ablation mechanism to a
   `--planner` that selects a params file per controller).
6. **Statistics** — count/mean/median/std/min/max/percentiles/success-rate per dimension;
   keep safety / efficiency / social / system as SEPARATE dimensions (no single score).
7. **Scenario matrix** — parametrized categories (EMPTY, DOORWAY, INTERSECTION,
   CROWDED_CORRIDOR, SUDDEN_CROSSING, …) with pedestrian_count/speed, obstacle_density,
   sensor_noise, prediction_horizon, social_cost_weight; sweepable.
8. **System telemetry** — CPU %, max RAM, GPU (when available), sim FPS, real-time factor;
   mark unavailable metrics as invalid (never fabricate).
9. **HTML dashboard/report** — `scripts/generate_report.py --input results/<benchmark>`:
   overview tiles, trajectory plots, time-series (human distance / velocity / latency /
   costs), event timeline, planner comparison, failure breakdown. Interview/GitHub ready.
10. **Artifact layout** — `results/benchmark_<date>/{benchmark_config.yaml, summary.json,
    comparison.csv, runs/run_XXXX/{config,metrics,events,trajectory,metadata,report}, report.html}`.
    Do NOT commit generated data (add to `.gitignore`).

## Architectural references (ideas, not code): ROSNavBench, NavigationAnalyzer, Nav2
planner benchmarking, Social Force Window Planner. Preserve licenses/attribution if reused.

## Suggested build order (when picked up)
event logger + trajectory.csv → rosbag record + offline analyzer → failure classifier →
baseline planners (RPP first) → statistics → HTML report → system telemetry → scenario
matrix params. Each is additive; the existing `social_nav_benchmarks` is the seed.
