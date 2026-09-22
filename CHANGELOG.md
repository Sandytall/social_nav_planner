# Changelog

All notable changes to the SocialNav Planner. Releases follow MASTER_PROMPT §58.

## [Unreleased]
### Added
- `social_nav_msgs`: HumanState/Array, PredictedHumanTrajectory/Array, PlannerMetrics.
- `social_nav_core`: anisotropic social cost (§10), TTC/closest-approach (§11), trajectory
  generation (§16), collision (§17-18), weighted scorer (§19), behavior modes (§20/§23).
  68 GoogleTest unit tests (§35).
- `social_nav_prediction`: constant-velocity, constant-acceleration, and uncertainty-aware
  human motion models (§8).
- `social_nav_human_model`: nearest-neighbour tracker (§7), group detection (§13),
  layered anisotropic social zones (§9).
- `social_nav_controller`: `nav2_core::Controller` plugin (pure-pursuit base + social
  perturbation), goal-approach deceleration, behavior modes, debug topics + `PlannerMetrics`.
- `social_nav_costs`: `SocialLayer` costmap plugin — routes the global planner around people (§15).
- `social_nav_sim` + `social_nav_description`: Gazebo Classic world + MiR-100 robot.
- `social_nav_bringup`: Nav2 params wiring our controller, `demo.launch.py`, scripts.
- `social_nav_tools`: simulated human publisher + RViz social-zone markers (§31).
- `social_nav_rviz`: RViz configuration.
- `social_nav_benchmarks`: scenario benchmark framework + CLI (§29/§38/§39).
- Docs: MASTER_PROMPT, TUNING, BENCHMARKING, architecture, troubleshooting, ISAAC_SIM_PLAN.
- Docker + docker-compose (§40); GitHub Actions build+test CI (§57).

### Verified
- Autonomous navigation to goal (MiR, ~18 s), including through the corridor gap.
- Social avoidance: keeps ~1.5 m clearance and routes AROUND a person on the path.
- Controller latency measured: p50 ≈ 0.08 ms, p95 ≈ 0.12 ms at 20 Hz, 0 deadline misses.

### Deferred / not done (honest, §62)
- Full benchmark sweep NOT RUN (framework ready; user runs it — see docs/BENCHMARKING.md).
- Isaac Sim photoreal track deferred (GPU below spec).
- Optional learned trajectory predictor (§50) not implemented.
- clang-format/clang-tidy lint + SPDX headers deferred (CI has build+test).

## Release plan (§58)
- v0.1.0 core planner · v0.2.0 tracking · v0.3.0 prediction · v0.4.0 social cost
- v0.5.0 simulation benchmark · v1.0.0 stable research release
