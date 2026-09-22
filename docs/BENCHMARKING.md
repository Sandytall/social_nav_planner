# Benchmarking Runbook (§29, §38, §39, §45)

The benchmark framework runs deterministic scenarios headless, records real metrics, and
writes JSON/CSV/report. **You run the sweep** (it's many minutes of sim); I built the tool.
Every number is measured from an actual run — nothing is fabricated (§62).

## Quick start

```bash
cd ~/Social_planner/social_nav_ws
source install/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null

# one scenario, a few runs (fast check):
ros2 run social_nav_benchmarks social-nav-benchmark --scenario blocker_on_path --runs 3

# the full research sweep (§29: >=20 trials/scenario) — this takes a while:
ros2 run social_nav_benchmarks social-nav-benchmark --scenario all --runs 20
```

Results land in `~/social_nav_results/<UTC-timestamp>/`:
- `summary.csv` — one row per run, all metrics
- `<scenario>_<run>.json` — raw per-run data
- `report.md` — per-scenario success / collisions / clearance / time / p95 latency
- `experiment_metadata.json` — git commit, ROS distro, params, seeds (§39 reproducibility)
- `logs/` — full launch log per run (for debugging any failure)

## Scenarios (§28)

`empty`, `stationary_side`, `blocker_on_path`, `crossing`, `head_on`, `same_direction`,
`group`, `corridor_pair`. See `social_nav_benchmarks/scenarios.py` for exact human
placements, goals, and seeds.

## Metrics recorded (§29)

success, status (SUCCEEDED/ABORTED/TIMEOUT), time-to-goal, path length, path efficiency,
avg velocity, **min/avg human distance**, collision (centre < 0.5 m of a person),
social-zone intrusion ratio, stops, oscillations, and controller **compute p50/p95/p99/max
+ deadline misses + actual Hz** (§30).

## Ablation study (§45) — baseline vs SocialNav, SocialLayer on/off

The controller/costmap config is chosen by the `SOCIAL_NAV_PARAMS` env var (an absolute
path to a params yaml). To compare configurations, copy `nav2_params.yaml`, edit, and run:

```bash
CFG=~/Social_planner/social_nav_ws/src/social_nav_bringup/config

# 1) Full SocialNav (default):
ros2 run social_nav_benchmarks social-nav-benchmark --scenario all --runs 20

# 2) SocialLayer OFF (no global routing around people) — set social_layer.enabled: false:
cp $CFG/nav2_params.yaml /tmp/nav2_no_social_layer.yaml
sed -i 's/^\(        enabled: \)true/\1false/' /tmp/nav2_no_social_layer.yaml   # under social_layer:
SOCIAL_NAV_PARAMS=/tmp/nav2_no_social_layer.yaml \
  ros2 run social_nav_benchmarks social-nav-benchmark --scenario all --runs 20

# 3) Human cost OFF (weights.human: 0, weights.ttc: 0) — social zones ignored:
#    edit /tmp/nav2_no_social_cost.yaml similarly, then run with SOCIAL_NAV_PARAMS=...
```

Compare the `summary.csv` files across configs to fill the ablation table (§45):
success rate, min clearance, social intrusion, time-to-goal. Expect the SocialLayer/human
terms to *increase* clearance and *reduce* intrusions, at some cost in path length/time —
that's the research question (§46 RQ2/RQ3).

## Notes

- **Known upstream flake (Nav2 Humble):** ~1 run in 5 may report `ABORTED` even though the
  robot reached the goal (a `bt_action_node` result race — see `IMPLEMENTATION_STATUS.md`).
  The metrics judge `success` by odometry arrival, not just the action status, so an
  odometry-reached run still counts as success; `status` is recorded separately so you can
  see the flake rate.
- Run at least 20 trials per scenario for the stochastic (moving-human) cases; seeds are
  recorded for reproducibility.
- If a run leaves Gazebo behind, `./scripts/stop.sh` clears it.
