# Contributing

## Build & test
```bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select social_nav_core social_nav_prediction \
  social_nav_human_model social_nav_controller
colcon test-result --verbose
```

## Ground rules (from MASTER_PROMPT)
- **§62 honesty:** never fabricate results. Mark incomplete work `TODO`, unrun benchmarks
  `NOT RUN`, simulated results `SIMULATED`. Every number in docs/reports must be measured.
- **§35 tests:** add meaningful GoogleTest unit tests for new core logic — not getter/setter
  tests. Keep `social_nav_core` ROS-free so it stays fast to test.
- **§51 C++:** modern C++17, RAII, const-correctness, `-Wall -Wextra -Wpedantic`.
- Keep the planner **simulator-agnostic** (consume `/scan /odom /social_nav/humans`, emit
  `/cmd_vel`); do not couple it to a specific detector or simulator.
- Update `IMPLEMENTATION_STATUS.md` and `CHANGELOG.md` with what you actually verified.

## Layout
Core math in `social_nav_core`; ROS wiring in the node/plugin packages; tuning in
`social_nav_bringup/config/nav2_params.yaml` (see `docs/TUNING.md`).
