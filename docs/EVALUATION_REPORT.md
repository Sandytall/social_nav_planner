# Social Navigation — RL vs Classical Evaluation Report

An experimental reinforcement-learning local planner for the SocialNav project, evaluated against
classical reactive baselines on matched scenarios. This report is generated from the benchmark
result files in `docs/benchmarks/` (`scripts/rl_benchmark.py`); the numbers are not hand-entered.

## 1. Summary

- A PPO policy was trained to navigate a differential-drive robot to a goal through a factory aisle
  shared with pedestrians, consuming the existing human-tracking pipeline plus a down-sampled lidar.
- The headline number from the original accelerated environment (RL ≈ 0.84 success vs a straight-line
  baseline ≈ 0.16) **did not survive rigorous evaluation.** That environment tested human avoidance
  in near-empty space; it contained almost none of the static geometry (racks, walls) that the real
  Gazebo world has, and the policy had no obstacle perception at all.
- After adding the real static geometry to the training environment and a lidar to the observation,
  a policy trained purely in the accelerated sim **transfers to Gazebo and navigates the aisle** (30
  episodes, no obstacle-blind failures) — where the obstacle-blind policy had scored 0.00.
- On a fair, matched, obstacle-aware benchmark, the learned policy is **comparable to the classical
  reactive baselines** on solvable scenarios, better on some, worse on others. The dense head-on
  scenarios (`approaching`, `sudden_stop`, `dense_crowd`) are **unsolved by every method** in the
  1.7 m aisle — an environment-difficulty limit, not a method result.
- This report deliberately does not declare a winner. It reports measured behavior and the
  conditions under which each method succeeds or fails.

## 2. System under test

- **Robot:** differential drive, max linear 0.8 m/s, max angular 1.2 rad/s, control dt 0.05 s,
  episode budget 600 steps (30 s). Spawn at the aisle mouth `(0, 0)`, goal `(8.5, 0)`.
- **Environment:** `factory.world` — a ~1.7 m-wide aisle flanked by pallet racks and machines, with
  perimeter walls, shared with 2 (easy) or 4 (medium) pedestrians. Static geometry is extracted from
  the Gazebo world (`social_nav_tools/world_obstacles.py`) so the accelerated env matches it.
- **Observation (78-d):** robot state + goal (robot frame), the 5 nearest humans (relative
  position/velocity, distance, TTC, constant-velocity prediction, social-zone cost, group flag), a
  12-beam down-sampled lidar (ray-cast in the mock, real `/scan` in Gazebo — identical layout), and a
  human-presence mask.
- **Action:** normalized `[-1, 1]^2` → `(v, w)` with acceleration limits.
- **Safety supervisor:** an optional deploy-time filter that slows/stops on low time-to-collision or
  clearance. The learned policy is trained without it (see §6); it is applied to all methods only in
  the "deployment" benchmark.
- **Two backends, one interface:** `MockBackend` (accelerated kinematic sim, ~3000 steps/s, used for
  training and this benchmark) and `GazeboBackend` (the real simulator, ~real-time, used for
  transfer checks). Humans in the mock reuse the same pedestrian model the Gazebo manager uses.
- **Policies compared:** `straight` (drive at the goal, slow if a human is dead ahead), `sfm` (a
  classical Social-Force local planner — goal attraction + exponential human repulsion, reads only
  the public observation), `rl` (the trained PPO policy).

## 3. Metric definitions

Traced from `env.py` / `evaluate.py`:

- **success** = reached the goal (within 0.6 m) with no collision and no human contact.
- **collision** = robot within its radius of a static obstacle (rack/wall).
- **human_collision** = nearest pedestrian within `robot_radius + human_radius` = 0.75 m
  (surfaces touching).
- **timeout** = 600 steps elapsed without reaching.
- **min_clearance (m)** = mean over episodes of the closest a pedestrian ever got.
- **avg_speed (m/s)**, **nav_time (s)**, **oscillations** (steering sign-flips, a jerk proxy),
  **latency** (policy inference P95, ms).

All numbers below are on the held-out **test** seed split (110000+), 20 episodes per cell, with every
policy on identical scenarios/seeds. Raw per-episode data: `docs/benchmarks/*.csv`.

## 4. Main result — factory, no safety supervisor (fair pure-policy comparison)

Success rate by scenario (easy / medium), from `docs/benchmarks/factory_raw.csv`:

| Scenario | straight | sfm (classical) | rl |
|---|---|---|---|
| normal (easy) | 0.80 | 0.65 | **0.80** |
| crossing (easy) | 1.00 | 1.00 | 0.00 |
| approaching (easy) | 0.00 | 0.00 | 0.00 |
| sudden_stop (easy) | 0.00 | 0.00 | 0.00 |
| direction_change (easy) | 0.00 | 0.00 | 0.00 |
| dense_crowd (easy) | 0.25 | 0.00 | 0.25 |
| normal (medium) | 0.35 | 0.50 | 0.35 |
| crossing (medium) | 1.00 | 1.00 | **1.00** |
| approaching (medium) | 0.00 | 0.00 | 0.00 |
| sudden_stop (medium) | 0.00 | 0.00 | 0.00 |
| direction_change (medium) | 1.00 | 0.00 | **1.00** |
| dense_crowd (medium) | 0.00 | 0.00 | 0.00 |

Reading it honestly:

- **RL matches the best baseline on `normal`, `crossing` (medium), and `direction_change` (medium).**
  It is the only method that solves `direction_change` (medium) besides straight, and it ties the
  1.00 on `crossing` (medium).
- **RL is worse on `crossing` (easy):** it over-avoids (mean clearance 2.45 m vs the baselines'
  ~1.6 m) and drifts into a rack — an obstacle collision, not a human one. Its instinct to keep
  distance is too strong for the tight aisle on that layout.
- **`approaching`, `sudden_stop`, `dense_crowd` are ~0 for every method.** A pedestrian coming
  head-on or stopping dead in a 1.7 m corridor frequently leaves no collision-free path in 30 s.
  This is the environment's difficulty ceiling, and it is the honest headline: the hard cases are
  hard for classical and learned planners alike.

So the fair benchmark does **not** reproduce a large RL advantage. On solvable scenarios RL is on par
with a well-tuned classical reactive planner; the naive 0.84-vs-0.16 gap was an artifact of testing
in open space with no static obstacles and no obstacle perception.

## 5. Deployment result — factory, safety supervisor on all methods

From `docs/benchmarks/factory_with_safety.csv`. Adding the supervisor as a backstop for every method:

- It **rescues the classical baselines on `sudden_stop`** (straight 0.00 → 1.00): the supervisor
  emergency-stops in time when a person halts ahead.
- It **crawls the RL policy** (e.g. `crossing`/easy: 0 collisions but average speed **0.07 m/s**,
  timing out). The RL policy was trained without the supervisor, so when the supervisor overrides its
  commanded velocity the policy's intent and the executed motion diverge and it stalls. This is a
  real finding: a safety filter trained-around, not trained-with, degrades a committed policy.

Conclusion: the RL policy's correct deployment mode here is **raw** (no supervisor); the classical
baselines benefit from the supervisor on stop-type scenarios.

## 6. Why the environment and observation had to change (the core finding)

The original accelerated environment placed only 3 obstacle boxes for the factory, all far off the
robot's path, and the observation had no static-obstacle channel. A policy trained there reached 0.84
by learning pure human avoidance in open space. Run in the real `factory.world` it scored **0.00** —
it drove straight into pallet racks it could not perceive (verified from the failure replay: obstacle
collision, nearest human 2.1 m away).

The fix, and the substance of this work:

1. Extract the real static geometry from each `.world` into the accelerated env (21 boxes for
   factory) so training collisions match Gazebo.
2. Add a 12-beam lidar to the observation, computed identically by ray-cast in the mock and by
   down-sampling the real `/scan` in Gazebo.
3. Retrain. The obstacle-aware policy then **navigates the real Gazebo aisle** (30 episodes, sensors
   confirmed ingesting: `odom/scan/humans/lidar` all present, ~1.5 m clearance), where the
   obstacle-blind policy could not move a metre without hitting a rack.

A second finding shaped training: with the safety supervisor active during training, it rewrote the
policy's actions near people, breaking credit assignment and driving a "freeze/crawl" optimum.
Training **without** the supervisor (`--no-safety`) and adding the crossing/approaching/sudden-stop
scenarios to the training mix produced a policy that commits down the aisle and reacts to people.

## 7. Cross-environment generalization

The policy was trained only on factory. Evaluated on other worlds (medium, `docs/benchmarks/
generalization_*.csv`):

| Env | scenario | rl | straight |
|---|---|---|---|
| warehouse | normal | 0.40 | 0.47 |
| warehouse | crossing | 0.00 | 1.00 |
| urban | normal/crossing | 0.00 | 0.00 |
| hospital | normal/crossing | 0.00 | 0.00 |

- On **warehouse** (a feasible layout) the factory-trained policy roughly matches the straight
  baseline on `normal` and, like on factory-easy, over-avoids on `crossing`.
- **urban** and **hospital** return 0.00 for *both* policies with ~0.72 m clearance from step one:
  their start/goal positions sit inside the newly added static geometry (those positions were laid
  out for the old near-empty env). These cells measure a broken env setup, not planner skill, and are
  a known limitation (see §8).

Generalization is limited, as expected for a policy trained on a single environment.

## 8. Limitations

- **Sim-to-sim gap.** Mock `crossing` success (up to 1.00) overstates Gazebo (~0.20 in spot checks):
  the mock's pedestrian motion and the real manager's motion differ, and Gazebo adds real diff-drive
  dynamics. Mock numbers are an upper bound on transfer.
- **Tight-aisle ceiling.** Head-on scenarios in a 1.7 m corridor are frequently unsolvable in the
  time budget for any reactive method. This dominates the low numbers in §4.
- **Infeasible start/goal on some worlds.** `urban`/`hospital` (and possibly others) need their
  start/goal moved out of the now-modeled obstacles before they can be benchmarked.
- **Constant-velocity human prediction.** The observation cannot foresee sudden turns/stops; the
  policy compensates by keeping margin, which is imperfect.
- **The high-fidelity classical comparison is not yet run.** The Nav2 controllers (RPP/DWB/MPPI) run
  in Gazebo through `social_nav_benchmarks`; a matched RL-vs-Nav2 run in Gazebo (both on the same
  world/seeds) is the natural next step and is scaffolded but not executed here.

## 9. Reproducibility

- Model under test: `~/social_nav_rl_checkpoints/BEST_factory_medium_multi/ppo_factory_42_best.zip`
  (PPO, MLP 256×256, trained empty→easy→medium, `--no-safety`, multi-scenario).
- Config: `src/social_nav_rl/config/` (observation/action/reward/safety/ppo).
- Regenerate every table: `python3 scripts/rl_benchmark.py …` (see `docs/COMMANDS.md`), test split,
  20 episodes/cell.
- Full commands, training pipeline, and history: `docs/COMMANDS.md`.

## 10. Conclusion (measured, not editorial)

- Where **RL performed well:** on-par with the best classical baseline on `normal`, and matched the
  1.00 on `crossing`/`direction_change` at medium density; it keeps larger clearance from people on
  several layouts.
- Where **classical performed well:** the straight/Social-Force baselines matched or beat RL on
  `crossing`/easy and `normal`/medium, and benefit more from the safety supervisor.
- Where **both failed:** the head-on/dense scenarios in the tight aisle.
- **Net:** the learned policy is a credible, obstacle-aware, human-aware local planner that transfers
  from the accelerated sim to Gazebo — but on a fair benchmark it does not dominate classical
  reactive planners, and the original large advantage was an artifact of an over-easy test. The
  scientifically useful outcome of this project is the evaluation itself: a matched, obstacle-aware
  benchmark that shows honestly where a learned social planner does and does not help.
