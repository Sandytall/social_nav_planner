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

## 1.1 Final cross-environment result (generalist policy)

The evaluation was later extended from the single factory map (§4+) to a **generalist** policy
(PPO + frame-stack) trained across all five maps × scenarios × difficulties — feasibility-filtered,
with unpredictable pedestrians and a multi-scenario eval so "best" is selected as a generalist, not a
single-map specialist. Success on **solvable** episodes (`succ|solv`), medium difficulty, 20
episodes/cell, held-out test seeds (`scripts/rl_benchmark.py`):

**`normal` — goal-reaching through a shared corridor:**

| Environment | Straight | Social-Force | RL |
|---|:--:|:--:|:--:|
| factory   | 0.50 | 0.71 | **0.86** |
| warehouse | 0.50 | **0.86** | 0.71 |
| urban     | 0.53 | **0.65** | 0.35 |
| office    | 0.00 | 0.00 | 0.00 |
| hospital  | 0.00 | 0.00 | 0.00 |

**Notable per-scenario:**

| Scenario | Straight | Social-Force | RL | Note |
|---|:--:|:--:|:--:|---|
| factory `direction_change`   | 1.00 | 0.00 | **1.00** | RL matches straight |
| warehouse `crossing`         | 1.00 | 1.00 | **1.00** | RL ties classical |
| urban `sudden_stop`          | 0.00 | 0.00 | **1.00** | **only RL waits/yields** |
| office/urban/hospital `direction_change` | 1.00 | 1.00 | 0.00 | RL over-avoids on narrow maps |
| factory `crossing`           | 1.00 | 1.00 | 0.00 | RL inconsistent map-to-map |

- **RL's standout:** urban `sudden_stop` = 1.00 while both baselines are 0.00 — the only method that
  stops and waits for a pedestrian to pass (payoff of the waiting-aware reward).
- **RL keeps more social distance** — larger clearance to people almost everywhere (e.g. ~2.0 m on
  head-on approaches vs ~0.75 m for the baselines).
- **office & hospital:** every method (including a straight line) scores ~0 on `normal`/`crossing` —
  narrow, furniture-filled corridors are hard for classical *and* learned planners alike.
- `approaching` and `dense_crowd` are ~0% feasible everywhere (excluded as unsolvable for all).

**Verdict:** a credible generalist, not a dominant one — matches or beats classical on the maps it was
built for, does something classical can't (yield/wait), keeps better social distance, but does not win
across the board, and the hardest maps defeat everyone. The sections below are the deeper factory
single-map analysis that preceded the generalist.

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

## 4.1 Feasibility-aware view (separating "impossible" from "policy failure")

Raw success conflates two very different things: scenarios no planner could solve in the time budget,
and scenarios that are solvable but the policy botches. A feasibility oracle
(`social_nav_rl/feasibility.py`) checks, per seed, whether *any* simple strategy — pick an aisle lane
(centre / offset) and a start delay (wait for a crosser to clear), then drive to the goal — is
collision-free against the real static geometry and the deterministic pedestrian trajectories. It's a
conservative lower bound (a cleverer path might exist), reported as `feasible_frac`. Then:

- `success | solvable` = success rate over seeds a path exists for.
- `safe_on_infeasible` = fraction of *impossible* seeds where the policy at least stays collision-free
  (i.e. correctly waits/yields instead of crashing).

Key findings (from `docs/benchmarks/factory_feasibility.csv`):

- **`approaching` (head-on) is 0% feasible, and `dense_crowd` (medium) too.** A pedestrian coming
  head-on in a 1.7 m aisle genuinely has no collision-free path in 30 s. Their 0% success is the
  **environment**, not any planner — they should be excluded from planner-quality claims.
- **`sudden_stop` is 100% feasible but 0% success for *every* method.** The oracle solves it by
  *waiting* for the stopped person; none of straight, Social-Force, or RL wait. This is a shared
  **policy gap** (no yielding/waiting behavior), not an environment limit — and the clearest target
  for future work.
- **No method fails gracefully:** `safe_on_infeasible = 0.0` across the board. On impossible
  head-on cells the classical planners drive into the *person*; the RL policy over-avoids the person
  (keeps ~4 m clearance) and drives into a *rack* instead. A policy that simply stopped would score
  1.0 here.
- On `success | solvable`, RL and the classical baselines are again close: e.g. `normal`/easy 0.94 for
  both RL and straight; RL's one clear deficit is `crossing`/easy (0.00 — it over-avoids into a rack),
  which is a joint human-plus-static-geometry reasoning failure.

The practical conclusion: the honest comparison is on the *solvable, interactive* subset, and the
two most valuable next steps both fall out of this analysis — a yielding/WAIT behavior (for
`sudden_stop` and graceful failure) and a better joint human+geometry representation (for
`crossing`/easy).

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

## 8.1 Closing the transfer gap (domain randomization + system-ID) — built, not yet validated

The §8 sim-to-sim gap (mock `crossing` up to 1.00 vs Gazebo ~0.20) is the single biggest threat to
the result. Three pieces of transfer work are now in the codebase; **all require a retrain on the
user's compute before their effect can be reported, so no improved number is claimed here.**

- **Cheaper, richer observation (Item 2).** Beyond the 12-beam lidar, the observation now carries
  derived corridor geometry (front/left/right clearances, a lateral centre-offset "which way is more
  open") and a crossing time-to-conflict feature — quantities an MLP struggles to read from raw beams.
  These target the two recurring §4.1 failures (person one side + rack the other; crossing timing).
- **Domain randomization (Item 3).** Training can now randomize, per episode, the robot's control
  latency + velocity-tracking noise + speed, the lidar's noise/dropout, and the crowd's speed
  (`--domain-rand`). A policy trained across this distribution should survive the real diff-drive
  dynamics and noisy `/scan` that the deterministic mock lacks — the mechanism behind the 1.00→0.20
  drop.
- **System-ID (Item 3).** `scripts/sysid.py` drives an identical open-loop command program through
  both backends and fits the actual gap (velocity/yaw gain, tracking noise, control latency, lidar
  noise/dropout), printing DR ranges to use instead of guessing them. On the mock it correctly fits
  ~zero deviation (rig sanity check).

A correctness fix landed alongside: under multi-scenario/curriculum training the mock backend was
rebuilt each episode without its observation config, so the lidar silently went blind after the
first episode — meaning some earlier "obstacle-aware" runs were effectively obstacle-blind. Fixed;
this alone may change the retrained numbers.

**Honest status:** these are the right levers for transfer and are unit-tested, but the delivered
model predates them (78-d obs, no DR). The claim they improve transfer is a hypothesis until the
86-d + `--domain-rand` retrain and a fresh Gazebo benchmark are run.

## 9. Reproducibility

- Model under test: `~/social_nav_rl_checkpoints/BEST_factory_medium_multi/ppo_factory_42_best.zip`
  (PPO, MLP 256×256, trained empty→easy→medium, `--no-safety`, multi-scenario).
- Config: `src/social_nav_rl/config/` (observation/action/reward/safety/ppo).
- Regenerate every table: `python3 scripts/rl_benchmark.py …` (see the README's *Run it* section),
  test split, 20 episodes/cell.
- Full training pipeline, commands, and the build story: [`JOURNAL.md`](JOURNAL.md) and the
  [README](../README.md).

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
