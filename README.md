# SocialNav — social navigation, classical **and** learned, rigorously evaluated

A robot that has to get somewhere through a space full of moving people. This repo has **two**
socially-aware local planners for a differential-drive robot — a **classical** Nav2 planner and a
**learned (PPO)** policy — and, most importantly, a **feasibility-aware, multi-environment benchmark**
that measures, with numbers, exactly where each one helps and where it doesn't.

Built on **ROS 2 Humble + Gazebo Classic 11** with a MiR-100 robot. Trained in a fast accelerated
simulator (~3,000–6,000 steps/s) that shares its pedestrian model and scenarios with Gazebo, and the
learned policy also runs **GPU-parallel in NVIDIA Isaac Sim / Isaac Lab**.

---

## TL;DR (the honest numbers)

- A first RL policy scored **0.84** success in the accelerated sim and **0.00** in the real
  simulator — it had no obstacle perception and had been tested in near-empty space. Fixing the
  *evaluation* (real obstacles, feasibility-aware metrics, unpredictable pedestrians, multi-map
  training) is the whole story below.
- On a fair, feasibility-aware benchmark (**5 environments, 20 episodes/cell, held-out seeds**), the
  final learned policy is **on par with classical reactive planners** — with two clear edges:
  - **factory `normal`: RL 0.86** vs Social-Force 0.71 vs straight-line 0.50 — RL is best.
  - **urban `sudden_stop`: RL 1.00 vs 0.00 / 0.00** — RL is the *only* method that stops and
    **waits** for a pedestrian to pass.
  - **social clearance:** RL keeps ~**2.0 m** from a head-on pedestrian vs ~**0.75 m** for the
    baselines.
- It does **not** win everywhere, and **no method** solves the hardest maps (office/hospital) or the
  genuinely-infeasible scenarios (`approaching`, `dense_crowd`). The point of the project is to show
  *which* is which, honestly.

---

## Why two planners?

### Why classical
A classical reactive planner is the right **baseline and default**:

- **No training, no data, no GPU** — it works the moment you launch it, and its behaviour is fully
  determined by a handful of interpretable parameters.
- **Predictable and certifiable** — you can read exactly why it did something (personal-space cost +
  time-to-collision + group intrusion), which matters for safety review.
- **Strong** — on the maps and scenarios where a clear path exists, it's hard to beat. In this
  benchmark the Social-Force baseline **ties or beats RL on `warehouse normal` (0.86 vs 0.71)** and
  **`urban normal` (0.65 vs 0.35)**, and both baselines hit **1.00 on `crossing`/`direction_change`**
  in most maps.

Here that's two Nav2 plugins:
- **`SocialNavController`** (`nav2_core::Controller`): pure-pursuit base command, then samples
  candidate motions, rejects collisions, and scores the rest by an **anisotropic personal-space
  cost + time-to-collision + group intrusion**. Behaviour modes (normal/cautious/crowded/emergency)
  scale speed as people get closer.
- **`SocialLayer`** (`nav2_costmap_2d::Layer`): stamps each person's personal-space cost into the
  global costmap so the global planner routes *around* people — kept below lethal so a passable gap
  never becomes blocked.

### Why RL
A learned policy can do things a hand-written reactive rule struggles with:

- **It learns behaviours you didn't code** — most strikingly, **waiting**: on `urban sudden_stop` the
  RL policy scores **1.00** where both classical baselines score **0.00**, because it learned to stop
  and yield when a pedestrian halts in its path. A reactive planner keeps trying to go.
- **It optimises the whole objective at once** — goal progress, obstacle clearance, and social
  distance jointly — instead of hand-tuning separate cost terms, and it ends up keeping **more social
  distance** (~2.0 m vs ~0.75 m on head-on approaches).
- **One policy generalises across maps** — trained across factory/warehouse/office/urban/hospital, it
  transfers to layouts it wasn't specialised on.

---

## Results (numbers)

Final generalist policy (PPO + frame-stack, trained across **5 maps × scenarios × difficulties**,
feasibility-filtered), reported as **success on *solvable* episodes** (`succ|solv`) — a feasibility
oracle separates *impossible scenario* from *policy failure*. Medium difficulty, 20 episodes/cell,
held-out test seeds (`scripts/rl_benchmark.py`).

### `normal` — goal-reaching through a shared corridor

| Environment | Straight-line | Social-Force | **RL** | best |
|---|:--:|:--:|:--:|:--:|
| factory   | 0.50 | 0.71 | **0.86** | RL |
| warehouse | 0.50 | 0.86 | 0.71 | SFM |
| urban     | 0.53 | 0.65 | 0.35 | SFM |
| office    | 0.00 | 0.00 | 0.00 | — (all fail) |
| hospital  | 0.00 | 0.00 | 0.00 | — (all fail) |

### Notable per-scenario (`succ|solv`)

| Scenario | Straight | Social-Force | **RL** | takeaway |
|---|:--:|:--:|:--:|---|
| urban `sudden_stop`            | 0.00 | 0.00 | **1.00** | **only RL waits/yields** |
| factory `direction_change`     | 1.00 | 0.00 | **1.00** | RL matches straight, beats SFM |
| warehouse `crossing`           | 1.00 | 1.00 | **1.00** | RL ties classical |
| warehouse `direction_change`   | 1.00 | 1.00 | **1.00** | all solve it |
| factory `crossing`             | 1.00 | 1.00 | 0.00 | RL inconsistent map-to-map |
| office/urban/hospital `direction_change` | 1.00 | 1.00 | 0.00 | RL over-avoids on narrow maps |

### Social clearance (mean closest a pedestrian ever got, m — higher = more polite)

| Scenario | Straight | Social-Force | **RL** |
|---|:--:|:--:|:--:|
| factory `approaching` (head-on) | 0.75 | 0.75 | **2.06** |
| warehouse `approaching`         | 0.75 | 0.75 | **1.93** |
| office `normal`                 | 0.74 | 0.73 | **1.06** |

### Feasibility (what fraction of episodes are even solvable, medium)

| Scenario | factory | warehouse | office | urban | hospital |
|---|:--:|:--:|:--:|:--:|:--:|
| normal            | 0.70 | 0.70 | 0.80 | 0.85 | 0.80 |
| crossing / sudden_stop / direction_change | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| approaching / dense_crowd | 0.00 | 0.00 | 0.00 | 0.00–0.10 | 0.00 |

`approaching` (head-on in a ~1.7 m aisle) and `dense_crowd` are **~0 % solvable for any planner** —
so a 0.00 there is a *map limit*, not a policy failure. Reporting `succ|solv` is what keeps the
comparison fair.

### Isaac Sim port (GPU-parallel, laptop RTX 4050)
Same MiR robot imported into Isaac Sim 4.5 / Isaac Lab, social-nav task trained with rsl_rl PPO:
**64 parallel envs at ~3,400 steps/s**, **reward 79.5, goal error 0.27 m, zero pedestrian
collisions** over **1.54 M steps in ~13 min**.

**Bottom line:** a *credible generalist, not a dominant one*. It matches or beats classical on the
maps it was built for, does something classical can't (**yield/wait**), keeps **more social
distance**, but doesn't win across the board — and office/hospital defeat everyone.

---

## How we did it

The interesting part wasn't the algorithm — it was the **evaluation and the training distribution**.
Each step below was driven by a number that turned out to be wrong.

1. **Honest benchmarking.** The first policy's 0.84 was an artifact of an obstacle-free test
   environment; in Gazebo it drove straight into pallet racks (0.00). **Fix:** real static geometry +
   a 12-beam lidar in the observation, computed identically in the accelerated sim and from Gazebo's
   real `/scan`.
2. **Don't let a safety filter rewrite actions during RL.** A supervisor mutating the policy's output
   broke credit assignment and the robot learned to crawl. **Fix:** train the raw policy; keep the
   supervisor as a deploy-time backstop only.
3. **Feasibility-aware metrics.** Many "failures" were impossible episodes. A feasibility oracle
   (`social_nav_rl/feasibility.py`) checks whether *any* wait-then-go path solves a scenario, so the
   benchmark reports **success on solvable episodes** instead of punishing the policy for physics.
4. **Fix the training distribution, not just the reward.** Multi-scenario, multi-map, multi-difficulty
   training with **unpredictable (non-linear) pedestrian motion** and **domain randomization** so the
   policy reacts to real motion instead of extrapolating a straight line.
5. **You get the policy your eval selects for.** A single-scenario eval quietly bred a
   factory-specialist — `direction_change`/`sudden_stop` got *worse* the longer it trained. **Fix:**
   a multi-scenario generalist eval, so "best" is chosen across the whole distribution.
6. **Align reward with the behaviour you want.** A "don't dawdle" penalty forbade *waiting*; making it
   conditional on a nearby pedestrian is what unlocked the `sudden_stop` win.

The accelerated sim (kinematic, ~3,000–6,000 steps/s, real pedestrian behaviour) does the training;
Gazebo and Isaac Sim are the high-fidelity transfer checks.

---

## Run it

### Classical planner (Gazebo + RViz)
```bash
colcon build --symlink-install && source install/setup.bash
# drive to a goal while avoiding a person on the route:
HUMANS="-1.3,-0.7,0,0" ./scripts/run_demo.sh          # GUI=false runs headless (RViz only)
```

### Learned policy — train / evaluate / benchmark
```bash
# train PPO in the accelerated sim (parallel envs; keeps the BEST checkpoint, early-stops on plateau)
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --environment factory \
  --scenarios normal,crossing,sudden_stop,direction_change --difficulty medium \
  --no-safety --net-arch 256,256 --frame-stack 4 --n-envs 8 --timesteps 5000000

# evaluate a checkpoint on held-out test seeds
ros2 run social_nav_rl social-nav-rl-eval --policy rl --no-safety \
  --model ~/social_nav_rl_checkpoints/<run>/ppo_<env>_<seed>_best.zip \
  --environment factory --scenario crossing --difficulty medium --split test --episodes 20

# benchmark RL vs classical (feasibility-aware succ|solv) across all 5 environments
for E in factory warehouse office urban hospital; do
  CUDA_VISIBLE_DEVICES= python3 scripts/rl_benchmark.py --model <best.zip> \
    --environment $E --difficulties medium --episodes 20
done
```
Also supported: `--curriculum`, `--init-model` (warm-start), `--domain-rand`, `--target-success`,
`--patience`. Transfer to Gazebo with `--backend gazebo` (launch `rl_sim.launch.py` first; RViz Fixed
Frame = `odom`).

---

## Repo layout

| Package | Role |
|---|---|
| `social_nav_core` | classical planner math (no ROS, GoogleTest) |
| `social_nav_controller`, `social_nav_costs` | Nav2 controller + social costmap layer |
| `social_nav_rl` | the learned policy — obs/reward/safety/action, Gym env, PPO training/eval, feasibility oracle, domain randomization, benchmark integration |
| `social_nav_benchmarks` | scenario benchmark + reports |
| `social_nav_sim`, `social_nav_description`, `social_nav_bringup` | Gazebo worlds, MiR robot, launch |
| `social_nav_tools` | simulated pedestrians, RViz markers, shared pedestrian model |
| `social_nav_msgs` | human / metrics messages |

Requirements: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, Nav2. RL extras:
`pip install 'stable-baselines3>=2.2' 'sb3-contrib>=2.2' gymnasium`.

More detail: [docs/architecture.md](docs/architecture.md) (classical planner),
[docs/TUNING.md](docs/TUNING.md), [docs/BENCHMARKING.md](docs/BENCHMARKING.md).

---

## Attribution & license

The MiR-100 visual meshes in `social_nav_description/meshes/mir/` are third-party (see
[docs/ATTRIBUTION.md](docs/ATTRIBUTION.md)) — used for visualization only; verify the upstream mesh
license if you redistribute. Everything else is original to SocialNav and licensed **Apache-2.0**
(see `LICENSE`). Architectural ideas were informed by public Nav2 / social-navigation work
(ROSNavBench, Social-Force planners); no code was copied.
