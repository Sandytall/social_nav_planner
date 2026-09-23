# SocialNav — Classical + Learned Social Navigation

**A research-grade ROS 2 social-navigation stack for differential-drive robots: classical Nav2
planning, a PPO-based learned policy, classical human-trajectory prediction, a deployment-time safety
supervisor, and a reproducible, feasibility-aware, multi-environment benchmark.**

> **The goal is not to prove that RL beats classical planning.** It is to measure *where* learned
> navigation helps, *where* classical planning is stronger, *why* either method fails, and *how well*
> a learned policy generalizes across environments.

Built on **ROS 2 Humble + Gazebo Classic 11** with a MiR-100 robot. Trained in a fast accelerated
kinematic simulator (~3,000–6,000 steps/s) that shares its pedestrian model and scenarios with
Gazebo; the learned policy also trains **GPU-parallel in NVIDIA Isaac Sim / Isaac Lab**.

Detailed evaluation: **[docs/EVALUATION_REPORT.md](docs/EVALUATION_REPORT.md)** · full build story and
lessons: **[docs/JOURNAL.md](docs/JOURNAL.md)**.

---

## What this project does

How should a mobile robot move through a space shared with moving people, balancing goal progress,
safety, clearance, and socially appropriate behaviour? SocialNav answers that with **two** local
planners behind one interface, plus the benchmark that compares them honestly.

- **Classical SocialNav** — a Nav2 controller + costmap layer: trajectory sampling, collision
  rejection, anisotropic personal-space cost, time-to-collision, group-aware cost, adaptive behaviour
  modes. No training required; fully interpretable.
- **Learned SocialNav** — a PPO policy that outputs `[linear_velocity, angular_velocity]` from robot
  state, goal, tracked humans + their (classically) predicted trajectories, and a down-sampled lidar.
  Optionally protected at deploy time by a safety supervisor.

---

## Why this project exists

The interesting part is not training PPO. The first experiment *looked* successful:

```text
Accelerated simulator:   PPO success ≈ 0.84
```

That result did **not** survive realistic evaluation. Once real static geometry (racks, walls) was
added, the policy would drive **straight into pallet racks** — in Gazebo it scored **0.00**. It had no
obstacle perception and had been tested in near-empty space. The lesson:

> **A navigation policy can score highly in an easy environment while being incapable of solving the
> actual navigation problem.**

So the project became a **social-navigation evaluation framework**:

```text
             Social Navigation
                    │
        ┌───────────┴───────────┐
     Classical                 PPO
      Nav2                    Policy
        └───────────┬───────────┘
             Safety / Feasibility
                    │
                  Robot
                    │
   Factory · Warehouse · Urban · Office · Hospital · Plaza
```

---

## The RL failure that taught the most

Adding static geometry + a lidar initially made PPO **stop learning**. It converged to:

```text
commanded velocity ≈ 0.1–0.2 m/s   (a crawl)
episode length     = timeout
success            = 0
```

The decisive diagnostic was an **empty aisle** (no humans): a hard-coded straight-line controller
reached the goal (**1.00**), while the RL policy still crawled and timed out. So the failure was **not
primarily human avoidance** — something in the RL/environment interface was broken.

The cause: during training, a **safety supervisor was rewriting the policy's chosen velocity** before
it reached the environment — a credit-assignment bug:

```text
PPO chooses action → Safety supervisor modifies it → Environment executes the modified action
                                                     → Reward is attributed to PPO's ORIGINAL action
```

PPO learned a conservative low-speed policy. The fix was to separate the two roles:

```text
Training:     PPO → raw action → environment
Deployment:   PPO → safety supervisor → robot
```

The safety layer still matters — but as a **deployment backstop**, not an opaque action-mutating
component inside the learning loop. This separation is now part of the architecture.

---

## Architecture

```text
                 Sensors
        ┌───────────┴───────────┐
     LiDAR (/scan)        Human tracking (/social_nav/humans)
        │                        │
        │                Trajectory prediction (classical, constant-velocity + uncertainty)
        │                        │
        └───────────┬────────────┘
              Shared human-state + obstacle representation
        ┌───────────┴───────────┐
   Classical planner          PPO policy
        └──────── candidate cmd_vel ────────┘
                    │
             Safety supervisor  (deploy only)
                    │
                  cmd_vel → Robot
```

Prediction is deliberately separated from the planners so the classical and learned policies consume
the **same** human-state representation — a fair comparison.

---

## Classical SocialNav

- **`SocialNavController`** (`nav2_core::Controller`): generates candidate local motions, rejects
  colliding ones, and scores the rest by human proximity, time-to-collision, and group intrusion;
  behaviour modes (normal / cautious / crowded / emergency) adapt speed to local human density.
- **`SocialLayer`** (`nav2_costmap_2d::Layer`): stamps each person's personal-space cost into the
  global costmap, kept **below lethal** so a genuinely passable gap never becomes an artificial
  obstacle.

An interpretable baseline with no training requirement — and a strong one (see Results).

---

## Learned SocialNav

PPO (Stable-Baselines3), continuous action `[v, w]`. Observation (fixed-size via nearest-N relevance
filtering + masking, so a variable number of humans always yields the same vector):

- **Robot:** linear/angular velocity, relative goal position, goal distance and bearing.
- **Humans (nearest N):** relative position and velocity, heading, distance, TTC, classical predicted
  trajectory + prediction uncertainty, social-zone cost, group flag; plus a derived crossing
  time-to-conflict.
- **Environment:** a 12-beam down-sampled lidar — ray-cast in the accelerated sim, from the real
  `/scan` in Gazebo, with an identical beam layout so an obstacle-aware policy sees the same input in
  both.

### Reward (per-component, independently configurable for ablations)

```text
goal_progress · goal_completion · collision · human_collision · ttc · clearance
social_zone · path_efficiency · time · smoothness · angular_smoothness · stopping · oscillation
```

Terms are weighted separately and logged individually, so any one can be ablated without code changes.

### Safety supervisor (deployment)

Raw policy velocity → supervisor (brake / slow on low TTC or clearance) → robot. Every intervention is
recorded (policy action, safety decision, executed command, reason, duration) so interventions are
**measurable, not invisible**.

---

## Simulation & transfer

```text
Fast accelerated sim  →  large-scale RL training  →  held-out evaluation  →  Gazebo validation  →  Isaac Sim
```

The accelerated simulator is deliberately **not** treated as proof of real-world performance — it is
where training happens (kinematic, thousands of steps/s, real pedestrian behaviour); Gazebo and Isaac
Sim are the high-fidelity transfer checks.

**Isaac Sim port (laptop RTX 4050):** the same MiR robot imported (URDF → USD) into Isaac Sim 4.5 /
Isaac Lab, social-nav task trained with rsl_rl PPO — **64 parallel envs at ~3,400 steps/s, reward
79.5, goal error 0.27 m, zero pedestrian collisions** over 1.54 M steps in ~13 min.

---

## Environment suite

Five industrial/human environments plus an open plaza — each a corridor/aisle shared with pedestrians,
differing in width, clutter, and crossing geometry (definitions in
`social_nav_tools/environments.py`):

| Environment | Character | Corridor |
|---|---|---|
| **factory**   | pallet racks + machines flanking an aisle, AMR routes | ~3.2 m |
| **warehouse** | pallet-rack rows around a wide central cross-aisle, forklifts/carts | ~6.4 m |
| **urban**     | building blocks beside a walkway, wide crossing | ~2.4 m |
| **office**    | corridor with a reception desk at the edge | ~2.8 m |
| **hospital**  | corridor with intruding furniture (nurse desk, reception, bench) | ~2.8 m |
| **plaza**     | open space, sparse obstacles (buildings, trees, benches), multiple routes | open |

### Scenario types

```text
empty · normal · approaching · crossing · same_direction · sudden_stop · direction_change
group · bottleneck · occlusion · goal_blocked · dense_crowd · dynamic_obstacle · multi_robot
```

Not every scenario is solvable in every environment — and that matters (below).

---

## Feasibility-aware evaluation

A central design choice is distinguishing **policy failure** from an **impossible scenario**. A ~1.7 m
aisle with two pedestrians approaching head-on may have *no* physically feasible path given the robot's
clearance — calling that "RL failed" is misleading.

A feasibility oracle (`social_nav_rl/feasibility.py`) checks whether *any* wait-then-go path solves an
episode, so the benchmark reports:

```text
success | solvable        (success on solvable episodes)
```

instead of counting impossible episodes as navigation failures.

| Scenario | factory | warehouse | office | urban | hospital |
|---|:--:|:--:|:--:|:--:|:--:|
| normal | 0.70 | 0.70 | 0.80 | 0.85 | 0.80 |
| crossing / sudden_stop / direction_change | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| approaching / dense_crowd | 0.00 | 0.00 | 0.00 | 0.00–0.10 | 0.00 |

`approaching` and `dense_crowd` are ~0 % solvable for *any* planner.

---

## Benchmark

Every planner runs under **identical conditions** — same environment, robot, start, goal, humans,
human trajectories, seed, and sensor configuration. Methods: **straight-line baseline**,
**classical Social-Force**, **PPO policy** (evaluated raw; the safety supervisor can be layered on at
deploy time).

Metrics recorded by `scripts/rl_benchmark.py` / `evaluate.py`:

- **Navigation:** success, `succ|solv`, path length, navigation time, goal error, timeout.
- **Safety:** collision, human collision, minimum clearance, minimum TTC, safety interventions.
- **Social:** social-zone violations, group intrusion, unnecessary stopping, excessive detour.
- **Motion:** oscillations (steering sign-flips), acceleration/angular smoothness.
- **System:** inference latency (P95).

Failed episodes are saved as replays (start/goal, per-step pose, action, safety decision, events) so
an individual failure can be re-played rather than reduced to a single percentage.

---

## Results (feasibility-aware `succ|solv`, medium, 20 episodes/cell, held-out seeds)

Reported **without** claiming a universal winner.

### `normal` — goal-reaching through a shared corridor

| Environment | Straight-line | Classical (SFM) | **RL** | best |
|---|:--:|:--:|:--:|:--:|
| factory   | 0.50 | 0.71 | **0.86** | RL |
| warehouse | 0.50 | 0.86 | 0.71 | classical |
| urban     | 0.53 | 0.65 | 0.35 | classical |
| office    | 0.00 | 0.00 | 0.00 | — (all fail) |
| hospital  | 0.00 | 0.00 | 0.00 | — (all fail) |

### What RL learned — waiting

The most interesting learned behaviour is **not** obstacle avoidance. On `urban/sudden_stop`:

```text
pedestrian moves → pedestrian stops → robot recognizes a blocked path → robot stops → waits
                → pedestrian clears → robot continues
```

| `urban/sudden_stop` | Straight | Classical | **RL** |
|---|:--:|:--:|:--:|
| success \| solvable | 0.00 | 0.00 | **1.00** |

RL is the **only** method that yields — a behaviour that's hard to express with a reactive rule.

### Social clearance (mean closest a pedestrian ever got, m — higher is more polite)

| Scenario | Straight | Classical | **RL** |
|---|:--:|:--:|:--:|
| factory `approaching` (head-on)   | 0.75 | 0.75 | **2.06** |
| warehouse `approaching`           | 0.75 | 0.75 | **1.93** |
| office `normal`                   | 0.74 | 0.73 | **1.06** |

### What RL does *not* solve

- It does not dominate every environment (classical wins `warehouse`/`urban normal`).
- It can **over-avoid** in narrow maps (`direction_change` on office/urban/hospital → 0.00 where a
  straight line succeeds).
- Map-to-map behaviour can be inconsistent (`factory crossing` = 0.00 while `warehouse crossing` = 1.00).
- **office & hospital** remain hard for *every* method.
- A policy trained on a narrow distribution becomes a specialist.

Failure analysis and generalization are treated as **first-class results**, not hidden.

---

## Generalization

The final policy is trained across **factory / warehouse / urban / office / hospital**, with multiple
scenarios and difficulties, held-out seeds, domain randomization, and unpredictable (non-linear)
pedestrian motion — the objective being *navigation behaviour that transfers*, not "train on factory →
succeed on factory". A crucial finding along the way: **you get the policy your eval selects for** — a
single-scenario eval quietly bred a factory specialist (its `sudden_stop`/`direction_change` got
*worse* with more training) until the eval was made multi-scenario.

---

## Commands

```bash
# build
colcon build --symlink-install && source install/setup.bash

# classical planner (Gazebo + RViz)
HUMANS="-1.3,-0.7,0,0" ./scripts/run_demo.sh          # GUI=false = headless; ./scripts/stop.sh clears Gazebo

# train the generalist (accelerated sim): a fast multi-map base, then the full generalist
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --algorithm ppo \
  --environments factory,warehouse,office,urban,hospital --scenario empty --difficulty easy \
  --frame-stack 4 --no-safety --net-arch 256,256 --n-envs 12 --timesteps 800000 --no-eval \
  --out ~/social_nav_rl_checkpoints/base_multi
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --algorithm ppo \
  --environments factory,warehouse,office,urban,hospital \
  --scenarios normal,crossing,sudden_stop,direction_change --difficulties easy,medium,hard \
  --require-feasible --human-unpredictable 0.3 --frame-stack 4 \
  --init-model ~/social_nav_rl_checkpoints/base_multi/ppo_urban_42.zip \
  --no-safety --learning-rate 0.0001 --n-envs 12 --timesteps 15000000 \
  --target-success 1.01 --patience 150 --stop-floor 0.4 \
  --out ~/social_nav_rl_checkpoints/generalist

# benchmark RL vs classical across all environments (feasibility-aware succ|solv)
for E in factory warehouse office urban hospital; do
  CUDA_VISIBLE_DEVICES= python3 scripts/rl_benchmark.py \
    --model ~/social_nav_rl_checkpoints/generalist/ppo_urban_42_best.zip \
    --environment $E --difficulties medium --episodes 20
done

# transfer check in Gazebo (RL owns /cmd_vel; RViz Fixed Frame = odom)
ros2 launch social_nav_bringup rl_sim.launch.py environment:=factory scenario:=crossing
ros2 run social_nav_rl social-nav-rl-eval --backend gazebo --policy rl --no-safety --debug \
  --model ~/social_nav_rl_checkpoints/generalist/ppo_urban_42_best.zip \
  --environment factory --scenario crossing --difficulty medium
```

Also supported: `--algorithm {ppo,sac,recurrent_ppo}`, `--curriculum`, warm-start (`--init-model`),
`--domain-rand`, early stopping (`--target-success`/`--patience`).

---

## Repository structure

```text
social_nav_ws/
├── social_nav_core/         classical planner mathematics (no ROS, GoogleTest)
├── social_nav_controller/   Nav2 SocialNav controller
├── social_nav_costs/        social costmap layer
├── social_nav_rl/           PPO/SAC/RecurrentPPO policy, Gym env, observations, rewards,
│                            prediction interface, safety, training, evaluation, feasibility oracle
├── social_nav_benchmarks/   scenarios + reports
├── social_nav_sim/          Gazebo worlds
├── social_nav_description/  robot description (MiR-100)
├── social_nav_bringup/      launch / configuration
├── social_nav_tools/        pedestrians + visualization + shared pedestrian model
└── social_nav_msgs/         ROS messages
```

Requirements: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, Nav2. RL extras:
`pip install 'stable-baselines3>=2.2' 'sb3-contrib>=2.2' gymnasium`.

---

## Research questions

1. **Classical vs learned** — when does a learned local policy outperform an interpretable reactive planner?
2. **Social behaviour** — can PPO learn waiting/yielding that's hard to express with reactive rules? *(Yes — `urban/sudden_stop`, 1.00 vs 0.00.)*
3. **Generalization** — can one policy transfer across factory/warehouse/office/hospital/urban?
4. **Safety** — how far can a learned policy be trusted without a separate safety layer?
5. **Feasibility** — how should benchmarks separate impossible scenarios from policy failures?
6. **Simulation fidelity** — how much does performance change from accelerated sim → Gazebo → Isaac Sim?

---

## Key lessons

1. **A high RL score can be meaningless** — 0.84 was obtained before realistic obstacle perception existed.
2. **Training-time action mutation is dangerous** — a safety layer changing the policy's action breaks credit assignment.
3. **Reward tuning isn't enough** — the crawl was fixed by the training interface/observation/distribution, not reward weights.
4. **Feasibility matters** — an impossible scenario should not count as a policy failure.
5. **Generalization matters more than one benchmark** — a policy that only works where it trained isn't convincing.
6. **You get the policy your eval selects for** — a single-scenario eval breeds a specialist; a multi-scenario eval selects a generalist.
7. **Failure cases are results** — a serious benchmark exposes where the system fails.

---

## Limitations

Simulation-to-real gap · simplified pedestrian behaviour · finite environment diversity · limited
observation history · classical (constant-velocity) prediction assumptions · approximate simulator
dynamics · no real-robot validation yet · the cost of exhaustive benchmarking. These are
**simulation-based research results**, not evidence of deployment readiness.

---

## Future work

```text
✓ classical SocialNav        ✓ multi-environment simulation
✓ PPO SocialNav              ✓ feasibility-aware evaluation
✓ human prediction           ✓ held-out evaluation
✓ realistic static geometry  ✓ Isaac Sim GPU-parallel training

→ learned pedestrian prediction   → transformer / recurrent policy   → multi-agent interaction
→ multi-robot social navigation   → large-scale Isaac Sim training   → sim-to-real validation
```

---

## Attribution & license

The MiR-100 visual meshes in `social_nav_description/meshes/mir/` (base, wheel, caster, SICK lidar)
are **third-party** — from the **ros2_mir_nav2_pick_place** project, adapted from the
[URDF files dataset](https://github.com/Daniella1/urdf_files_dataset), used here for visualization
only; verify the upstream mesh license if you redistribute. The URDF/kinematics in
`mir_social.urdf.xacro` were re-authored for this project. Everything else is original to SocialNav and
licensed **Apache-2.0** (see `LICENSE`). Architectural ideas were informed by public Nav2 /
social-navigation work (ROSNavBench, Social-Force planners); no code was copied.
