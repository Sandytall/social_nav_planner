# SocialNav — social navigation, classical **and** learned, rigorously evaluated

Two socially-aware local planners for a differential-drive robot moving around people — a **classical**
Nav2 planner and a **learned (PPO)** policy — plus the thing that ties them together: a
**feasibility-aware, multi-environment benchmark** that measures, honestly, where a learned social
planner helps and where it doesn't.

Built on **ROS 2 Humble + Gazebo Classic 11** with a MiR-100 robot. The learned policy also trains
**GPU-parallel in NVIDIA Isaac Sim / Isaac Lab** (companion scripts in `~/isaac`).

> **The honest headline.** A first RL policy scored **0.84** in the accelerated sim and **0.00** in
> the real simulator — it had no obstacle perception and had been tested in near-empty space. After
> fixing the *evaluation* (real obstacles, feasibility-aware metrics, unpredictable pedestrians,
> multi-map training), the learned policy is **on par with classical reactive planners** — better at
> some things (waiting/yielding, keeping social distance), worse at others, and **no method solves the
> hardest maps**. The real deliverable is the evaluation itself.

---

## What's in here

- **Classical planner** — `social_nav_controller` (a `nav2_core::Controller`) + `social_nav_costs`
  (a `nav2_costmap_2d::Layer`). Pure-pursuit base command, motion sampling scored by an anisotropic
  personal-space cost + time-to-collision + group intrusion; a social costmap layer routes the global
  plan *around* people instead of stopping in front of them. Behaviour modes scale speed with
  proximity. Planner math is in `social_nav_core` (no ROS deps, GoogleTest unit tests).
- **Learned policy** — `social_nav_rl`. PPO (Stable-Baselines3) trained in a fast accelerated
  kinematic sim (~3000–6000 steps/s) that **shares the pedestrian model and scenario registry with
  Gazebo**, then transfers to Gazebo. Obstacle-aware (12-beam down-sampled lidar, computed identically
  in mock and from real `/scan`), social (nearest-human features + crossing time-to-conflict),
  trainable across all maps/scenarios/difficulties with domain randomization, unpredictable
  pedestrians, and a **feasibility-filtered, multi-scenario generalist eval**. 87 unit tests.
- **Evaluation** — `social_nav_benchmarks` + `scripts/rl_benchmark.py`. RL vs classical baselines
  (`straight`, Social-Force `sfm`) on matched scenarios/seeds, reporting **success on *solvable*
  episodes**: a feasibility oracle (`social_nav_rl/feasibility.py`) separates *impossible scenario*
  from *policy failure*.

---

## Results

Final generalist policy (PPO + frame-stack, trained across 5 maps × scenarios × difficulties),
**success on solvable episodes** (`succ|solv`), medium difficulty, 20 episodes/cell, held-out test
seeds. Full data: [`docs/EVALUATION_REPORT.md`](docs/EVALUATION_REPORT.md).

**`normal` — goal-reaching through a shared corridor:**

| Environment | Straight | Social-Force | **RL** |
|---|:--:|:--:|:--:|
| factory   | 0.50 | 0.71 | **0.86** |
| warehouse | 0.50 | **0.86** | 0.71 |
| urban     | 0.53 | **0.65** | 0.35 |
| office    | 0.00 | 0.00 | 0.00 |
| hospital  | 0.00 | 0.00 | 0.00 |

**Notable per-scenario behaviour:**

- **`sudden_stop` (a pedestrian stops in the path): RL = 1.00 on urban — the *only* method that
  waits.** Straight and Social-Force get 0.00 (they can't yield). This is the payoff of a
  waiting-aware reward.
- **Social clearance:** RL keeps *more* distance from people almost everywhere (e.g. ~2.0 m on head-on
  approaches vs ~0.75 m for the baselines).
- **`crossing` / `direction_change`:** RL matches classical (1.00) on factory & warehouse, but is
  inconsistent on the narrow maps (e.g. it over-avoids on `direction_change` in office/urban/hospital
  where a straight line succeeds).
- **office & hospital:** *every* method — including a straight line — scores ~0 on `normal`/`crossing`.
  Those narrow, furniture-filled corridors are hard for classical *and* learned planners alike.

**Verdict:** a credible generalist, not a dominant one. It matches or beats classical on the maps it
was built for, does something classical can't (yield/wait), and keeps better social distance — but it
doesn't win across the board, and the hardest maps defeat everyone.

---

## Run it

### Classical planner (Gazebo + RViz)
```bash
colcon build --symlink-install && source install/setup.bash
# drive to a goal while avoiding a person on the route:
HUMANS="-1.3,-0.7,0,0" ./scripts/run_demo.sh
```
`GUI=false ./scripts/run_demo.sh` runs headless (RViz only). `./scripts/stop.sh` clears a stuck Gazebo.

### Learned policy — train (accelerated sim, your compute)
```bash
# 1) a fast "drive the empty corridor" base across all maps (fixed budget, no early-stop trap)
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --algorithm ppo \
  --environments factory,warehouse,office,urban,hospital --scenario empty --difficulty easy \
  --frame-stack 4 --no-safety --net-arch 256,256 --n-envs 12 --timesteps 800000 --no-eval \
  --out ~/social_nav_rl_checkpoints/base_multi
# 2) the generalist: all maps × scenarios × difficulties, feasibility-filtered, unpredictable humans
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --algorithm ppo \
  --environments factory,warehouse,office,urban,hospital \
  --scenarios normal,crossing,sudden_stop,direction_change --difficulties easy,medium,hard \
  --require-feasible --human-unpredictable 0.3 --frame-stack 4 \
  --init-model ~/social_nav_rl_checkpoints/base_multi/ppo_urban_42.zip \
  --no-safety --learning-rate 0.0001 --n-envs 12 --timesteps 15000000 \
  --target-success 1.01 --patience 150 --stop-floor 0.4 \
  --out ~/social_nav_rl_checkpoints/generalist
```
Also supported: `--algorithm {ppo,sac,recurrent_ppo}`, warm-start (`--init-model`), `--domain-rand`.

### Learned policy — evaluate / benchmark / Gazebo
```bash
# per-environment benchmark (RL vs classical, feasibility-aware succ|solv)
for E in factory warehouse office urban hospital; do
  CUDA_VISIBLE_DEVICES= python3 scripts/rl_benchmark.py \
    --model ~/social_nav_rl_checkpoints/generalist/ppo_urban_42_best.zip \
    --environment $E --difficulties medium --episodes 20
done
# transfer check in Gazebo (RL owns /cmd_vel; RViz Fixed Frame = odom):
ros2 launch social_nav_bringup rl_sim.launch.py environment:=factory scenario:=crossing
ros2 run social_nav_rl social-nav-rl-eval --backend gazebo --policy rl --no-safety --debug \
  --model ~/social_nav_rl_checkpoints/generalist/ppo_urban_42_best.zip \
  --environment factory --scenario crossing --difficulty medium
```

### Isaac Sim (GPU-parallel, optional)
Companion scripts in `~/isaac` import the same MiR robot into Isaac Sim 4.5 / Isaac Lab and train the
social-nav task GPU-parallel (rsl_rl) in Isaac's prebuilt warehouse. See `~/isaac/README.md`.

---

## Repo layout

| Package | Role |
|---|---|
| `social_nav_core` | classical planner math (no ROS, GoogleTest) |
| `social_nav_controller`, `social_nav_costs` | Nav2 controller + social costmap layer |
| `social_nav_rl` | the learned policy: obs/reward/safety/action, env, PPO/SAC/RecurrentPPO training, eval, feasibility oracle, domain randomization |
| `social_nav_benchmarks` | scenario benchmark + reports |
| `social_nav_sim`, `social_nav_description`, `social_nav_bringup` | Gazebo worlds, MiR robot, launch |
| `social_nav_tools` | simulated pedestrians, RViz markers, shared pedestrian model |
| `social_nav_msgs` | human / metrics messages |

Requirements: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, Nav2; for the RL extras
`pip install 'stable-baselines3>=2.2' 'sb3-contrib>=2.2' gymnasium`.

---

## Docs

- **[docs/JOURNAL.md](docs/JOURNAL.md)** — the full build story: what broke, how it was diagnosed, and
  the engineering lessons (honest benchmarking, feasibility-aware evaluation, "you get the policy your
  eval selects for", …). The source for the write-up.
- **[docs/EVALUATION_REPORT.md](docs/EVALUATION_REPORT.md)** — the detailed, measured RL-vs-classical
  evaluation.

---

## Attribution & license

The MiR-100 visual meshes in `social_nav_description/meshes/mir/` (base, wheel, caster, SICK lidar) are
third-party — sourced from the **ros2_mir_nav2_pick_place** project, adapted from the
[URDF files dataset](https://github.com/Daniella1/urdf_files_dataset), used here for visualization
only. If you redistribute, verify the upstream mesh license and preserve that attribution. The URDF /
kinematics in `mir_social.urdf.xacro` were re-authored for this project.

Everything else is original to SocialNav and licensed **Apache-2.0** (see `LICENSE`). Architectural
ideas were informed by public Nav2 / social-navigation work (ROSNavBench, Social-Force planners); no
code was copied.
