# Commands & Workflow — RL Social Navigation

Everything needed to build, train, evaluate, benchmark, and demo the RL social-navigation module.
Console scripts are ROS `ament_python` entry points, so they run via `ros2 run` (they live under
`lib/social_nav_rl/`, not on `PATH`).

## Setup

```bash
cd ~/Social_planner/social_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Learning extras (once): `pip install "stable-baselines3>=2.2" gymnasium` (torch already required).

Build (after any code/config change):
```bash
colcon build --packages-select social_nav_rl social_nav_tools --symlink-install
```

Tests (local pytest clashes with ROS launch_testing, so disable plugin autoload):
```bash
cd src/social_nav_rl
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/../social_nav_tools:$PYTHONPATH" python3 -m pytest test/ -q
```

`CUDA_VISIBLE_DEVICES=` in front of the eval/train commands forces CPU (SB3 MLP runs best on CPU and
it silences the GPU warning).

## Training pipeline (how the delivered model was produced)

The policy is built bottom-up: learn the empty aisle, then add people, then denser people. Each stage
warm-starts from the previous with a gentle learning rate. `--no-safety` trains without the safety
supervisor so the policy learns from the action it actually executes (the supervisor is a deploy-time
backstop only). `--net-arch 256,256` gives capacity for the timing skill.

```bash
# 1. empty aisle (no humans) — learn to drive the corridor with lidar
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --environment factory \
    --scenario empty --difficulty easy --net-arch 256,256 --n-envs 8 --timesteps 2000000 \
    --seed 42 --target-success 0.9 --out ~/social_nav_rl_checkpoints/obs_empty

# 2. easy (2 humans), multi-scenario, warm-started from empty
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --environment factory --difficulty easy \
    --scenarios normal,crossing,approaching,sudden_stop,direction_change \
    --init-model ~/social_nav_rl_checkpoints/obs_empty/ppo_factory_42_best.zip \
    --no-safety --learning-rate 0.0001 --n-envs 8 --timesteps 3000000 --seed 42 --target-success 0.85 \
    --out ~/social_nav_rl_checkpoints/obs_easy_multi

# 3. medium (4 humans), warm-started from easy  → the delivered model
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-train --environment factory --difficulty medium \
    --scenarios normal,crossing,approaching,sudden_stop,direction_change \
    --init-model ~/social_nav_rl_checkpoints/obs_easy_multi/ppo_factory_42_best.zip \
    --no-safety --learning-rate 0.0001 --n-envs 8 --timesteps 4000000 --seed 42 --patience 20 \
    --out ~/social_nav_rl_checkpoints/obs_medium_multi
```

Delivered model (preserved, write-locked):
`~/social_nav_rl_checkpoints/BEST_factory_medium_multi/ppo_factory_42_best.zip`

Key training flags: `--scenarios a,b,c` (sample a scenario per episode), `--no-safety`,
`--net-arch`, `--frame-stack N`, `--init-model` (warm-start; frame-stack auto-detected),
`--learning-rate`, `--ent-coef`, `--target-success`, `--patience`, `--stop-floor` (plateau-stop only
fires once best success clears this — default 0.3, so weak runs won't auto-stop; Ctrl+C them).

## Evaluate a policy (accelerated env, held-out test seeds)

```bash
# RL (use the _best.zip); --no-safety = raw policy (its correct deploy mode here)
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-eval --policy rl --no-safety \
  --model ~/social_nav_rl_checkpoints/BEST_factory_medium_multi/ppo_factory_42_best.zip \
  --environment factory --scenario crossing --difficulty medium --split test --episodes 20

# classical Social-Force baseline / straight-line baseline
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-eval --policy sfm --environment factory \
  --scenario crossing --difficulty medium --split test --episodes 20
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-eval --policy straight --environment factory \
  --scenario crossing --difficulty medium --split test --episodes 20

# --debug prints the model's live inputs (humans seen, nearest human, lidar min) to verify ingestion
```

## Benchmark (the report's tables)

```bash
# main comparison on the trained env (RL vs classical vs straight, all scenarios/difficulties)
python3 scripts/rl_benchmark.py --environment factory --policies straight,sfm,rl \
    --scenarios normal,crossing,approaching,sudden_stop,direction_change,dense_crowd \
    --difficulties easy,medium --episodes 20

# deployment config (safety supervisor as a backstop for all policies)
python3 scripts/rl_benchmark.py --environment factory --policies straight,sfm,rl --safe-filter …

# cross-environment generalization
python3 scripts/rl_benchmark.py --environment warehouse --policies rl,straight \
    --scenarios normal,crossing --difficulties medium --episodes 15
```

Outputs per run: `~/social_nav_rl_results/benchmark/<env>_<stamp>/` with `episodes.jsonl`,
`aggregate.csv`, `summary.json`. The report tables live in `docs/benchmarks/`.

## Run it in Gazebo (the transfer check)

```bash
# Terminal 1 — sim + robot + pedestrians, NO Nav2 controller (RL owns /cmd_vel).
# Prefix forces gzclient onto the NVIDIA GPU (Optimus on-demand laptops).
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch social_nav_bringup rl_sim.launch.py \
  environment:=factory scenario:=crossing difficulty:=medium seed:=110000 gui:=true

# In RViz set Global Options → Fixed Frame → "odom" (rl_sim has no map frame) to see /scan.

# Terminal 2 — drive the live sim with the RL policy (raw). --debug + the per-episode [inputs] log
# confirm odom/scan/humans/lidar are arriving.
CUDA_VISIBLE_DEVICES= ros2 run social_nav_rl social-nav-rl-eval --backend gazebo --policy rl --no-safety --debug \
  --model ~/social_nav_rl_checkpoints/BEST_factory_medium_multi/ppo_factory_42_best.zip \
  --environment factory --scenario crossing --difficulty medium --split test --episodes 5
```

## Failure replays

Failed eval episodes are saved as `failure_*.json` (start/goal, per-step robot pose, action, safety
kind, events) under the eval output dir, for offline inspection.

## What was done (chronological)

1. Built `social_nav_rl` (ament_python): observation adapter, action mapper, per-component reward,
   safety supervisor, Gymnasium env with an accelerated `MockBackend` and a live `GazeboBackend`,
   PPO training/eval CLIs, curriculum, baselines, diagnostics, config files, tests.
2. First trained policy hit 0.84 in the accelerated env — but that env was near-empty of static
   obstacles and the policy had no obstacle perception.
3. Ran it in Gazebo → 0.00: it drove into pallet racks it could not see. Diagnosed via failure
   replay (obstacle collision, humans far away).
4. Added real static geometry to the env (`world_obstacles.py`) and a 12-beam lidar to the
   observation (`perception.py`), computed identically in mock and Gazebo.
5. Fixed the resulting "freeze/crawl" (the safety supervisor was rewriting actions during training →
   broken credit assignment): added `--no-safety` training; supervisor stays a deploy backstop.
6. Added multi-scenario training so the policy learns to react to crossing/approaching/sudden humans,
   not just barrel down `normal`.
7. Built `scripts/rl_benchmark.py` and ran the matched RL-vs-classical benchmark (this report).
8. Fixed `social_nav_benchmarks` packaging (an illegal `--` in an XML comment made colcon build it as
   plain Python, so `ros2 run social_nav_benchmarks …` was undiscoverable) — unblocks the future
   Gazebo Nav2 comparison.

## Not done / next

- High-fidelity RL-vs-Nav2 comparison in Gazebo (framework ready; needs real-time runs).
- Move start/goal out of obstacles for `urban`/`hospital` so they can be benchmarked.
- Optional Gazebo-backend fine-tune to close the sim-to-sim gap.
