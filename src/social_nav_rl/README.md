# social_nav_rl

Experimental reinforcement-learning social navigation policy for the SocialNav project. It
consumes the existing human tracking + classical prediction, learns a continuous local command,
and runs behind a safety supervisor so the policy never bypasses the hard constraints. Built to
be compared against Nav2, the classical SocialNav planner, and a hybrid.

## Install the learning extras (once)

```bash
pip install "stable-baselines3>=2.2" gymnasium    # torch is already required by the project
```

## Layout

- Pure core (no ROS/torch): `observation.py` (fixed-size obs, nearest-N + masking + ablation
  flags), `reward.py` (per-component, each weighted/logged), `safety.py` (supervisor), `action.py`
  (limit/accel mapping), `features.py`, `config.py`.
- `env.py` — Gymnasium env. **MockBackend** is the accelerated training env (kinematic, reuses
  the project's environment registry + pedestrian model, so humans behave as in Gazebo). A
  GazeboBackend (real sensors/physics/prediction) is the high-fidelity path, run in the sim.
- `curriculum.py` (10 levels + domain randomization + train/val/test seed split), `latency.py`,
  `replay.py`, `checkpoint.py`, `hybrid.py`, `policies.py`, `diagnostics.py` (reward-hacking).
- `config/*.yaml` — observation / action / reward / safety / ppo.

## Train

These are ROS console scripts, so run them with `ros2 run` (they live in `lib/social_nav_rl/`,
not on `PATH`). Source the workspace first (`source install/setup.bash`).

```bash
# validate the pipeline instantly (no training):
ros2 run social_nav_rl social-nav-rl-train --n-envs 4 --check
# real training (accelerated env; ~minutes on CPU):
ros2 run social_nav_rl social-nav-rl-train --environment factory --curriculum --n-envs 8 \
    --timesteps 3000000 --seed 42
```

Training evaluates on the held-out `val` split every `--eval-freq` steps, saves the **best**
policy as `ppo_<env>_<seed>_best.zip`, and **stops early** when the eval success rate reaches
`--target-success` (default 0.9) or stops improving for `--patience` evals. So a late collapse
can't destroy your result, and you don't have to guess `--timesteps`. Key knobs:

```
--target-success 0.9   # the "high score" that ends training
--eval-freq 25000      # evaluate every N steps
--n-eval-episodes 20   # episodes per evaluation (more = less noisy success rate)
--patience 15          # plateau-stop after this many evals with no improvement
--stop-floor 0.3       # don't plateau-stop until best success clears this (survive the noisy
                       #   early phase); --target-success can still stop anytime
--eval-split val       # held-out split to score on
--no-eval              # disable all of the above; train exactly --timesteps
--init-model PATH      # warm-start from a checkpoint (e.g. train easy, then continue on medium)
--net-arch 256,256     # policy/value hidden sizes (default 64,64); more capacity for timing.
                       #   Fixed by the checkpoint under --init-model; set it on the fresh run.
--frame-stack 4        # stack the last N obs so a feedforward policy infers human motion over
                       #   time (attacks late-reaction collisions). Warm-start auto-detects N.
--learning-rate 5e-5   # override config/checkpoint LR (needed to anneal a warm-start fine-tune)
--ent-coef 0.001       # override entropy coef (lower = sharpen a fine-tune once competent)
--difficulty easy      # easy < medium < hard (fewer/slower humans); use easy to bootstrap
--save-freq 100000     # also drop periodic checkpoints
--tensorboard DIR      # write curves (eval/success_rate, eval/mean_reward, rollout/*)
```

Hard environments (e.g. `factory/medium` = 4 crossing humans) are hard to learn from scratch.
Bootstrap: train `--difficulty easy` to a good policy, then continue on medium with
`--init-model <that _best.zip>`.

## Evaluate / ablate (unseen seeds; no privileged info for RL)

Use the `_best.zip` from training:

```bash
ros2 run social_nav_rl social-nav-rl-eval --policy rl \
    --model ~/social_nav_rl_checkpoints/ppo_factory_42_best.zip \
    --environment hospital --scenario crossing --split test --episodes 50
ros2 run social_nav_rl social-nav-rl-eval --policy sfm --environment factory \
    --difficulty medium --split test --episodes 50   # classical Social-Force baseline
ros2 run social_nav_rl social-nav-rl-eval --policy straight --environment factory \
    --difficulty medium --split test --episodes 50   # naive baseline
ros2 run social_nav_rl social-nav-rl-ablate --policy straight --environment urban \
    --scenario crossing --episodes 20
```

## RL vs. classical (same env, same seeds)

`--policy sfm` is a classical Social-Force local planner (goal attraction + exponential human
repulsion) that runs in the *same* accelerated env on the *same* test seeds as the RL policy, so
it's a fair, matched comparison - not a Gazebo apples-to-oranges. Reference numbers on
factory/medium (test, 50 eps): straight 0.16 < Social-Force 0.28 < RL ~0.84.

The higher-fidelity Nav2 / classical planners in Gazebo run through `social_nav_benchmarks`
(`ros2 run social_nav_benchmarks social-nav-benchmark --planner ...`).
