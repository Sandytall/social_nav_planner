"""PPO training CLI for RL social navigation (accelerated MockBackend by default).

  # single env
  ros2 run social_nav_rl social-nav-rl-train --environment factory --curriculum --seed 42 \
      --timesteps 3000000
  # parallel envs (big speedup on multi-core), checkpoint every 100k, tensorboard on
  ros2 run social_nav_rl social-nav-rl-train --environment warehouse --n-envs 8 \
      --timesteps 1000000 --save-freq 100000 --tensorboard ~/social_nav_rl_tb
  # validate the whole pipeline in ~a second, no training:
  ros2 run social_nav_rl social-nav-rl-train --n-envs 4 --check

By default training periodically evaluates the current policy on the held-out `val` split,
saves the BEST policy (`ppo_<env>_<seed>_best.zip`, highest success rate) and STOPS EARLY once
it reaches the target success rate (`--target-success`) or stops improving for `--patience`
consecutive evals. So you don't have to babysit the log or guess `--timesteps`, and a late
collapse can never destroy your result - the best checkpoint is already on disk. Use `--no-eval`
to disable this and train for exactly `--timesteps`.

`--check` builds the (vectorized) env + PPO policy and runs one predict/step WITHOUT training.
Actual training (`.learn`) is the long run; it trains on the accelerated env (kinematic + real
human behaviour), not Gazebo.
"""
import argparse
import os
import random
from functools import partial

from social_nav_rl import config as C
from social_nav_rl.checkpoint import CheckpointMeta
from social_nav_rl.curriculum import SEED_SPLITS, Curriculum, curriculum_sampler
from social_nav_rl.env import EpisodeConfig, SocialNavEnv


def make_env(spec: dict):
    """Top-level (picklable) env factory for SubprocVecEnv. `spec` is a plain dict."""
    from stable_baselines3.common.monitor import Monitor

    exp = C.load_experiment(spec.get("config_dir") or None)
    common = dict(obs_cfg=exp["observation"], limits=exp["action"],
                  reward_cfg=exp["reward"], safety_cfg=exp["safety"],
                  safe_filter=spec.get("safe_filter", True))
    scenarios = spec.get("scenarios") or []
    if spec["curriculum"]:
        cur = Curriculum(split="train", max_steps=spec["max_steps"])
        sampler = curriculum_sampler(cur, random.Random(spec["seed"]))
        env = SocialNavEnv(episode_sampler=sampler, **common)
    elif len(scenarios) > 1:
        # Sample a scenario per episode so the policy trains on situations where barreling fails
        # (crossing / head-on / sudden-stop / direction-change), not just `normal`. Train-split seeds.
        rng = random.Random(spec["seed"])
        env_name, diff, ms = spec["environment"], spec["difficulty"], spec["max_steps"]
        lo, hi = SEED_SPLITS["train"]

        def sampler(_prev_success):
            sc = rng.choice(scenarios)
            return (EpisodeConfig(environment=env_name, scenario=sc, difficulty=diff,
                                  max_steps=ms), rng.randint(lo, hi - 1))
        env = SocialNavEnv(episode_sampler=sampler, **common)
    else:
        ep = EpisodeConfig(environment=spec["environment"], scenario=spec["scenario"],
                           difficulty=spec["difficulty"], max_steps=spec["max_steps"])
        env = SocialNavEnv(ep=ep, **common)
    return Monitor(env)


def _make_eval_stop_callback(base_cls):
    """Build the EvalStopCallback subclass (SB3 is imported lazily, so the pure package needs none)."""
    import numpy as np

    class EvalStopCallback(base_cls):
        """Evaluate on held-out seeds, keep the best policy, stop at a high score or on plateau.

        Every ``eval_freq_ts`` timesteps it runs ``seeds`` deterministic episodes on a fixed eval
        env, records success rate + mean reward, saves ``best_path`` whenever the success rate
        improves, and returns False (stops training) once the best success reaches
        ``target_success`` or fails to improve for ``patience`` consecutive evals.
        """

        def __init__(self, eval_env, seeds, best_path, target_success, patience,
                     eval_freq_ts, save_meta_fn, stop_floor=0.0, frame_stack=1, verbose=1):
            super().__init__(verbose)
            self.eval_env = eval_env
            self.seeds = list(seeds)
            # Match training's frame stacking during eval, else the policy sees the wrong-size obs.
            self.stacker = None
            if frame_stack and frame_stack > 1:
                from social_nav_rl.policies import FrameStacker
                base = int(np.prod(eval_env.observation_space.shape))
                self.stacker = FrameStacker(frame_stack, base)
            self.best_path = best_path
            self.target_success = target_success
            self.patience = patience
            self.eval_freq_ts = max(1, eval_freq_ts)
            self.save_meta_fn = save_meta_fn
            # Don't plateau-stop until the best success clears this floor: early training is noisy
            # and passes through 0, so a fluke early "best" must not end the run before the policy
            # is actually competitive. `target_success` can still stop it at any point.
            self.stop_floor = stop_floor
            self._next_eval = self.eval_freq_ts
            self._best_success = -1.0
            self._best_reward = float("-inf")
            self._no_improve = 0

        def _run_eval(self):
            n_succ, rewards = 0, []
            for s in self.seeds:
                obs, _ = self.eval_env.reset(seed=s)
                x = self.stacker.reset(obs) if self.stacker is not None else obs
                done, ep_r, info = False, 0.0, {}
                while not done:
                    act, _ = self.model.predict(x, deterministic=True)
                    obs, r, term, trunc, info = self.eval_env.step(act)
                    ep_r += float(r)
                    done = term or trunc
                    x = self.stacker.push(obs) if self.stacker is not None else obs
                ev = info.get("events", {})
                if ev.get("reached") and not ev.get("collision") and not ev.get("human_collision"):
                    n_succ += 1
                rewards.append(ep_r)
            return n_succ / max(1, len(self.seeds)), float(np.mean(rewards))

        def _on_step(self):
            if self.num_timesteps < self._next_eval:
                return True
            self._next_eval = self.num_timesteps + self.eval_freq_ts
            succ, mean_r = self._run_eval()
            self.logger.record("eval/success_rate", succ)
            self.logger.record("eval/mean_reward", mean_r)
            improved = (succ > self._best_success + 1e-9 or
                        (abs(succ - self._best_success) <= 1e-9 and mean_r > self._best_reward))
            if improved:
                self._best_success, self._best_reward = succ, mean_r
                self._no_improve = 0
                self.model.save(self.best_path)
                if self.save_meta_fn:
                    self.save_meta_fn(self.best_path, self.num_timesteps)
                print(f"[eval] ts={self.num_timesteps} success={succ:.2f} reward={mean_r:.1f} "
                      f"-> new best, saved {os.path.basename(self.best_path)}")
            else:
                self._no_improve += 1
                print(f"[eval] ts={self.num_timesteps} success={succ:.2f} reward={mean_r:.1f} "
                      f"(best {self._best_success:.2f}, no-improve {self._no_improve}/{self.patience})")
            if succ >= self.target_success:
                print(f"[eval] reached target success {self.target_success:.2f} -> stopping early.")
                return False
            if self._best_success >= self.stop_floor and self._no_improve >= self.patience:
                print(f"[eval] no improvement for {self.patience} evals "
                      f"(best success {self._best_success:.2f}) -> stopping early.")
                return False
            return True

    return EvalStopCallback


def main():
    ap = argparse.ArgumentParser(description="Train PPO RL social navigation")
    ap.add_argument("--algorithm", default="ppo", choices=["ppo"])
    ap.add_argument("--environment", default="urban")
    ap.add_argument("--scenario", default="normal")
    ap.add_argument("--scenarios", default="",
                    help="comma-separated scenarios sampled per episode during training (e.g. "
                         "normal,crossing,head_on,sudden_stop,direction_change) so the policy learns "
                         "to react to approaching/sudden humans instead of barreling. Eval still uses "
                         "--scenario. Ignored with --curriculum.")
    ap.add_argument("--difficulty", default="medium")
    ap.add_argument("--curriculum", action="store_true", help="train across the 10-level curriculum")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--timesteps", type=int, default=3_000_000)
    ap.add_argument("--n-envs", dest="n_envs", type=int, default=1, help="parallel envs")
    ap.add_argument("--vec", default="subproc", choices=["subproc", "dummy"])
    ap.add_argument("--max-steps", dest="max_steps", type=int, default=600)
    ap.add_argument("--device", default="cpu", choices=["cpu", "cuda", "auto"])
    ap.add_argument("--config", default="", help="config dir (default: package config/)")
    ap.add_argument("--save-freq", dest="save_freq", type=int, default=0,
                    help="also checkpoint every N total steps (0 = off; best model is saved anyway)")
    ap.add_argument("--tensorboard", default="", help="tensorboard log dir (empty = off)")
    ap.add_argument("--out", default=os.path.expanduser("~/social_nav_rl_checkpoints"))
    ap.add_argument("--init-model", dest="init_model", default="",
                    help="warm-start: load this checkpoint's weights and keep training "
                         "(e.g. train on easy, then continue on medium)")
    ap.add_argument("--net-arch", dest="net_arch", default="",
                    help="policy/value hidden layer sizes, e.g. 256,256 (default SB3 64,64); "
                         "more capacity for the timing skill. Fixed by the checkpoint under "
                         "--init-model, so set it on the fresh (easy) run and it carries over.")
    ap.add_argument("--frame-stack", dest="frame_stack", type=int, default=1,
                    help="stack the last N observations so a feedforward policy can infer human "
                         "motion/heading over time (default 1 = off; try 4). Attacks the late-"
                         "reaction collisions. It is an env wrapper, so pass the SAME N on both "
                         "the fresh run AND any --init-model warm-start (eval infers it from the "
                         "checkpoint automatically).")
    ap.add_argument("--learning-rate", dest="lr", type=float, default=0.0,
                    help="override the config/checkpoint learning rate (needed to anneal a "
                         "warm-start fine-tune, e.g. 0.00005)")
    ap.add_argument("--ent-coef", dest="ent", type=float, default=-1.0,
                    help="override the config/checkpoint entropy coef (e.g. 0.001 to sharpen a "
                         "fine-tune once the policy is already competent)")
    ap.add_argument("--check", action="store_true", help="build + 1 step, no training")
    # periodic evaluation + early stopping (on by default)
    ap.add_argument("--no-eval", dest="no_eval", action="store_true",
                    help="disable periodic eval + early stopping (train exactly --timesteps)")
    ap.add_argument("--eval-split", dest="eval_split", default="val", choices=list(SEED_SPLITS),
                    help="held-out split to evaluate on (default: val)")
    ap.add_argument("--eval-freq", dest="eval_freq", type=int, default=25_000,
                    help="evaluate every N timesteps")
    ap.add_argument("--n-eval-episodes", dest="n_eval_episodes", type=int, default=20,
                    help="episodes per evaluation (more = less noisy success rate)")
    ap.add_argument("--target-success", dest="target_success", type=float, default=0.9,
                    help="stop early once the eval success rate reaches this (the 'high score')")
    ap.add_argument("--patience", type=int, default=15,
                    help="stop early after this many evals with no improvement (above --stop-floor)")
    ap.add_argument("--no-safety", dest="no_safety", action="store_true",
                    help="train + eval-callback WITHOUT the safety supervisor rewriting actions, so "
                         "PPO learns from the action it actually executed (the supervisor throttling "
                         "under dense humans otherwise breaks credit assignment and causes a crawl). "
                         "Safety is taught via the collision reward; the supervisor is still applied "
                         "at deployment (evaluate.py defaults to safe_filter on).")
    ap.add_argument("--stop-floor", dest="stop_floor", type=float, default=0.3,
                    help="don't plateau-stop until best success clears this (avoids quitting "
                         "during the noisy early phase); --target-success can still stop anytime")
    args = ap.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
        from stable_baselines3.common.vec_env import (
            DummyVecEnv, SubprocVecEnv, VecFrameStack)
    except ImportError:
        raise SystemExit("stable-baselines3 not installed. Run:\n"
                         "  pip install 'stable-baselines3>=2.2' gymnasium")

    cfg_dir = args.config or C.config_dir()
    ppo_kw = C.load_ppo_config(os.path.join(cfg_dir, "ppo.yaml"))
    exp = C.load_experiment(args.config or None)
    scenario_list = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    if scenario_list:
        from social_nav_tools.environments import SCENARIO_TYPES
        bad = [s for s in scenario_list if s not in SCENARIO_TYPES]
        if bad:
            raise SystemExit(f"unknown --scenarios {bad}; valid: {list(SCENARIO_TYPES)}")
    specs = [{"config_dir": args.config or None, "curriculum": args.curriculum,
              "environment": args.environment, "scenario": args.scenario,
              "scenarios": scenario_list,
              "difficulty": args.difficulty, "max_steps": args.max_steps,
              "safe_filter": not args.no_safety,
              "seed": args.seed + rank} for rank in range(max(1, args.n_envs))]
    fns = [partial(make_env, s) for s in specs]
    if args.n_envs > 1 and args.vec == "subproc":
        # fork avoids re-importing the entry module (forkserver/spawn would recurse from the
        # console-script and can't find a `-`/stdin main); env workers use no CUDA, so fork is safe.
        venv = SubprocVecEnv(fns, start_method="fork")
    else:
        venv = DummyVecEnv(fns)
    # Effective frame-stack depth. On a warm-start, peek the checkpoint's obs dim (from the zip
    # metadata, no model build) and INFER it so the env is wrapped to match automatically - the
    # user needn't re-pass --frame-stack.
    frame_stack = args.frame_stack
    base_dim = venv.observation_space.shape[0]
    if args.init_model:
        from stable_baselines3.common.save_util import load_from_zip_file
        _data, _, _ = load_from_zip_file(args.init_model, device="cpu")
        ckpt_dim = _data["observation_space"].shape[0]
        if ckpt_dim != base_dim:
            if base_dim and ckpt_dim % base_dim == 0:
                inferred = ckpt_dim // base_dim
                if args.frame_stack > 1 and args.frame_stack != inferred:
                    raise SystemExit(f"--frame-stack {args.frame_stack} conflicts with the "
                                     f"checkpoint's stack depth {inferred}.")
                frame_stack = inferred
            else:
                raise SystemExit(f"checkpoint obs dim {ckpt_dim} is not a multiple of this env's "
                                 f"{base_dim}; the model is incompatible with this observation.")
    if frame_stack > 1:
        venv = VecFrameStack(venv, n_stack=frame_stack)

    # CLI overrides (apply whether fresh or warm-started; a warm-start otherwise keeps the
    # checkpoint's hyperparameters, so this is how you anneal a fine-tune).
    ppo_kw = dict(ppo_kw)
    if args.lr > 0:
        ppo_kw["learning_rate"] = args.lr
    if args.ent >= 0:
        ppo_kw["ent_coef"] = args.ent
    policy_kwargs = {}
    if args.net_arch:
        policy_kwargs["net_arch"] = [int(x) for x in args.net_arch.split(",") if x.strip()]

    if args.init_model:
        # load WITH env (handles a different n_envs than the checkpoint); venv is already wrapped
        # to the matching obs dim above.
        model = PPO.load(args.init_model, env=venv, device=args.device,
                         tensorboard_log=(args.tensorboard or None))
        if args.net_arch:
            print("note: --net-arch ignored with --init-model (architecture is fixed by the "
                  "checkpoint; set it on the fresh run instead).")
        if args.lr > 0:
            from stable_baselines3.common.utils import get_schedule_fn
            model.learning_rate = args.lr
            model.lr_schedule = get_schedule_fn(args.lr)
        if args.ent >= 0:
            model.ent_coef = args.ent
        extras = ((f" fs={frame_stack}" if frame_stack > 1 else "")
                  + (f" lr={args.lr}" if args.lr > 0 else "")
                  + (f" ent={args.ent}" if args.ent >= 0 else ""))
        print(f"warm-started from {args.init_model}{extras}")
    else:
        model = PPO("MlpPolicy", venv, device=args.device, seed=args.seed,
                    tensorboard_log=(args.tensorboard or None),
                    verbose=0 if args.check else 1,
                    policy_kwargs=(policy_kwargs or None), **ppo_kw)

    if args.check:
        obs = venv.reset()
        actions, _ = model.predict(obs, deterministic=True)
        venv.step(actions)
        print(f"[check] n_envs={args.n_envs} vec={args.vec if args.n_envs > 1 else 'single'} "
              f"obs={obs.shape} action={actions.shape} curriculum={'on' if args.curriculum else 'off'} "
              f"eval={'off' if args.no_eval else 'on'} -> pipeline OK (no training run).")
        venv.close()
        return

    os.makedirs(args.out, exist_ok=True)

    def save_meta(path, steps):
        CheckpointMeta(algorithm="ppo", environment=args.environment, split="train",
                       training_steps=int(steps), seed=args.seed,
                       reward_config=dict(exp["reward"].weights),
                       obs_config=vars(exp["observation"]),
                       curriculum={"enabled": bool(args.curriculum)},
                       extra={"n_envs": args.n_envs, "ppo": ppo_kw}).save(path)

    callbacks = []
    if args.save_freq > 0:
        callbacks.append(CheckpointCallback(
            save_freq=max(1, args.save_freq // max(1, args.n_envs)),
            save_path=args.out, name_prefix=f"ppo_{args.environment}"))

    best_path = os.path.join(args.out, f"ppo_{args.environment}_{args.seed}_best.zip")
    eval_env = None
    if not args.no_eval:
        eval_env = SocialNavEnv(
            ep=EpisodeConfig(environment=args.environment, scenario=args.scenario,
                             difficulty=args.difficulty, max_steps=args.max_steps),
            obs_cfg=exp["observation"], limits=exp["action"],
            reward_cfg=exp["reward"], safety_cfg=exp["safety"],
            safe_filter=not args.no_safety)
        lo, _ = SEED_SPLITS[args.eval_split]
        seeds = list(range(lo, lo + max(1, args.n_eval_episodes)))
        EvalStopCallback = _make_eval_stop_callback(BaseCallback)
        callbacks.append(EvalStopCallback(
            eval_env, seeds, best_path, args.target_success, args.patience,
            args.eval_freq, save_meta, stop_floor=args.stop_floor,
            frame_stack=frame_stack))

    model.learn(total_timesteps=args.timesteps, callback=callbacks or None)
    final_path = os.path.join(args.out, f"ppo_{args.environment}_{args.seed}.zip")
    model.save(final_path)
    save_meta(final_path, model.num_timesteps)
    venv.close()
    if eval_env is not None:
        eval_env.close()
    print(f"saved final ({model.num_timesteps} steps): {final_path} (+ .meta.json)")
    if not args.no_eval:
        print(f"BEST model (use this for eval): {best_path}  "
              f"[highest {args.eval_split} success_rate]")


if __name__ == "__main__":
    main()
