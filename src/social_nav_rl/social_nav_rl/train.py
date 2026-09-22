"""PPO training CLI for RL social navigation (accelerated MockBackend by default).

  social-nav-rl-train --environment factory --curriculum --seed 42 --timesteps 500000
  social-nav-rl-train --environment warehouse --scenario crossing --difficulty hard --check

`--check` builds the env + PPO policy and runs one reset/predict/step WITHOUT training, so the
whole pipeline can be validated in a second. Actual training (`.learn`) is the long run - it
uses your torch/GPU - and is left for you to start.
"""
import argparse
import os
import random

from social_nav_rl import config as C
from social_nav_rl.checkpoint import CheckpointMeta
from social_nav_rl.curriculum import Curriculum
from social_nav_rl.env import EpisodeConfig, SocialNavEnv


def build_env(args):
    exp = C.load_experiment(args.config or None)
    common = dict(obs_cfg=exp["observation"], limits=exp["action"],
                  reward_cfg=exp["reward"], safety_cfg=exp["safety"])
    if args.curriculum:
        cur = Curriculum(split="train", max_steps=args.max_steps)
        rng = random.Random(args.seed)
        env = SocialNavEnv(episode_sampler=lambda: cur.sample_episode(rng), **common)
        return env, cur, exp
    ep = EpisodeConfig(environment=args.environment, scenario=args.scenario,
                       difficulty=args.difficulty, max_steps=args.max_steps)
    return SocialNavEnv(ep=ep, **common), None, exp


def main():
    ap = argparse.ArgumentParser(description="Train PPO RL social navigation")
    ap.add_argument("--algorithm", default="ppo", choices=["ppo"])
    ap.add_argument("--environment", default="urban")
    ap.add_argument("--scenario", default="normal")
    ap.add_argument("--difficulty", default="medium")
    ap.add_argument("--curriculum", action="store_true", help="train across the 10-level curriculum")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--timesteps", type=int, default=500_000)
    ap.add_argument("--max-steps", dest="max_steps", type=int, default=600)
    ap.add_argument("--config", default="", help="config dir (default: package config/)")
    ap.add_argument("--out", default=os.path.expanduser("~/social_nav_rl_checkpoints"))
    ap.add_argument("--device", default="cpu", choices=["cpu", "cuda", "auto"],
                    help="SB3 recommends CPU for MLP PPO; use cuda only for large nets")
    ap.add_argument("--check", action="store_true", help="build + 1 step, no training")
    args = ap.parse_args()

    env, cur, exp = build_env(args)
    try:
        from stable_baselines3 import PPO
    except ImportError:
        raise SystemExit("stable-baselines3 not installed. Run:\n"
                         "  pip install 'stable-baselines3>=2.2' gymnasium")

    model = PPO("MlpPolicy", env, seed=args.seed, device=args.device,
                verbose=0 if args.check else 1)

    if args.check:
        obs, _ = env.reset(seed=args.seed)
        action, _ = model.predict(obs, deterministic=True)
        env.step(action)
        print(f"[check] obs_dim={obs.shape[0]} action_dim={env.action_space.shape[0]} "
              f"curriculum={'on' if cur else 'off'} -> pipeline OK (no training run).")
        return

    callback = _curriculum_callback(cur) if cur else None
    model.learn(total_timesteps=args.timesteps, callback=callback)
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, f"ppo_{args.environment}_{args.seed}.zip")
    model.save(path)
    CheckpointMeta(algorithm="ppo", environment=args.environment, split="train",
                   training_steps=args.timesteps, seed=args.seed,
                   reward_config=dict(exp["reward"].weights),
                   obs_config=vars(exp["observation"]),
                   curriculum={"enabled": bool(cur)}).save(path)
    print(f"saved {path} (+ .meta.json)")


def _curriculum_callback(cur):
    from stable_baselines3.common.callbacks import BaseCallback

    class CurriculumCallback(BaseCallback):
        def _on_step(self) -> bool:
            for done, info in zip(self.locals.get("dones", []), self.locals.get("infos", [])):
                if done and "events" in info:
                    if cur.record(bool(info["events"].get("reached"))):
                        self.logger.record("curriculum/level", cur.level)
            return True

    return CurriculumCallback()


if __name__ == "__main__":
    main()
