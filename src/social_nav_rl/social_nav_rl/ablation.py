"""Ablation runner: evaluate the same policy under different observation feature sets.

Each ablation is a set of ObsConfig overrides (which human features the policy may see). The
runner evaluates every ablation over the same scenarios/seeds and reports success + mean metrics
+ reward-hacking diagnostics, so "does prediction / TTC / social-zone info actually help?" is a
one-command experiment. The non-learned `straight` baseline runs offline (no training/GPU); an
RL policy would need a model trained at the matching observation size per ablation.
"""
import argparse
import json
import os
from datetime import datetime, timezone
from statistics import mean

from social_nav_rl import config as C
from social_nav_rl import diagnostics as D
from social_nav_rl.curriculum import SEED_SPLITS
from social_nav_rl.env import EpisodeConfig, SocialNavEnv
from social_nav_rl.evaluate import _make_policy, run_episode
from social_nav_rl.latency import LatencyMeter
from social_nav_rl.observation import ObsConfig

# name -> ObsConfig overrides on top of the base config (progressively adds human information).
ABLATIONS = {
    "no_humans": dict(n_humans=0),
    "position_only": dict(use_velocity=False, use_prediction=False, use_uncertainty=False,
                          use_ttc=False, use_social_zone=False, use_group=False),
    "with_velocity": dict(use_prediction=False, use_uncertainty=False, use_ttc=False,
                          use_social_zone=False, use_group=False),
    "with_prediction": dict(use_uncertainty=False, use_ttc=False, use_social_zone=False,
                            use_group=False),
    "with_uncertainty": dict(use_ttc=False, use_social_zone=False, use_group=False),
    "with_ttc": dict(use_social_zone=False, use_group=False),
    "with_social_zone": dict(use_group=False),
    "full": dict(),
}


def _obs_config(base: ObsConfig, overrides: dict) -> ObsConfig:
    fields = {f: getattr(base, f) for f in base.__dataclass_fields__}
    fields.update(overrides)
    return ObsConfig(**fields)


def run_ablation(name, base_obs, exp, args, policy):
    obs_cfg = _obs_config(base_obs, ABLATIONS[name])
    ep = EpisodeConfig(environment=args.environment, scenario=args.scenario,
                       difficulty=args.difficulty, max_steps=args.max_steps)
    env = SocialNavEnv(ep=ep, obs_cfg=obs_cfg, limits=exp["action"],
                       reward_cfg=exp["reward"], safety_cfg=exp["safety"])
    lat = LatencyMeter()
    lo, _ = SEED_SPLITS[args.split]
    results = [run_episode(env, policy, lo + i, lat)[0] for i in range(args.episodes)]
    succ = mean(1.0 if r["success"] else 0.0 for r in results)
    clr = [r["min_clearance"] for r in results if r["min_clearance"] is not None]
    return {"ablation": name, "obs_dim": env.observation_space.shape[0],
            "success_rate": round(succ, 3),
            "mean_min_clearance": round(mean(clr), 3) if clr else None,
            "mean_path_length": round(mean(r["path_length"] for r in results), 3),
            "diagnostics": D.summarize(results)["rates"],
            "inference_latency_ms": lat.summary()}


def main():
    ap = argparse.ArgumentParser(description="Observation-feature ablation study")
    ap.add_argument("--policy", default="straight", choices=["straight", "stop", "rl"])
    ap.add_argument("--model", default="")
    ap.add_argument("--environment", default="urban")
    ap.add_argument("--scenario", default="crossing")
    ap.add_argument("--difficulty", default="medium")
    ap.add_argument("--split", default="test", choices=list(SEED_SPLITS))
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--max-steps", dest="max_steps", type=int, default=400)
    ap.add_argument("--only", default="", help="comma-separated ablation names (default: all)")
    ap.add_argument("--config", default="")
    ap.add_argument("--out", default=os.path.expanduser("~/social_nav_rl_results"))
    ap.add_argument("--check", action="store_true", help="one ablation, 1 episode")
    args = ap.parse_args()

    exp = C.load_experiment(args.config or None)
    base_obs = exp["observation"]
    policy = _make_policy(args)
    names = [n for n in (args.only.split(",") if args.only else ABLATIONS) if n in ABLATIONS]

    if args.check:
        args.episodes = 1
        r = run_ablation(names[0], base_obs, exp, args, policy)
        print(f"[check] ablation={r['ablation']} obs_dim={r['obs_dim']} "
              f"success={r['success_rate']} -> OK.")
        return

    rows = [run_ablation(n, base_obs, exp, args, policy) for n in names]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(args.out, f"ablation_{args.policy}_{stamp}")
    os.makedirs(outdir, exist_ok=True)
    summary = {"policy": args.policy, "environment": args.environment, "scenario": args.scenario,
               "difficulty": args.difficulty, "episodes": args.episodes, "ablations": rows}
    with open(os.path.join(outdir, "ablation.json"), "w") as f:
        json.dump(summary, f, indent=2)
    for r in rows:
        print(f"  {r['ablation']:<16} obs_dim={r['obs_dim']:<3} success={r['success_rate']}")
    print(f"wrote {outdir}/ablation.json")


if __name__ == "__main__":
    main()
