#!/usr/bin/env python3
"""Benchmark the RL social-nav policy against the classical Social-Force planner and a straight-line
baseline on the accelerated env.

Every policy runs the SAME environment / scenario / difficulty / seeds (held-out test split) with
no safety supervisor, so it is a matched, fair comparison. Writes episode-level JSONL, an aggregate
CSV, and a summary JSON. Numbers in the report come from these files - nothing is hand-entered.

  # main comparison on the trained environment
  python3 scripts/rl_benchmark.py --environment factory --policies straight,sfm,rl \
      --scenarios normal,crossing,approaching,sudden_stop,direction_change,dense_crowd \
      --difficulties easy,medium --episodes 20 --model <keeper.zip>

  # cross-environment generalization for the RL policy (trained on factory)
  python3 scripts/rl_benchmark.py --environment warehouse --policies rl,straight \
      --scenarios normal,crossing --difficulties medium --episodes 20 --model <keeper.zip>
"""
import argparse
import csv
import json
import os
import statistics as st
from datetime import datetime, timezone

from social_nav_rl import config as C
from social_nav_rl.curriculum import SEED_SPLITS
from social_nav_rl.env import EpisodeConfig, SocialNavEnv
from social_nav_rl.evaluate import run_episode
from social_nav_rl.latency import LatencyMeter
from social_nav_rl.policies import SocialForcePolicy, StraightToGoalPolicy, load_rl_policy

DEFAULT_MODEL = os.path.expanduser(
    "~/social_nav_rl_checkpoints/BEST_factory_medium_multi/ppo_factory_42_best.zip")


def _aggregate(rows):
    n = len(rows)
    mc = [r["min_clearance"] for r in rows if r["min_clearance"] is not None]
    ttc = [r["min_ttc"] for r in rows if r["min_ttc"] is not None]
    return {
        "episodes": n,
        "success": round(sum(r["success"] for r in rows) / n, 3),
        "collision": round(sum(r["collision"] for r in rows) / n, 3),
        "human_collision": round(sum(r["human_collision"] for r in rows) / n, 3),
        "timeout": round(sum(1 for r in rows if r["outcome"] == "timeout") / n, 3),
        "min_clearance_m": round(st.mean(mc), 3) if mc else None,
        "min_ttc_s": round(st.mean(ttc), 3) if ttc else None,
        "avg_speed": round(st.mean(r["avg_speed"] for r in rows), 3),
        "nav_time_s": round(st.mean(r["navigation_time_s"] for r in rows), 2),
        "path_length_m": round(st.mean(r["path_length"] for r in rows), 2),
        "oscillations": round(st.mean(r["oscillations"] for r in rows), 2),
    }


def main():
    ap = argparse.ArgumentParser(description="RL vs classical vs straight benchmark (mock substrate)")
    ap.add_argument("--environment", default="factory")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--policies", default="straight,sfm,rl")
    ap.add_argument("--scenarios",
                    default="normal,crossing,approaching,sudden_stop,direction_change,dense_crowd")
    ap.add_argument("--difficulties", default="easy,medium")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--split", default="test", choices=list(SEED_SPLITS))
    ap.add_argument("--safe-filter", dest="safe_filter", action="store_true",
                    help="apply the safety supervisor as a deploy-time backstop to ALL policies "
                         "(fairer deployment comparison); default off = raw policy actions")
    ap.add_argument("--out", default=os.path.expanduser("~/social_nav_rl_results/benchmark"))
    args = ap.parse_args()

    exp = C.load_experiment()
    lo, _ = SEED_SPLITS[args.split]
    policies = [p.strip() for p in args.policies.split(",") if p.strip()]
    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    diffs = [d.strip() for d in args.difficulties.split(",") if d.strip()]

    rl_policy = load_rl_policy(args.model) if "rl" in policies else None
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(args.out, f"{args.environment}_{stamp}")
    os.makedirs(outdir, exist_ok=True)
    raw = open(os.path.join(outdir, "episodes.jsonl"), "w")
    agg_rows = []

    def make_policy(name):
        if name == "straight":
            return StraightToGoalPolicy()
        if name == "sfm":
            return SocialForcePolicy(n_humans=exp["observation"].n_humans)
        if name == "rl":
            return rl_policy
        raise SystemExit(f"unknown policy {name!r}")

    print(f"benchmark env={args.environment} split={args.split} episodes={args.episodes}")
    print(f"{'policy':8} {'diff':6} {'scenario':16} {'succ':>5} {'coll':>5} {'clr(m)':>6} {'spd':>5}")
    for pol_name in policies:
        for diff in diffs:
            for sc in scenarios:
                ep = EpisodeConfig(environment=args.environment, scenario=sc, difficulty=diff)
                env = SocialNavEnv(ep=ep, obs_cfg=exp["observation"], limits=exp["action"],
                                   reward_cfg=exp["reward"], safety_cfg=exp["safety"],
                                   safe_filter=args.safe_filter)
                policy = make_policy(pol_name)
                lat = LatencyMeter()
                rows = []
                for i in range(args.episodes):
                    res, _, _ = run_episode(env, policy, lo + i, lat)
                    res.update(policy=pol_name, environment=args.environment,
                               scenario=sc, difficulty=diff)
                    rows.append(res)
                    raw.write(json.dumps(res) + "\n")
                agg = _aggregate(rows)
                agg.update(policy=pol_name, environment=args.environment, scenario=sc,
                           difficulty=diff, latency_p95_ms=lat.summary()["p95"])
                agg_rows.append(agg)
                print(f"{pol_name:8} {diff:6} {sc:16} {agg['success']:5.2f} "
                      f"{agg['collision'] + agg['human_collision']:5.2f} "
                      f"{('%.2f' % agg['min_clearance_m']) if agg['min_clearance_m'] else '--':>6} "
                      f"{agg['avg_speed']:5.2f}")
    raw.close()

    cols = ["policy", "environment", "difficulty", "scenario", "episodes", "success", "collision",
            "human_collision", "timeout", "min_clearance_m", "min_ttc_s", "avg_speed", "nav_time_s",
            "path_length_m", "oscillations", "latency_p95_ms"]
    with open(os.path.join(outdir, "aggregate.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows([{k: r.get(k) for k in cols} for r in agg_rows])
    with open(os.path.join(outdir, "summary.json"), "w") as f:
        json.dump({"timestamp_utc": stamp, "environment": args.environment, "model": args.model,
                   "split": args.split, "episodes_per_cell": args.episodes, "policies": policies,
                   "scenarios": scenarios, "difficulties": diffs, "rows": agg_rows}, f, indent=2)
    print(f"\nwrote {outdir}/  (episodes.jsonl, aggregate.csv, summary.json)")


if __name__ == "__main__":
    main()
