"""Evaluate a policy over held-out scenarios/seeds; record metrics + failure replays.

All policies (RL, hybrid, or the non-learned baselines here) run through the SAME env /
scenarios / seeds - RL gets no privileged info. Nav2 and the classical SocialNav baselines run
in Gazebo via the existing social_nav_benchmarks `--planner` path; this harness covers the RL
env. Per-episode metrics, inference latency (P50/P95/P99/max) and failure replays are recorded.

  social-nav-rl-eval --policy straight --environment warehouse --scenario crossing --episodes 20
  social-nav-rl-eval --policy rl --model ~/ckpts/ppo.zip --environment hospital --split test
"""
import argparse
import json
import math
import os
from datetime import datetime, timezone

from social_nav_rl import config as C
from social_nav_rl.curriculum import SEED_SPLITS
from social_nav_rl.env import EpisodeConfig, SocialNavEnv
from social_nav_rl.latency import LatencyMeter
from social_nav_rl.policies import ConstantPolicy, StraightToGoalPolicy
from social_nav_rl.replay import FailureReplay


def _make_policy(args):
    if args.policy == "straight":
        return StraightToGoalPolicy()
    if args.policy == "stop":
        return ConstantPolicy(-1.0, 0.0)
    if args.policy == "rl":
        if not args.model:
            raise SystemExit("--policy rl requires --model <checkpoint.zip>")
        from social_nav_rl.policies import load_rl_policy
        return load_rl_policy(args.model)
    raise SystemExit(f"unknown policy {args.policy!r}")


def run_episode(env, policy, seed, lat, comfort=1.2):
    obs, _ = env.reset(seed=seed)
    r = env.backend.robot
    replay = FailureReplay(env.ep.environment, env.ep.scenario, env.ep.difficulty,
                           seed, list(env.backend.goal))
    m = {"min_clearance": math.inf, "min_ttc": math.inf, "path_length": 0.0,
         "num_stops": 0, "oscillations": 0, "interventions": 0, "social_violations": 0}
    prev = (r["x"], r["y"])
    prev_v, prev_w = 0.0, 0.0
    steps, events = 0, {"reached": False, "collision": False, "human_collision": False}
    done = False
    while not done:
        with lat.measure():
            action = policy.act(obs)
        obs, _, term, trunc, info = env.step(action)
        events = info["events"]
        rb = env.backend.robot
        m["path_length"] += math.hypot(rb["x"] - prev[0], rb["y"] - prev[1])
        prev = (rb["x"], rb["y"])
        if events["min_clearance"] is not None:
            m["min_clearance"] = min(m["min_clearance"], events["min_clearance"])
            if events["min_clearance"] < comfort:
                m["social_violations"] += 1
        m["min_ttc"] = min(m["min_ttc"], events["min_ttc"])
        m["interventions"] += info["interventions"]
        if abs(rb["v"]) < 0.05 <= abs(prev_v):
            m["num_stops"] += 1
        if rb["w"] * prev_w < 0 and abs(rb["w"]) > 0.1 and abs(prev_w) > 0.1:
            m["oscillations"] += 1
        prev_v, prev_w = rb["v"], rb["w"]
        replay.add(t=steps, robot=[rb["x"], rb["y"], rb["yaw"]], action=[float(a) for a in action],
                   safety_kind=info["safety"], events=events)
        steps += 1
        done = term or trunc

    success = bool(events["reached"]) and not events["collision"] and not events["human_collision"]
    outcome = ("reached" if success else "collision" if (events["collision"] or
               events["human_collision"]) else "timeout")
    result = {"seed": seed, "success": success, "outcome": outcome,
              "collision": bool(events["collision"]),
              "human_collision": bool(events["human_collision"]),
              "navigation_time_s": round(steps * env.limits.dt, 2),
              "min_clearance": None if math.isinf(m["min_clearance"]) else round(m["min_clearance"], 3),
              "min_ttc": None if m["min_ttc"] >= 1e3 else round(m["min_ttc"], 3),
              "path_length": round(m["path_length"], 3), "num_stops": m["num_stops"],
              "oscillations": m["oscillations"], "social_violations": m["social_violations"],
              "safety_interventions": m["interventions"]}
    return result, (replay if not success else None), outcome


def main():
    ap = argparse.ArgumentParser(description="Evaluate an RL / baseline policy")
    ap.add_argument("--policy", default="straight", choices=["straight", "stop", "rl"])
    ap.add_argument("--model", default="", help="checkpoint for --policy rl")
    ap.add_argument("--environment", default="urban")
    ap.add_argument("--scenario", default="normal")
    ap.add_argument("--difficulty", default="medium")
    ap.add_argument("--split", default="test", choices=list(SEED_SPLITS))
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--max-steps", dest="max_steps", type=int, default=600)
    ap.add_argument("--config", default="")
    ap.add_argument("--out", default=os.path.expanduser("~/social_nav_rl_results"))
    ap.add_argument("--check", action="store_true", help="one short episode, no files written")
    args = ap.parse_args()

    exp = C.load_experiment(args.config or None)
    ep = EpisodeConfig(environment=args.environment, scenario=args.scenario,
                       difficulty=args.difficulty,
                       max_steps=10 if args.check else args.max_steps)
    env = SocialNavEnv(ep=ep, obs_cfg=exp["observation"], limits=exp["action"],
                       reward_cfg=exp["reward"], safety_cfg=exp["safety"])
    policy = _make_policy(args)
    lat = LatencyMeter()
    lo, _ = SEED_SPLITS[args.split]

    if args.check:
        res, _, _ = run_episode(env, policy, lo, lat)
        print(f"[check] ran 1 episode ({args.policy}) outcome={res['outcome']} "
              f"latency_p95={lat.summary()['p95']} ms -> OK.")
        return

    results = []
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(args.out, f"{args.policy}_{args.environment}_{stamp}")
    os.makedirs(outdir, exist_ok=True)
    for i in range(args.episodes):
        res, replay, outcome = run_episode(env, policy, lo + i, lat)
        results.append(res)
        if replay is not None:
            replay.finalize(outcome)
            replay.save(os.path.join(outdir, f"failure_{i:03d}_seed{lo + i}.json"))

    succ = sum(1 for r in results if r["success"]) / max(1, len(results))
    summary = {"policy": args.policy, "environment": args.environment,
               "scenario": args.scenario, "difficulty": args.difficulty, "split": args.split,
               "episodes": len(results), "success_rate": round(succ, 3),
               "inference_latency_ms": lat.summary(), "runs": results, "timestamp_utc": stamp}
    with open(os.path.join(outdir, "results.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(f"success_rate={succ:.2f} over {len(results)} eps; latency p95="
          f"{lat.summary()['p95']} ms; wrote {outdir}/results.json")


if __name__ == "__main__":
    main()
