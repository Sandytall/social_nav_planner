"""social-nav-benchmark: run scenarios headless, record metrics, write results
(MASTER_PROMPT §29, §38, §39). Do NOT cherry-pick; run all trials, keep every result.

  social-nav-benchmark --scenario blocker_on_path --runs 5
  social-nav-benchmark --scenario all --runs 20
  SOCIAL_NAV_PARAMS=/abs/baseline.yaml social-nav-benchmark --scenario all --runs 20  # ablation
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from datetime import datetime, timezone

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from social_nav_benchmarks import metrics as M
from social_nav_benchmarks.scenarios import get_scenario, scenario_names

try:
    from social_nav_msgs.msg import HumanArray, PlannerMetrics
except Exception:  # noqa: BLE001
    HumanArray = PlannerMetrics = None

KILL_PATTERN = ("controller_server|planner_server|behavior_server|bt_navigator|"
                "lifecycle_manager|static_transform_publisher|spawn_entity|"
                "robot_state_publisher|human_publisher|human_markers")


def _sh(cmd):
    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _kill_all():
    _sh("pkill -9 -f gzserver"); _sh("pkill -9 -f gzclient")
    _sh(f"pkill -9 -f '{KILL_PATTERN}'")


class Recorder(Node):
    def __init__(self):
        super().__init__("benchmark_recorder")
        self.odom = []
        self.humans = []
        self.metrics = None
        self._t0 = time.time()
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        if HumanArray is not None:
            self.create_subscription(HumanArray, "/social_nav/humans", self._humans, 10)
        if PlannerMetrics is not None:
            self.create_subscription(
                PlannerMetrics, "/social_nav/debug/metrics", self._metrics, 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _odom(self, m):
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.odom.append((time.time() - self._t0, m.pose.pose.position.x,
                          m.pose.pose.position.y, v))

    def _humans(self, m):
        self.humans.append((time.time() - self._t0,
                            [(h.pose.position.x, h.pose.position.y) for h in m.humans]))

    def _metrics(self, m):
        self.metrics = dict(p50_ms=m.p50_ms, p95_ms=m.p95_ms, p99_ms=m.p99_ms,
                            max_ms=m.max_ms, deadline_misses=m.deadline_misses,
                            actual_frequency_hz=m.actual_frequency_hz)


def _launch_env():
    env = dict(os.environ)
    env["GAZEBO_MODEL_PATH"] = "/usr/share/gazebo-11/models:" + env.get("GAZEBO_MODEL_PATH", "")
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    return env


def run_one(scenario_name, run_idx, goal, timeout_s, logdir):
    scn = get_scenario(scenario_name)
    _kill_all()
    time.sleep(3)
    env = _launch_env()
    logf = open(os.path.join(logdir, f"{scenario_name}_{run_idx:03d}.log"), "w")

    demo = subprocess.Popen(
        ["ros2", "launch", "social_nav_bringup", "demo.launch.py", "gui:=false"],
        stdout=logf, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    hp = None
    if scn["humans"]:
        humans_arg = "[" + ",".join(f"'{h}'" for h in scn["humans"]) + "]"
        hp = subprocess.Popen(
            ["ros2", "run", "social_nav_tools", "human_publisher", "--ros-args",
             "-p", f"humans:={humans_arg}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
            start_new_session=True)

    # Wait for the full stack to activate (bt_navigator last).
    active = False
    for _ in range(90):
        logf.flush()
        try:
            with open(logf.name) as fr:
                if "Managed nodes are active" in fr.read():
                    active = True
                    break
        except OSError:
            pass
        time.sleep(1)
    time.sleep(2)

    rec = Recorder()
    record = {"scenario": scenario_name, "run": run_idx, "status": "NO_ACTIVE"}
    if active and rec.client.wait_for_server(timeout_sec=40):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = float(goal[0]), float(goal[1])
        g.pose.pose.orientation.w = 1.0
        fut = rec.client.send_goal_async(g)
        rclpy.spin_until_future_complete(rec, fut, timeout_sec=10)
        gh = fut.result()
        if gh and gh.accepted:
            rf = gh.get_result_async()
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                rclpy.spin_once(rec, timeout_sec=0.1)
                if rf.done():
                    st = rf.result().status
                    record["status"] = {
                        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                        GoalStatus.STATUS_ABORTED: "ABORTED",
                        GoalStatus.STATUS_CANCELED: "CANCELED"}.get(st, f"CODE_{st}")
                    break
            else:
                record["status"] = "TIMEOUT"
        else:
            record["status"] = "REJECTED"

    record["odom"] = rec.odom
    record["humans"] = rec.humans
    record["planner"] = rec.metrics
    rec.destroy_node()

    # Teardown.
    for p in (demo, hp):
        if p is None:
            continue
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(2)
    _kill_all()
    logf.close()
    time.sleep(2)
    return record


def main():
    ap = argparse.ArgumentParser(description="SocialNav benchmark runner (§29)")
    ap.add_argument("--scenario", default="all")
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--output", default=os.path.expanduser("~/social_nav_results"))
    ap.add_argument("--goal", default="2.5,2.5")
    args = ap.parse_args()

    goal = tuple(float(x) for x in args.goal.split(","))
    names = scenario_names() if args.scenario == "all" else [args.scenario]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(args.output, stamp)
    os.makedirs(outdir, exist_ok=True)
    logdir = os.path.join(outdir, "logs")
    os.makedirs(logdir, exist_ok=True)

    # Reproducibility metadata (§39).
    meta = {
        "timestamp_utc": stamp,
        "params_file": os.environ.get("SOCIAL_NAV_PARAMS", "default (shipped nav2_params.yaml)"),
        "scenarios": names, "runs_each": args.runs, "goal": list(goal),
        "git_commit": subprocess.getoutput(
            "git -C ~/Social_planner/social_nav_ws rev-parse --short HEAD 2>/dev/null"),
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
    }
    with open(os.path.join(outdir, "experiment_metadata.json"), "w") as f:
        json.dump(meta, f, indent=2)

    rclpy.init()
    rows = []
    total = len(names) * args.runs
    n = 0
    for name in names:
        for run in range(1, args.runs + 1):
            n += 1
            print(f"[{n}/{total}] scenario={name} run={run} ...", flush=True)
            record = run_one(name, run, goal, get_scenario(name)["timeout"], logdir)
            row = M.compute(record, goal)
            with open(os.path.join(outdir, f"{name}_{run:03d}.json"), "w") as f:
                json.dump(row, f, indent=2)
            rows.append(row)
            print(f"      -> success={row.get('success')} status={row.get('status')} "
                  f"min_clear={row.get('min_human_distance_m')} "
                  f"t_goal={row.get('time_to_goal_s')}", flush=True)
    rclpy.shutdown()

    # summary.csv
    with open(os.path.join(outdir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=M.SUMMARY_FIELDS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    _write_report(outdir, rows, names, meta)
    print(f"\nDone. Results in {outdir}\n  summary.csv, report.md, per-run JSON, logs/")


def _write_report(outdir, rows, names, meta):
    lines = ["# SocialNav Benchmark Report", "",
             f"- Time (UTC): {meta['timestamp_utc']}",
             f"- Params: `{meta['params_file']}`",
             f"- Git: `{meta['git_commit']}`  ROS: `{meta['ros_distro']}`",
             f"- Runs per scenario: {meta['runs_each']}", "",
             "| Scenario | Success | Collisions | min clear (m) | t_goal (s) | p95 ms |",
             "|---|---|---|---|---|---|"]
    def avg(vals):
        return f"{sum(vals) / len(vals):.2f}" if vals else "-"

    for name in names:
        sub = [r for r in rows if r.get("scenario") == name]
        if not sub:
            continue
        succ = sum(1 for r in sub if r.get("success"))
        coll = sum(1 for r in sub if r.get("collision"))
        clears = [r["min_human_distance_m"] for r in sub if r.get("min_human_distance_m") is not None]
        tgoals = [r["time_to_goal_s"] for r in sub if r.get("time_to_goal_s") is not None]
        p95 = [r["compute_p95_ms"] for r in sub if r.get("compute_p95_ms") is not None]
        min_clear = f"{min(clears):.2f}" if clears else "-"
        lines.append(
            f"| {name} | {succ}/{len(sub)} | {coll} | {min_clear} | "
            f"{avg(tgoals)} | {avg(p95)} |")
    lines += ["", "Full per-run numbers in `summary.csv`. Raw data in the per-run `*.json`.",
              "", "**§62:** every number here is measured from an actual run; none fabricated."]
    with open(os.path.join(outdir, "report.md"), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
