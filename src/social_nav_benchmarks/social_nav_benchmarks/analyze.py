"""Offline analysis of a benchmark results directory or a rosbag2 recording.

Consumes the artifacts a run leaves behind and produces the derived layer without touching
Gazebo, so the same command works on a rerun, a copied results folder, or a real-robot
rosbag2 recording:

  * per raw record  -> <base>_events.json, <base>_trajectory.csv
  * across all rows -> stats.json, failures.json, report.html

    social-nav-analyze --input ~/social_nav_results/<stamp>
    social-nav-analyze --bag /path/to/rosbag2_dir [--goal X,Y] [--scenario name]
"""

import argparse
import glob
import json
import os
from datetime import datetime, timezone

from social_nav_benchmarks import (
    bag_ingest, events, failure_classifier, metrics, report_html, statistics, trajectory,
)


def analyze_records(results_dir):
    """Write events + trajectory for every raw record dump found. Returns the count."""
    done = 0
    for path in sorted(glob.glob(os.path.join(results_dir, "*_record.json"))):
        try:
            with open(path) as f:
                record = json.load(f)
        except (OSError, ValueError):
            continue
        base = path[: -len("_record.json")]
        events.write_events(events.detect_events(record), base + "_events.json")
        trajectory.write_trajectory_csv(record, base + "_trajectory.csv")
        done += 1
    return done


def analyze_dir(results_dir):
    """Run the full offline analysis over a results directory; return a summary dict."""
    rows = report_html.load_rows(results_dir)
    meta = report_html.load_meta(results_dir)
    records_done = analyze_records(results_dir)

    stats = {"overall": statistics.aggregate(rows),
             "by_scenario": statistics.by_scenario(rows)}
    with open(os.path.join(results_dir, "stats.json"), "w") as f:
        json.dump(stats, f, indent=2)

    failures = {
        "breakdown": failure_classifier.breakdown(rows),
        "per_run": [
            {"scenario": r.get("scenario"), "run": r.get("run"),
             **failure_classifier.classify(r)}
            for r in rows
        ],
    }
    with open(os.path.join(results_dir, "failures.json"), "w") as f:
        json.dump(failures, f, indent=2)

    report = report_html.generate(results_dir, rows=rows, meta=meta)
    return {"rows": len(rows), "records_analyzed": records_done,
            "stats": "stats.json", "failures": "failures.json", "report": report}


def analyze_bag(bag_dir, goal, scenario="bag", run=1, status="UNKNOWN", output=None):
    """Ingest a rosbag2 recording into a record, write the run artifacts, then analyze.

    Success is derived geometrically (final pose within the arrival radius of the goal),
    exactly as for a live run; a bag carries no nav goal outcome, so ``status`` stays
    UNKNOWN unless the caller supplies one. Returns the analyze_dir summary.
    """
    record = bag_ingest.load_bag(bag_dir, scenario=scenario, run=run, status=status)
    outdir = output or (os.path.normpath(bag_dir) + "_analysis")
    os.makedirs(outdir, exist_ok=True)

    row = metrics.compute(record, goal)
    row["failure"] = failure_classifier.classify(row, record)
    base = os.path.join(outdir, f"{scenario}_{run:03d}")
    with open(base + ".json", "w") as f:
        json.dump(row, f, indent=2)
    with open(base + "_record.json", "w") as f:
        json.dump(record, f)
    events.write_events(events.detect_events(record, goal), base + "_events.json")
    trajectory.write_trajectory_csv(record, base + "_trajectory.csv")

    meta = {
        "timestamp_utc": datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S"),
        "source": "rosbag2", "bag": os.path.abspath(bag_dir),
        "scenarios": [scenario], "runs_each": 1, "goal": list(goal),
    }
    with open(os.path.join(outdir, "experiment_metadata.json"), "w") as f:
        json.dump(meta, f, indent=2)

    summary = analyze_dir(outdir)
    summary["output_dir"] = outdir
    return summary


def main():
    ap = argparse.ArgumentParser(
        description="Offline analysis of a results directory or a rosbag2 recording")
    ap.add_argument("--input", help="benchmark results directory")
    ap.add_argument("--bag", help="rosbag2 directory to ingest and analyze")
    ap.add_argument("--goal", default="2.5,2.5",
                    help="goal x,y for success/geometry (bag mode); default 2.5,2.5")
    ap.add_argument("--scenario", default="bag", help="scenario label for a bag run")
    ap.add_argument("--run", type=int, default=1, help="run index for a bag run")
    ap.add_argument("--output", help="output dir for --bag mode (default: <bag>_analysis)")
    args = ap.parse_args()

    if bool(args.input) == bool(args.bag):
        ap.error("provide exactly one of --input or --bag")

    if args.bag:
        goal = tuple(float(x) for x in args.goal.split(","))
        summary = analyze_bag(args.bag, goal, scenario=args.scenario, run=args.run,
                              output=args.output)
        where = summary["output_dir"]
    else:
        summary = analyze_dir(args.input)
        where = args.input
    print(f"Analyzed {summary['rows']} runs, {summary['records_analyzed']} records.")
    print(f"Wrote stats.json, failures.json, {os.path.basename(summary['report'])} "
          f"in {where}")


if __name__ == "__main__":
    main()
