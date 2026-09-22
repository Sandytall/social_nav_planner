"""Offline analysis of a benchmark results directory.

Consumes the artifacts a run leaves behind and produces the derived layer without touching
Gazebo, so the same command works on a rerun, a copied results folder, or (once a loader is
added) a real-robot rosbag:

  * per raw record  -> <base>_events.json, <base>_trajectory.csv
  * across all rows -> stats.json, failures.json, report.html

    social-nav-analyze --input ~/social_nav_results/<stamp>
"""

import argparse
import glob
import json
import os

from social_nav_benchmarks import (
    events, failure_classifier, report_html, statistics, trajectory,
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


def main():
    ap = argparse.ArgumentParser(description="Offline analysis of a results directory")
    ap.add_argument("--input", required=True, help="benchmark results directory")
    args = ap.parse_args()
    summary = analyze_dir(args.input)
    print(f"Analyzed {summary['rows']} runs, {summary['records_analyzed']} records.")
    print(f"Wrote stats.json, failures.json, {os.path.basename(summary['report'])} "
          f"in {args.input}")


if __name__ == "__main__":
    main()
