"""Per-planner aggregation for baseline comparison.

Groups computed rows by their ``planner`` tag and summarises each group along the same
safety / efficiency / social / system dimensions used elsewhere. Pure standard-library so
it runs and unit-tests without ROS. Missing metrics stay absent (n/a), never zero-filled.
"""

import csv

from social_nav_benchmarks import statistics

# Columns written to comparison.csv, in order.
COMPARISON_FIELDS = [
    "planner", "runs", "success_rate", "collision_rate",
    "mean_time_to_goal_s", "worst_clearance_m", "mean_min_clearance_m",
    "mean_intrusion_ratio", "mean_p95_ms",
]


def planners_present(rows):
    """Distinct planner tags across rows, order-stable. Untagged rows are ignored here."""
    seen = []
    for r in rows:
        p = r.get("planner")
        if p is not None and p not in seen:
            seen.append(p)
    return seen


def group_by_planner(rows):
    """Return {planner_tag: [rows]} preserving row order within each group."""
    groups = {}
    for r in rows:
        groups.setdefault(r.get("planner"), []).append(r)
    return groups


def _cell(summary, key="mean"):
    return None if summary is None else summary.get(key)


def comparison_rows(rows):
    """One aggregate dict per planner tag (see COMPARISON_FIELDS)."""
    out = []
    for planner, sub in group_by_planner(rows).items():
        agg = statistics.aggregate(sub)
        safety = agg["dimensions"]["safety"]["min_human_distance_m"]
        eff = agg["dimensions"]["efficiency"]["time_to_goal_s"]
        soc = agg["dimensions"]["social"]["social_intrusion_ratio"]
        sysd = agg["dimensions"]["system"]["compute_p95_ms"]
        out.append({
            "planner": planner,
            "runs": agg["n"],
            "success_rate": agg["success_rate"],
            "collision_rate": agg["collision_rate"],
            "mean_time_to_goal_s": _cell(eff),
            "worst_clearance_m": _cell(safety, "min"),
            "mean_min_clearance_m": _cell(safety, "mean"),
            "mean_intrusion_ratio": _cell(soc),
            "mean_p95_ms": _cell(sysd),
        })
    return out


def write_comparison_csv(rows, path):
    """Write the per-planner comparison table to ``path`` and return it."""
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COMPARISON_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in comparison_rows(rows):
            writer.writerow(row)
    return path
