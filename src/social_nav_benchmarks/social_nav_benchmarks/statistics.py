"""Aggregate per-run rows into summary statistics.

Safety, efficiency, social and system are kept as SEPARATE dimensions: there is no single
blended "score", because trading a collision against a faster time-to-goal is a judgement
the numbers should not hide. Missing values (None) are dropped, not treated as zero.
"""

import math

# Each dimension lists the row fields it summarises.
DIMENSIONS = {
    "safety": ["min_human_distance_m"],
    "efficiency": ["time_to_goal_s", "path_length_m", "path_efficiency", "num_stops"],
    "social": ["avg_human_distance_m", "social_intrusion_ratio"],
    "system": ["compute_p95_ms", "compute_p99_ms", "deadline_misses",
               "actual_frequency_hz"],
}


def summarize(values):
    """count/mean/median/std/min/max/p95 for a list of numbers (None values dropped)."""
    nums = [float(v) for v in values if isinstance(v, (int, float))]
    if not nums:
        return {"count": 0, "mean": None, "median": None, "std": None,
                "min": None, "max": None, "p95": None}
    nums_sorted = sorted(nums)
    mean = sum(nums) / len(nums)
    var = sum((x - mean) ** 2 for x in nums) / len(nums)
    return {
        "count": len(nums),
        "mean": round(mean, 3),
        "median": round(_percentile(nums_sorted, 50), 3),
        "std": round(math.sqrt(var), 3),
        "min": round(min(nums), 3),
        "max": round(max(nums), 3),
        "p95": round(_percentile(nums_sorted, 95), 3),
    }


def _percentile(sorted_nums, pct):
    """Linear-interpolated percentile of an already-sorted list."""
    if len(sorted_nums) == 1:
        return sorted_nums[0]
    rank = (pct / 100.0) * (len(sorted_nums) - 1)
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return sorted_nums[low]
    frac = rank - low
    return sorted_nums[low] * (1 - frac) + sorted_nums[high] * frac


def _rate(rows, field):
    """Fraction of rows where `field` is truthy (missing counts as False)."""
    if not rows:
        return None
    return round(sum(1 for r in rows if r.get(field)) / len(rows), 3)


def aggregate(rows):
    """Aggregate a list of rows into overall rates plus per-dimension summaries."""
    rows = list(rows)
    result = {
        "n": len(rows),
        "success_rate": _rate(rows, "success"),
        "collision_rate": _rate(rows, "collision"),
        "dimensions": {},
    }
    for dim, fields in DIMENSIONS.items():
        result["dimensions"][dim] = {
            field: summarize([r.get(field) for r in rows]) for field in fields
        }
    return result


def by_scenario(rows):
    """Return {scenario_name: aggregate(rows_for_that_scenario)}."""
    groups = {}
    for row in rows:
        groups.setdefault(row.get("scenario", "?"), []).append(row)
    return {name: aggregate(sub) for name, sub in groups.items()}
