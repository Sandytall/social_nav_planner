"""Write a per-run trajectory table to CSV.

One row per odom sample: robot pose/velocity plus the nearest-person distance at that
time (matched to the closest human observation). This is the raw time series a report or
an external tool plots; it is ROS-level and sim-agnostic.
"""

import csv

from social_nav_benchmarks import common

FIELDS = ["t", "robot_x", "robot_y", "robot_v", "num_humans", "min_human_dist"]


def rows(record):
    """Yield trajectory rows (dicts) for one run."""
    humans = record.get("humans") or []
    for (t, x, y, v) in record.get("odom") or []:
        people = _people_at(humans, t)
        dist = common.min_clearance(x, y, people)
        yield {
            "t": round(t, 3),
            "robot_x": round(x, 3),
            "robot_y": round(y, 3),
            "robot_v": round(v, 3),
            "num_humans": len(people),
            "min_human_dist": round(dist, 3) if dist is not None else "",
        }


def _people_at(humans, t):
    """People from the human observation closest in time to `t` (empty if none)."""
    best = None
    best_dt = float("inf")
    for (ht, people) in humans:
        dt = abs(ht - t)
        if dt < best_dt:
            best_dt = dt
            best = people
    return best or []


def write_trajectory_csv(record, path):
    """Write the trajectory table for one run to `path`."""
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows(record):
            writer.writerow(row)
    return path
