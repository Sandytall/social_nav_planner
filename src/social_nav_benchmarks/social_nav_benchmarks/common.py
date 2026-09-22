"""Shared helpers for the offline analysis layer.

Everything here operates on a `record` dict, the same shape the benchmark runner
captures for a single run (and the same shape a rosbag loader would produce), so the
analysis is sim-agnostic and works on real-robot data too:

    record = {
        "scenario": str,
        "run": int,
        "status": str,                         # SUCCEEDED / ABORTED / TIMEOUT / ...
        "odom":   [(t, x, y, v), ...],         # time-ordered, seconds from run start
        "humans": [(t, [(hx, hy), ...]), ...], # people positions per observation
        "planner": {"p95_ms": ..., ...} | None,
        "goal":   (gx, gy),                    # optional; passed explicitly otherwise
    }

These modules deliberately import only the standard library so they run without ROS.
"""

import math


def nearest_pose(odom, t):
    """Return the (x, y, v) odom sample closest in time to `t`, or None if empty."""
    best = None
    best_dt = float("inf")
    for (ot, x, y, v) in odom:
        dt = abs(ot - t)
        if dt < best_dt:
            best_dt = dt
            best = (x, y, v)
    return best


def min_clearance(rx, ry, people):
    """Distance from the robot to the nearest person, or None if nobody is present."""
    if not people:
        return None
    return min(math.hypot(rx - hx, ry - hy) for (hx, hy) in people)


def clearance_series(record):
    """Yield (t, robot_xy, robot_v, min_dist) for each human observation in the run.

    `min_dist` is None on frames with no people. The robot pose is taken from the odom
    sample nearest in time, so this tolerates odom and human streams at different rates.
    """
    odom = record.get("odom") or []
    for (t, people) in record.get("humans") or []:
        pose = nearest_pose(odom, t)
        if pose is None:
            continue
        rx, ry, rv = pose
        yield t, (rx, ry), rv, min_clearance(rx, ry, people)
