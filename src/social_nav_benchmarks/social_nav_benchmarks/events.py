"""Derive a structured event log from a recorded run.

Rather than instrumenting the C++ controller (which would couple this to the planner),
events are reconstructed offline from the recorded odometry, human positions and final
status. That keeps the layer additive and lets it run on any recording, including a
real-robot rosbag, not just Gazebo output.

Each event is a plain dict:
    {"t": float, "type": str, "severity": "info"|"warning"|"critical",
     "robot": [x, y], "robot_v": float, "min_human_dist": float|None, "detail": str}

Only events the data actually supports are emitted; nothing is inferred beyond the
recorded signals.
"""

import json

from social_nav_benchmarks import common
from social_nav_benchmarks.metrics import COLLISION_DIST, COMFORT_DIST, STOP_SPEED

INFO = "info"
WARNING = "warning"
CRITICAL = "critical"

# How long the robot must stay below STOP_SPEED before a stop is called "prolonged".
PROLONGED_STOP_S = 5.0

SUCCESS_STATUSES = {"SUCCEEDED"}


def _event(t, etype, severity, robot, robot_v, min_dist, detail=""):
    return {
        "t": round(float(t), 2),
        "type": etype,
        "severity": severity,
        "robot": [round(robot[0], 3), round(robot[1], 3)] if robot else None,
        "robot_v": round(float(robot_v), 3) if robot_v is not None else None,
        "min_human_dist": round(min_dist, 2) if min_dist is not None else None,
        "detail": detail,
    }


def detect_events(record, goal=None):
    """Return a time-ordered list of events for one run."""
    events = []
    odom = record.get("odom") or []
    scenario = record.get("scenario", "?")
    status = record.get("status", "UNKNOWN")

    if not odom:
        events.append(_event(0.0, "RUN_STARTED", INFO, None, None, None,
                             f"scenario={scenario}"))
        events.append(_event(0.0, "NO_ODOM", CRITICAL, None, None, None,
                             "no odometry recorded"))
        return events

    (t0, x0, y0, v0) = odom[0]
    (tf, xf, yf, _) = odom[-1]
    events.append(_event(t0, "RUN_STARTED", INFO, (x0, y0), v0, None,
                         f"scenario={scenario}"))

    events += _proximity_events(record)
    events += _stop_events(odom)

    # Terminal outcome.
    if status in SUCCESS_STATUSES:
        events.append(_event(tf, "GOAL_REACHED", INFO, (xf, yf), 0.0, None,
                             f"status={status}"))
    else:
        events.append(_event(tf, "RUN_FAILED", CRITICAL, (xf, yf), 0.0, None,
                             f"status={status}"))

    events.sort(key=lambda e: e["t"])
    return events


def _proximity_events(record):
    """Emit one event per episode of entering the comfort / collision zones.

    An episode is a contiguous stretch below the threshold; re-entering after leaving
    starts a new one, so a person passing twice yields two events (no per-frame spam).
    """
    out = []
    in_comfort = False
    in_collision = False
    for (t, robot, rv, dist) in common.clearance_series(record):
        if dist is None:
            in_comfort = in_collision = False
            continue
        if dist < COLLISION_DIST:
            if not in_collision:
                out.append(_event(t, "NEAR_COLLISION", CRITICAL, robot, rv, dist,
                                 f"clearance {dist:.2f} m < {COLLISION_DIST} m"))
                in_collision = True
        else:
            in_collision = False
        if dist < COMFORT_DIST:
            if not in_comfort:
                out.append(_event(t, "SOCIAL_INTRUSION", WARNING, robot, rv, dist,
                                 f"clearance {dist:.2f} m < {COMFORT_DIST} m"))
                in_comfort = True
        else:
            in_comfort = False
    return out


def _stop_events(odom):
    """Emit ROBOT_STOPPED / PROLONGED_STOP / ROBOT_RESUMED around low-speed episodes."""
    out = []
    stopped = False
    stop_start = None
    prolonged_flagged = False
    for (t, x, y, v) in odom:
        if v < STOP_SPEED:
            if not stopped:
                stopped = True
                stop_start = t
                prolonged_flagged = False
                out.append(_event(t, "ROBOT_STOPPED", INFO, (x, y), v, None, ""))
            elif not prolonged_flagged and (t - stop_start) >= PROLONGED_STOP_S:
                prolonged_flagged = True
                out.append(_event(t, "PROLONGED_STOP", WARNING, (x, y), v, None,
                                 f"stopped for {t - stop_start:.1f} s"))
        else:
            if stopped:
                out.append(_event(t, "ROBOT_RESUMED", INFO, (x, y), v, None, ""))
            stopped = False
    return out


def write_events(events, path):
    """Write the event list to `path` as a JSON array."""
    with open(path, "w") as f:
        json.dump(events, f, indent=2)
    return path
