"""Compute per-run benchmark metrics from recorded time series."""
import math

COLLISION_DIST = 0.5     # robot centre this close to a person = collision
COMFORT_DIST = 1.2       # inside this of a person = social-zone intrusion
STOP_SPEED = 0.05        # below this = "stopped"


def compute(record, goal, arrival_radius=0.6):
    """`record` fields (lists, time-ordered):
       odom: [(t, x, y, v)], humans: [(t, [(hx,hy), ...])],
       planner: last PlannerMetrics dict or None, status: str, t_start, t_end."""
    odom = record["odom"]
    out = {
        "scenario": record.get("scenario"),
        "run": record.get("run"),
        "status": record.get("status", "UNKNOWN"),
    }
    if not odom:
        out["success"] = False
        out["note"] = "no odom recorded"
        return out

    (t0, x0, y0, _) = odom[0]
    (tf, xf, yf, _) = odom[-1]

    reached = math.hypot(xf - goal[0], yf - goal[1]) < arrival_radius
    out["success"] = bool(reached)
    out["time_to_goal_s"] = round(tf - t0, 2) if reached else None

    # Path length + efficiency.
    plen = sum(math.hypot(odom[i][1] - odom[i - 1][1], odom[i][2] - odom[i - 1][2])
               for i in range(1, len(odom)))
    straight = math.hypot(goal[0] - x0, goal[1] - y0)
    out["path_length_m"] = round(plen, 2)
    out["path_efficiency"] = round(straight / plen, 3) if plen > 1e-3 else None

    speeds = [v for (_, _, _, v) in odom]
    out["avg_velocity_mps"] = round(sum(speeds) / len(speeds), 3)
    stops = sum(1 for i in range(1, len(speeds))
                if speeds[i] < STOP_SPEED <= speeds[i - 1])
    out["num_stops"] = stops

    # Oscillations: sign changes of heading rate along the path.
    yaws = [math.atan2(odom[i][2] - odom[i - 1][2], odom[i][1] - odom[i - 1][1])
            for i in range(1, len(odom))
            if math.hypot(odom[i][1] - odom[i - 1][1], odom[i][2] - odom[i - 1][2]) > 0.02]
    osc = 0
    for i in range(2, len(yaws)):
        d1 = math.atan2(math.sin(yaws[i - 1] - yaws[i - 2]), math.cos(yaws[i - 1] - yaws[i - 2]))
        d2 = math.atan2(math.sin(yaws[i] - yaws[i - 1]), math.cos(yaws[i] - yaws[i - 1]))
        if d1 * d2 < 0 and abs(d1) > 0.05 and abs(d2) > 0.05:
            osc += 1
    out["oscillations"] = osc

    # Human clearance over the run.
    clearances = []
    for (t, people) in record["humans"]:
        # nearest odom sample in time
        rx, ry = _nearest_pose(odom, t)
        if rx is None or not people:
            continue
        clearances.append(min(math.hypot(rx - hx, ry - hy) for (hx, hy) in people))
    if clearances:
        out["min_human_distance_m"] = round(min(clearances), 2)
        out["avg_human_distance_m"] = round(sum(clearances) / len(clearances), 2)
        out["collision"] = bool(min(clearances) < COLLISION_DIST)
        out["social_intrusion_ratio"] = round(
            sum(1 for c in clearances if c < COMFORT_DIST) / len(clearances), 3)
    else:
        out["min_human_distance_m"] = None
        out["collision"] = False
        out["social_intrusion_ratio"] = 0.0

    # Planner compute latency, taken from the last PlannerMetrics.
    pm = record.get("planner")
    if pm:
        out["compute_p50_ms"] = round(pm.get("p50_ms", 0.0), 2)
        out["compute_p95_ms"] = round(pm.get("p95_ms", 0.0), 2)
        out["compute_p99_ms"] = round(pm.get("p99_ms", 0.0), 2)
        out["compute_max_ms"] = round(pm.get("max_ms", 0.0), 2)
        out["deadline_misses"] = pm.get("deadline_misses", 0)
        out["actual_frequency_hz"] = round(pm.get("actual_frequency_hz", 0.0), 1)
    return out


def _nearest_pose(odom, t):
    best = None
    bestdt = 1e9
    for (ot, x, y, _) in odom:
        dt = abs(ot - t)
        if dt < bestdt:
            bestdt, best = dt, (x, y)
    return best if best else (None, None)


SUMMARY_FIELDS = [
    "planner",
    "scenario", "run", "success", "status", "time_to_goal_s", "path_length_m",
    "path_efficiency", "avg_velocity_mps", "min_human_distance_m", "avg_human_distance_m",
    "collision", "social_intrusion_ratio", "num_stops", "oscillations",
    "compute_p50_ms", "compute_p95_ms", "compute_p99_ms", "deadline_misses",
    "actual_frequency_hz",
]
