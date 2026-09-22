"""Classify the outcome of a run into a single failure label with evidence.

Works from a computed metrics `row` (see metrics.compute); the raw record is optional and
only used for extra evidence. The classifier reports what the recorded signals support and
never asserts a cause the data cannot back up: when no strong signal is present the label
is INCOMPLETE with low confidence rather than a guessed root cause.
"""

# Non-fatal quality flags (can accompany a success).
OSCILLATION_FLAG = 6      # heading sign-changes along the path
STOPS_FLAG = 4            # discrete stop episodes
INTRUSION_FLAG = 0.5      # fraction of observations inside the comfort zone
STUCK_PATH_M = 0.5        # moved less than this and did not reach the goal

HIGH, MEDIUM, LOW = "high", "medium", "low"

# Terminal statuses that name their own failure directly.
_STATUS_LABEL = {
    "TIMEOUT": ("GOAL_TIMEOUT", HIGH),
    "ABORTED": ("NAV_ABORTED", HIGH),
    "REJECTED": ("GOAL_REJECTED", HIGH),
    "CANCELED": ("GOAL_CANCELED", HIGH),
    "NO_ACTIVE": ("STACK_NOT_ACTIVE", HIGH),
    "NO_PROGRESS": ("NO_PROGRESS", HIGH),
}


def classify(row, record=None):
    """Return {"label", "confidence", "evidence", "flags"} for one run."""
    evidence = {}
    flags = _quality_flags(row)

    # Safety first: a measured collision outranks any status.
    min_dist = row.get("min_human_distance_m")
    if row.get("collision"):
        evidence["min_human_distance_m"] = min_dist
        return _result("COLLISION", HIGH, evidence, flags)

    if row.get("success"):
        evidence["time_to_goal_s"] = row.get("time_to_goal_s")
        return _result("NONE", HIGH, evidence, flags)

    # Not a success: prefer an explicit terminal status.
    status = row.get("status", "UNKNOWN")
    if status in _STATUS_LABEL:
        label, conf = _STATUS_LABEL[status]
        evidence["status"] = status
        path_len = row.get("path_length_m")
        if path_len is not None and path_len < STUCK_PATH_M:
            # Timed out/aborted having barely moved -> the robot was stuck.
            evidence["path_length_m"] = path_len
            return _result("STUCK", MEDIUM, evidence, flags)
        return _result(label, conf, evidence, flags)

    # No explicit status: infer only what the motion supports.
    path_len = row.get("path_length_m")
    if path_len is not None and path_len < STUCK_PATH_M:
        evidence["path_length_m"] = path_len
        return _result("STUCK", MEDIUM, evidence, flags)

    evidence["status"] = status
    evidence["path_length_m"] = path_len
    return _result("INCOMPLETE", LOW, evidence, flags)


def _quality_flags(row):
    flags = []
    if (row.get("oscillations") or 0) >= OSCILLATION_FLAG:
        flags.append("OSCILLATION")
    if (row.get("num_stops") or 0) >= STOPS_FLAG:
        flags.append("EXCESSIVE_STOPS")
    if (row.get("social_intrusion_ratio") or 0.0) >= INTRUSION_FLAG:
        flags.append("SOCIAL_INTRUSION")
    return flags


def _result(label, confidence, evidence, flags):
    return {"label": label, "confidence": confidence,
            "evidence": evidence, "flags": flags}


def breakdown(rows):
    """Count runs per failure label across a list of rows."""
    counts = {}
    for row in rows:
        label = classify(row)["label"]
        counts[label] = counts.get(label, 0) + 1
    return counts
