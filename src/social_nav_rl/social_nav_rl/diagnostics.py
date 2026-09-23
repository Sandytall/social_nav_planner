"""Reward-hacking / degenerate-behaviour diagnostics over evaluation results.

The RL policy can "succeed" at the reward while behaving badly (stopping forever, crawling,
huge detours, oscillating, exploiting the episode timeout). These pure checks flag such runs
from the recorded per-episode metrics so a training run can be audited instead of trusted.
"""
DEFAULT_THRESHOLDS = {
    "stuck_path_m": 0.5,      # moved less than this -> refuses to move
    "slow_speed_mps": 0.1,    # average speed below this -> crawling
    "detour_factor": 2.0,     # path_length / straight_dist above this -> excessive detour
    "osc_count": 15,          # left/right switches above this -> oscillating
    "min_progress_m": 1.0,    # timed out having moved less than this -> no progress
}


def diagnose(result: dict, thresholds: dict = None) -> dict:
    """Return {"flags": {name: bool}, "triggered": [names]} for one episode result."""
    t = {**DEFAULT_THRESHOLDS, **(thresholds or {})}
    success = bool(result.get("success"))
    path = float(result.get("path_length", 0.0))
    nav_t = float(result.get("navigation_time_s", 0.0))
    avg_speed = result.get("avg_speed", (path / nav_t if nav_t > 0 else 0.0))
    straight = result.get("straight_dist")
    outcome = result.get("outcome", "")

    flags = {
        "refuses_to_move": (path < t["stuck_path_m"]) and not success,
        "crawling": (avg_speed < t["slow_speed_mps"]) and not success,
        "excessive_detour": bool(straight) and path > t["detour_factor"] * straight,
        "oscillating": int(result.get("oscillations", 0)) >= t["osc_count"],
        "timeout_no_progress": outcome == "timeout" and path < t["min_progress_m"],
    }
    return {"flags": flags, "triggered": [k for k, v in flags.items() if v]}


def summarize(results, thresholds: dict = None) -> dict:
    """Fraction of episodes triggering each diagnostic across a list of results."""
    n = max(1, len(results))
    counts = {k: 0 for k in DEFAULT_DIAG_NAMES}
    for r in results:
        for k in diagnose(r, thresholds)["triggered"]:
            counts[k] += 1
    return {"episodes": len(results), "rates": {k: round(c / n, 3) for k, c in counts.items()}}


DEFAULT_DIAG_NAMES = ["refuses_to_move", "crawling", "excessive_detour",
                      "oscillating", "timeout_no_progress"]
