"""Shared fixtures: synthetic records/rows so the offline analysis is tested without ROS."""

import pytest

GOAL = (2.5, 2.5)


@pytest.fixture
def goal():
    return GOAL


@pytest.fixture
def crossing_record():
    """A run that reaches the goal but passes very close to one person mid-way."""
    return {
        "scenario": "crossing", "run": 1, "status": "SUCCEEDED",
        "odom": [
            (0.0, -3.5, -3.5, 0.0),
            (1.0, -3.0, -3.0, 0.5),
            (2.0, -2.0, -2.0, 0.5),
            (3.0, 0.0, 0.0, 0.4),
            (4.0, 2.5, 2.5, 0.0),
        ],
        "humans": [
            (1.0, [(0.0, 0.0)]),
            (2.0, [(-1.5, -1.5)]),
            (3.0, [(0.2, 0.2)]),
            (4.0, [(1.0, 1.0)]),
        ],
        "planner": {"p50_ms": 0.08, "p95_ms": 0.12, "p99_ms": 0.2, "max_ms": 0.5,
                    "deadline_misses": 0, "actual_frequency_hz": 20.0},
    }


@pytest.fixture
def sample_rows():
    """Three computed rows: two successes and one timeout that barely moved."""
    return [
        {"scenario": "empty", "run": 1, "success": True, "status": "SUCCEEDED",
         "time_to_goal_s": 18.0, "path_length_m": 7.5, "path_efficiency": 0.9,
         "avg_velocity_mps": 0.4, "min_human_distance_m": None,
         "avg_human_distance_m": None, "collision": False,
         "social_intrusion_ratio": 0.0, "num_stops": 0, "oscillations": 1,
         "compute_p95_ms": 0.12, "compute_p99_ms": 0.2, "deadline_misses": 0,
         "actual_frequency_hz": 20.0},
        {"scenario": "crossing", "run": 1, "success": True, "status": "SUCCEEDED",
         "time_to_goal_s": 22.0, "path_length_m": 8.1, "path_efficiency": 0.82,
         "avg_velocity_mps": 0.36, "min_human_distance_m": 0.9,
         "avg_human_distance_m": 1.6, "collision": False,
         "social_intrusion_ratio": 0.3, "num_stops": 1, "oscillations": 2,
         "compute_p95_ms": 0.15, "compute_p99_ms": 0.25, "deadline_misses": 0,
         "actual_frequency_hz": 20.0},
        {"scenario": "crossing", "run": 2, "success": False, "status": "TIMEOUT",
         "time_to_goal_s": None, "path_length_m": 0.2, "path_efficiency": None,
         "avg_velocity_mps": 0.02, "min_human_distance_m": 0.7,
         "avg_human_distance_m": 0.9, "collision": False,
         "social_intrusion_ratio": 0.8, "num_stops": 6, "oscillations": 9,
         "compute_p95_ms": 0.14, "compute_p99_ms": 0.22, "deadline_misses": 1,
         "actual_frequency_hz": 19.5},
    ]
