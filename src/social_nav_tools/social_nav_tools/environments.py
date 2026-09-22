"""Environment + scenario registry and generator for the SocialNav simulation suite.

Pure Python (no ROS/Gazebo imports) so the whole scenario-generation logic is unit-testable.
Each environment is DATA: its Gazebo world file, the walkable spawn region, obstacle keep-out
boxes (for spawn safety), the robot start and named goals, crossing geometry, and whether it
hosts non-human dynamic obstacles (forklifts/carts/AMRs). ``generate_scenario(...)`` turns
(environment, scenario, difficulty, seed) into the concrete pedestrian-manager parameters +
robot goal + dynamic-obstacle/robot counts + sensor profile. The planner therefore never sees
anything environment-specific: the environment provides the challenge, the planner solves it.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

from social_nav_tools.pedestrian_model import (
    BLOCKER, CROSSING, GROUP, HEAD_ON, MERGE, MIXED, SAME_DIRECTION, STOP_GO, TURNING, WALKING,
)

# ----------------------------------------------------------------------------------------
# Scenario types (spec) -> a base pedestrian behaviour profile + a base human count (medium).
# These are meaningful across environments; the environment supplies the geometry that makes
# a bottleneck a bottleneck or an occluder an occluder.
# ----------------------------------------------------------------------------------------
EMPTY = "empty"
LOW_DENSITY = "low_density"
NORMAL = "normal"
HIGH_DENSITY = "high_density"
CROSSING_S = "crossing"
APPROACHING = "approaching"
SAME_DIRECTION_S = "same_direction"
GROUP_S = "group"
BOTTLENECK = "bottleneck"
OCCLUSION = "occlusion"
SUDDEN_STOP = "sudden_stop"
DIRECTION_CHANGE = "direction_change"
GOAL_BLOCKED = "goal_blocked"
MULTI_HUMAN = "multi_human"
DENSE_CROWD = "dense_crowd"
DYNAMIC_OBSTACLE = "dynamic_obstacle"
MULTI_ROBOT = "multi_robot"

# scenario -> (behaviour profile, base human count at MEDIUM difficulty)
SCENARIO_BASE: Dict[str, Tuple[str, int]] = {
    EMPTY: (WALKING, 0),
    LOW_DENSITY: (MIXED, 2),
    NORMAL: (MIXED, 4),
    HIGH_DENSITY: (MIXED, 8),
    DENSE_CROWD: (MIXED, 14),
    CROSSING_S: (CROSSING, 2),
    APPROACHING: (HEAD_ON, 2),
    SAME_DIRECTION_S: (SAME_DIRECTION, 2),
    GROUP_S: (GROUP, 4),
    BOTTLENECK: (MIXED, 6),
    OCCLUSION: (CROSSING, 3),
    SUDDEN_STOP: (STOP_GO, 1),
    DIRECTION_CHANGE: (TURNING, 2),
    GOAL_BLOCKED: (BLOCKER, 1),
    MULTI_HUMAN: (MIXED, 6),
    DYNAMIC_OBSTACLE: (WALKING, 2),
    MULTI_ROBOT: (MIXED, 3),
}
SCENARIO_TYPES = tuple(SCENARIO_BASE.keys())

# difficulty -> knobs. human_mult scales the count; speed sets pedestrian pace; sensor picks
# the noise profile; dyn adds baseline non-human dynamic obstacles.
DIFFICULTY: Dict[str, Dict[str, float]] = {
    "easy":   {"human_mult": 0.5, "speed": 0.7, "sensor": "clean",     "dyn": 0},
    "medium": {"human_mult": 1.0, "speed": 0.9, "sensor": "realistic", "dyn": 0},
    "hard":   {"human_mult": 1.5, "speed": 1.1, "sensor": "realistic", "dyn": 1},
    "stress": {"human_mult": 2.0, "speed": 1.3, "sensor": "stress",    "dyn": 2},
}
DIFFICULTIES = tuple(DIFFICULTY.keys())


@dataclass(frozen=True)
class Environment:
    """Static description of one simulation environment."""

    name: str
    world: str                                   # file in social_nav_sim/worlds/
    spawn_region: Tuple[float, float, float, float]   # xmin, xmax, ymin, ymax (map frame)
    robot_start: Tuple[float, float, float]      # x, y, yaw
    goals: Dict[str, Tuple[float, float]]        # named goals (kept within ~10 m of start)
    default_goal: str
    obstacles: Sequence[float] = ()              # flat [xmin,ymin,xmax,ymax, ...] keep-outs
    crossing_x: float = 5.5                      # lane where "crossing" pedestrians walk
    crossing_north: float = 2.2                  # crossing route north end (env-specific)
    crossing_south: float = -3.0                 # crossing route south end
    dynamic_obstacles: bool = False              # hosts forklifts/carts/AMRs as obstacles
    scenarios: Tuple[str, ...] = SCENARIO_TYPES  # scenario types meaningful here

    def goal_xy(self, name: str = "") -> Tuple[float, float]:
        return self.goals.get(name or self.default_goal, self.goals[self.default_goal])


# ----------------------------------------------------------------------------------------
# Registry. urban + factory reference the existing worlds (their dedicated launches still
# work unchanged); warehouse is the first world built for the generic env_demo launch.
# ----------------------------------------------------------------------------------------
ENVIRONMENTS: Dict[str, Environment] = {
    "urban": Environment(
        name="urban", world="urban.world",
        spawn_region=(1.0, 12.0, -1.0, 1.4), robot_start=(0.0, 0.0, 0.0),
        goals={"far": (9.0, 0.0), "near": (6.0, 0.0)}, default_goal="far",
        obstacles=(-1.0, 1.6, 3.5, 4.6, 7.5, 1.6, 13.0, 4.6),
        crossing_x=5.5, dynamic_obstacles=True),
    "factory": Environment(
        name="factory", world="factory.world",
        spawn_region=(1.5, 9.0, -1.6, 1.6), robot_start=(0.0, 0.0, 0.0),
        goals={"east": (8.5, 0.0), "aisle": (6.0, 0.0)}, default_goal="east",
        obstacles=(-3.7, 3.4, -1.5, 6.2, 9.5, 1.6, 10.7, 3.2, 9.5, -2.9, 10.7, -1.5),
        crossing_x=4.6, dynamic_obstacles=True),
    "warehouse": Environment(
        name="warehouse", world="warehouse.world",
        spawn_region=(1.5, 9.0, -3.2, 3.2), robot_start=(0.0, 0.0, 0.0),
        goals={"dock": (9.0, 0.0), "aisle_end": (9.0, 1.0)}, default_goal="dock",
        # Pallet-rack rows (y bands) flanking the main cross-aisle at y~0.
        obstacles=(1.0, 1.6, 9.5, 3.4, 1.0, -3.4, 9.5, -1.6),
        # Crossers stay within the central aisle (racks flank it), not the urban wide crossing.
        crossing_x=4.5, crossing_north=1.5, crossing_south=-1.5, dynamic_obstacles=True),
    "hospital": Environment(
        name="hospital", world="hospital.world",
        spawn_region=(1.0, 9.0, -1.4, 1.4), robot_start=(0.0, 0.0, 0.0),
        goals={"corridor_end": (9.0, 0.0), "midway": (5.0, 0.0)}, default_goal="corridor_end",
        # Corridor-intruding furniture (nurse desk, reception, waiting bench).
        obstacles=(4.25, 0.95, 5.75, 1.55, 0.9, -1.55, 2.1, -0.95, 7.0, -1.5, 8.0, -1.1),
        crossing_x=3.0, crossing_north=1.4, crossing_south=-1.4, dynamic_obstacles=True),
    "office": Environment(
        name="office", world="office.world",
        spawn_region=(1.0, 9.0, -1.4, 1.4), robot_start=(0.0, 0.0, 0.0),
        goals={"far": (9.0, 0.0), "atrium": (6.0, 0.0)}, default_goal="far",
        obstacles=(0.8, -1.5, 2.4, -0.9),  # reception desk (edge of corridor)
        crossing_x=3.0, crossing_north=1.4, crossing_south=-1.4, dynamic_obstacles=False),
    "plaza": Environment(
        name="plaza", world="plaza.world",
        spawn_region=(1.0, 9.0, -3.5, 3.5), robot_start=(0.0, 0.0, 0.0),
        goals={"far_side": (9.0, 0.0), "corner": (8.0, 2.0)}, default_goal="far_side",
        obstacles=(5.75, 3.0, 9.25, 6.0, 1.0, -5.9, 4.0, -3.3,     # buildings
                   2.7, 1.3, 3.3, 1.9, 5.7, -2.0, 6.3, -1.4, 8.2, 1.9, 8.8, 2.5,  # trees
                   3.8, 0.7, 5.2, 1.1, 5.0, -1.4, 6.0, -0.4),      # bench, planter
        crossing_x=4.5, crossing_north=3.0, crossing_south=-3.0, dynamic_obstacles=False),
}
ENVIRONMENT_NAMES = tuple(ENVIRONMENTS.keys())


def _round_humans(base: int, mult: float) -> int:
    if base <= 0:
        return 0
    return max(1, round(base * mult))


def generate_scenario(environment: str, scenario: str = NORMAL, difficulty: str = "medium",
                      seed: int = 42, human_count: int = None,
                      sensor_profile: str = None) -> dict:
    """Deterministically turn (environment, scenario, difficulty, seed) into a runnable config.

    Returns a dict with the world, robot start/goal, sensor profile, robot + dynamic-obstacle
    counts, and the pedestrian-manager parameter block. Raises ValueError on unknown inputs.
    Same inputs => same output (the seed flows into the pedestrian planner).
    """
    if environment not in ENVIRONMENTS:
        raise ValueError(f"unknown environment {environment!r}; use {ENVIRONMENT_NAMES}")
    if scenario not in SCENARIO_BASE:
        raise ValueError(f"unknown scenario {scenario!r}; use {SCENARIO_TYPES}")
    if difficulty not in DIFFICULTY:
        raise ValueError(f"unknown difficulty {difficulty!r}; use {DIFFICULTIES}")

    env = ENVIRONMENTS[environment]
    diff = DIFFICULTY[difficulty]
    profile, base_humans = SCENARIO_BASE[scenario]

    humans = human_count if human_count is not None else _round_humans(base_humans, diff["human_mult"])
    sensor = sensor_profile or diff["sensor"]

    num_robots = 3 if scenario == MULTI_ROBOT else 1
    dyn = int(diff["dyn"])
    if scenario == DYNAMIC_OBSTACLE:
        dyn = max(dyn, 2)
    if not env.dynamic_obstacles:
        dyn = 0

    pedestrian = {
        "num_humans": humans,
        "behavior_profile": profile,
        "seed": seed,
        "spawn_region": list(env.spawn_region),
        "obstacles": list(env.obstacles),
        "robot_start": [env.robot_start[0], env.robot_start[1]],
        "crossing_x": env.crossing_x,
        "crossing_north": env.crossing_north,
        "crossing_south": env.crossing_south,
        "speed": diff["speed"],
    }
    return {
        "environment": env.name,
        "world": env.world,
        "scenario": scenario,
        "difficulty": difficulty,
        "seed": seed,
        "robot_start": list(env.robot_start),
        "goal": list(env.goal_xy()),
        "sensor_profile": sensor,
        "num_robots": num_robots,
        "num_dynamic_obstacles": dyn,
        "pedestrian": pedestrian,
    }
