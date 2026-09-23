"""Scenario feasibility oracle: does a collision-free path to the goal actually exist?

In a 1.7 m aisle, a head-on or sudden-stop pedestrian often leaves no way through in the time
budget, so 0% success there is the environment, not the planner. This lets the benchmark report
`success | solvable` (and treat correct waiting as success) instead of letting impossible cells
drag every method to zero.

It is a conservative LOWER bound: it searches a family of simple strategies — pick an aisle lane
(centre / offset either side) and a start delay (wait for a crosser to clear), then drive at max
speed to the goal — against the real static geometry and the deterministic pedestrian trajectories
(the same ones the policy faces). If ANY strategy is collision-free to the goal, the scenario is
feasible. A "not feasible" verdict means no simple strategy works; a cleverer path might, so we
label it `hard` rather than a hard "impossible".
"""
import math
from functools import lru_cache

from social_nav_tools.environments import generate_scenario
from social_nav_tools.pedestrian_model import Box, SpawnConfig, plan_pedestrians, pose_at_time

ROBOT_RADIUS = 0.45
HUMAN_RADIUS = 0.30
ARRIVAL = 0.6
MAX_LIN_VEL = 0.8


def _plan_people(cfg):
    p = cfg["pedestrian"]
    sc = SpawnConfig(
        num_humans=p["num_humans"], seed=p["seed"], spawn_region=tuple(p["spawn_region"]),
        behavior_profile=p["behavior_profile"],
        obstacles=tuple(Box(*p["obstacles"][i:i + 4]) for i in range(0, len(p["obstacles"]), 4)),
        robot_start=tuple(p["robot_start"]), speed=p.get("speed", 0.9),
        crossing_x=p.get("crossing_x", 5.5), crossing_north=p.get("crossing_north", 2.2),
        crossing_south=p.get("crossing_south", -3.0))
    return plan_pedestrians(sc)


def _hits_box(x, y, boxes, r):
    for (xmin, ymin, xmax, ymax) in boxes:
        if (xmin - r) <= x <= (xmax + r) and (ymin - r) <= y <= (ymax + r):
            return True
    return False


def _strategy_clear(sx, sy, gx, lane, delay, boxes, people, max_time, dt):
    """Wait `delay` s at the start, then drive straight down the aisle at y=`lane` to the goal x.
    Returns True if the goal is reached with no static or pedestrian collision within max_time."""
    contact = ROBOT_RADIUS + HUMAN_RADIUS
    t = 0.0
    while t <= max_time:
        if t < delay:
            x, y = sx, sy
        else:
            x = min(gx, sx + MAX_LIN_VEL * (t - delay))
            y = lane
        if _hits_box(x, y, boxes, ROBOT_RADIUS):
            return False
        for ped in people:
            px, py, *_ = pose_at_time(ped, t)
            if math.hypot(px - x, py - y) < contact:
                return False
        if math.hypot(gx - x, 0.0 - y) < ARRIVAL:      # goal is on the centreline (y=0)
            return True
        t += dt
    return False


@lru_cache(maxsize=8192)
def is_feasible(environment, scenario, difficulty, seed, max_time=30.0, dt=0.1):
    """True if some simple lane+delay strategy reaches the goal collision-free (see module doc).
    Cached: depends only on (env, scenario, difficulty, seed), not on the policy under test."""
    cfg = generate_scenario(environment, scenario, difficulty, seed)
    boxes = [tuple(b) for b in cfg.get("world_obstacles", [])]
    people = _plan_people(cfg)
    sx, sy, _ = cfg["robot_start"]
    gx, _gy = cfg["goal"]
    lanes = (0.0, 0.4, -0.4, 0.7, -0.7)                 # centre, then offsets within a ~1.7 m aisle
    delays = [0.5 * k for k in range(0, 17)]            # 0 .. 8 s of waiting for a crosser to clear
    for lane in lanes:
        if _hits_box(sx, lane, boxes, ROBOT_RADIUS):    # this lane is blocked at the start
            continue
        for delay in delays:
            if _strategy_clear(sx, sy, gx, lane, delay, boxes, people, max_time, dt):
                return True
    return False
