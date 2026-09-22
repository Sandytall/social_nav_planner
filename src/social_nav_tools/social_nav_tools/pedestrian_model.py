"""Deterministic pedestrian planning: where people spawn and how they move.

This module is deliberately free of ROS and Gazebo imports so the spawn/route logic can
be unit tested and reasoned about on its own. The node in pedestrian_manager.py feeds these
plans to both Gazebo (visible motion) and /social_nav/humans (planner input), so a single
trajectory drives both and they can never drift apart.

Coordinates are the shared map/world frame in metres (the urban launch spawns the robot at
the world origin so map == odom == Gazebo world; see urban_demo.launch.py).
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

WALKING = "walking"
CROSSING = "crossing"
MIXED = "mixed"
GROUP = "group"
PROFILES = (WALKING, CROSSING, MIXED, GROUP)


@dataclass(frozen=True)
class Box:
    """Axis-aligned keep-out footprint (metres). Half-open coverage test with a margin."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            self.xmin - margin <= x <= self.xmax + margin
            and self.ymin - margin <= y <= self.ymax + margin
        )


@dataclass
class Pedestrian:
    """A planned pedestrian: identity, behaviour, and a poly-line route walked back and forth."""

    id: int
    name: str
    behavior: str
    waypoints: List[Tuple[float, float]]
    speed: float
    group_id: int = -1

    @property
    def start(self) -> Tuple[float, float]:
        return self.waypoints[0]


@dataclass
class SpawnConfig:
    """Inputs that fully determine a plan. Same config + same seed => same people."""

    num_humans: int
    seed: int
    spawn_region: Tuple[float, float, float, float]  # xmin, xmax, ymin, ymax
    behavior_profile: str
    min_separation: float = 1.0
    obstacles: Sequence[Box] = field(default_factory=tuple)
    robot_start: Tuple[float, float] = (0.0, 0.0)
    robot_keepout: float = 1.5
    speed: float = 0.9
    # Group behaviour (behavior_profile == "group"): members per group and their spacing.
    group_size: int = 3
    group_spacing: float = 0.7
    # Geometry the routes are built around; defaults match urban.world.
    crossing_x: float = 5.5
    sidewalk_north: float = 1.4
    crossing_north: float = 2.2
    crossing_south: float = -3.0
    walk_xmin: float = 1.0
    walk_xmax: float = 12.0


def _behavior_for(profile: str, index: int) -> str:
    if profile == WALKING:
        return WALKING
    if profile == CROSSING:
        return CROSSING
    # MIXED: even indices cross (guarantees at least one crosser), odd walk along.
    return CROSSING if index % 2 == 0 else WALKING


def _sample_spawn(cfg: SpawnConfig, rng: random.Random, placed: List[Tuple[float, float]],
                  extra_margin: float = 0.0):
    """Rejection-sample a spawn point that is inside the region, clear of obstacles and the
    robot, and at least min_separation from every already-placed pedestrian. `extra_margin`
    widens every clearance (used to fit a whole group's width). Returns None if no valid point
    is found within the attempt budget (caller treats that as a hard error)."""
    xmin, xmax, ymin, ymax = cfg.spawn_region
    for _ in range(200):
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        if any(o.contains(x, y, margin=cfg.min_separation * 0.5 + extra_margin)
               for o in cfg.obstacles):
            continue
        if math.hypot(x - cfg.robot_start[0], y - cfg.robot_start[1]) < \
                cfg.robot_keepout + extra_margin:
            continue
        if any(math.hypot(x - px, y - py) < cfg.min_separation + extra_margin
               for px, py in placed):
            continue
        return (x, y)
    return None


def _walk_route(cfg: SpawnConfig, spawn: Tuple[float, float]):
    """A walking route starts AT the (validated) spawn and paces to the far end of the sidewalk
    at the spawn's y, so the route start is exactly the spawn that was checked for safety."""
    sx, sy = spawn
    mid = 0.5 * (cfg.walk_xmin + cfg.walk_xmax)
    target_x = cfg.walk_xmax if sx <= mid else cfg.walk_xmin
    return [(sx, sy), (target_x, sy)]


def _place_crossing_x(cfg: SpawnConfig, placed: List[Tuple[float, float]]) -> float:
    """Pick an x lane for a crossing pedestrian near the crossing. Lanes are staggered around
    crossing_x in min_separation steps so multiple crossers do not overlap, and each candidate
    is checked clear of obstacles. Raises ValueError if no lane fits."""
    step = max(cfg.min_separation, 0.5)
    offsets = [0.0]
    for k in range(1, 12):
        offsets.extend((k * step, -k * step))
    for off in offsets:
        x = cfg.crossing_x + off
        spawn = (x, cfg.crossing_north)
        if any(o.contains(*spawn, margin=cfg.min_separation * 0.5) for o in cfg.obstacles):
            continue
        if any(math.hypot(x - px, cfg.crossing_north - py) < cfg.min_separation
               for px, py in placed):
            continue
        return x
    raise ValueError(
        "could not place a crossing pedestrian clear of buildings/others near "
        f"crossing_x={cfg.crossing_x}; widen the crossing gap or reduce num_humans")


def _group_sizes(n: int, size: int) -> List[int]:
    """Split n people into groups of at most `size` (clamped to [2, 5]). A trailing lone person
    is merged into the previous group so there is never a 'group' of one."""
    size = max(2, min(5, size))
    sizes: List[int] = []
    remaining = n
    while remaining > 0:
        take = min(size, remaining)
        sizes.append(take)
        remaining -= take
    if len(sizes) >= 2 and sizes[-1] == 1:
        lone = sizes.pop()
        sizes[-1] += lone
    return sizes


def _plan_groups(cfg: SpawnConfig, rng: random.Random) -> List[Pedestrian]:
    """Plan cohesive walking groups. Each group shares one validated centre route; members are
    spread laterally by group_spacing (perpendicular to travel) and tagged with a common
    group_id so they move together. Raises ValueError if a group cannot be placed safely."""
    placed: List[Tuple[float, float]] = []
    people: List[Pedestrian] = []
    idx = 0
    for gid, size in enumerate(_group_sizes(cfg.num_humans, cfg.group_size)):
        half_width = 0.5 * (size - 1) * cfg.group_spacing
        center = _sample_spawn(cfg, rng, placed, extra_margin=half_width)
        if center is None:
            raise ValueError(
                f"could not place group {gid} (size {size}) in region {cfg.spawn_region}; "
                "loosen min_separation/group_spacing or widen the region")
        (x0, y0), (x1, y1) = _walk_route(cfg, center)
        seg = math.hypot(x1 - x0, y1 - y0)
        px, py = (-(y1 - y0) / seg, (x1 - x0) / seg) if seg > 1e-6 else (0.0, 1.0)
        for j in range(size):
            offset = (j - 0.5 * (size - 1)) * cfg.group_spacing
            route = [(x0 + px * offset, y0 + py * offset),
                     (x1 + px * offset, y1 + py * offset)]
            placed.append(route[0])
            people.append(Pedestrian(id=idx, name=f"pedestrian_{idx}", behavior=GROUP,
                                     waypoints=route, speed=cfg.speed, group_id=gid))
            idx += 1
    return people


def plan_pedestrians(cfg: SpawnConfig) -> List[Pedestrian]:
    """Deterministically plan cfg.num_humans pedestrians. A crossing pedestrian's spawn is its
    route start (already validated free); a walking pedestrian is placed by rejection sampling.
    Raises ValueError on an unknown profile or when a safe spawn cannot be found."""
    if cfg.behavior_profile not in PROFILES:
        raise ValueError(f"unknown behavior_profile {cfg.behavior_profile!r}; use {PROFILES}")

    rng = random.Random(cfg.seed)
    if cfg.behavior_profile == GROUP:
        return _plan_groups(cfg, rng)
    placed: List[Tuple[float, float]] = []
    people: List[Pedestrian] = []
    for i in range(cfg.num_humans):
        behavior = _behavior_for(cfg.behavior_profile, i)
        if behavior == CROSSING:
            x = _place_crossing_x(cfg, placed)
            route = [(x, cfg.crossing_north), (x, cfg.crossing_south)]
            spawn = route[0]
        else:
            spawn = _sample_spawn(cfg, rng, placed)
            if spawn is None:
                raise ValueError(
                    f"could not place walking pedestrian {i} in region {cfg.spawn_region} "
                    "clear of obstacles/robot/others; loosen min_separation or the region"
                )
            route = _walk_route(cfg, spawn)
        placed.append(spawn)
        people.append(Pedestrian(id=i, name=f"pedestrian_{i}", behavior=behavior,
                                 waypoints=route, speed=cfg.speed))
    return people


def pose_at_time(ped: Pedestrian, t: float) -> Tuple[float, float, float, float, float]:
    """Pose of a pedestrian at elapsed sim time t (s). The route is walked start->end->start at
    constant speed. Returns (x, y, yaw, vx, vy); a zero-length route yields a stationary pose."""
    (x0, y0), (x1, y1) = ped.waypoints[0], ped.waypoints[-1]
    seg = math.hypot(x1 - x0, y1 - y0)
    if seg < 1e-6 or ped.speed <= 0.0:
        return (x0, y0, 0.0, 0.0, 0.0)

    period = 2.0 * seg / ped.speed  # out and back
    phase = (t % period) / period
    # Triangle wave: 0->1 over the first half, 1->0 over the second.
    frac = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
    direction = 1.0 if phase < 0.5 else -1.0

    ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
    x = x0 + (x1 - x0) * frac
    y = y0 + (y1 - y0) * frac
    vx, vy = ux * ped.speed * direction, uy * ped.speed * direction
    yaw = math.atan2(vy, vx)
    return (x, y, yaw, vx, vy)
