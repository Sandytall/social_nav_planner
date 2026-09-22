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
HEAD_ON = "head_on"
TURNING = "turning"
SAME_DIRECTION = "same_direction"
BLOCKER = "blocker"
STOP_GO = "stop_go"
MERGE = "merge"
PROFILES = (WALKING, CROSSING, MIXED, GROUP, HEAD_ON, TURNING,
            SAME_DIRECTION, BLOCKER, STOP_GO, MERGE)


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
    stop_start: float = -1.0     # sudden-stop schedule (s from route start); < 0 disables
    stop_duration: float = 0.0   # how long the pedestrian holds position when it stops

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
    if profile in (WALKING, CROSSING, HEAD_ON, TURNING, SAME_DIRECTION, BLOCKER, STOP_GO):
        return profile
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


def _head_on_route(cfg: SpawnConfig, index: int) -> List[Tuple[float, float]]:
    """A pedestrian on the robot's lane (y approx 0) walking from far ahead toward the robot
    start, i.e. head-on. Multiple head-on walkers are staggered in x and slightly in y."""
    y = 0.0 if index == 0 else (0.4 if index % 2 else -0.4)
    x_far = min(cfg.walk_xmax, 8.0) - index * 1.6
    return [(x_far, y), (1.5, y)]


def _turning_route(cfg: SpawnConfig, index: int) -> List[Tuple[float, float]]:
    """An L-shaped route: walk east along the sidewalk, then turn south across the robot's
    lane. The bend gives the planner a genuine direction change to react to."""
    sx = 2.0 + index * 1.2
    turn_x = 6.0 + index * 1.0
    return [(sx, 1.0), (turn_x, 1.0), (turn_x, -0.5)]


def _same_direction_route(cfg: SpawnConfig, index: int) -> List[Tuple[float, float]]:
    """Walk EAST along the robot's lane (same direction as the robot), so the robot overtakes."""
    y = 0.0 if index == 0 else (0.4 if index % 2 else -0.4)
    x_near = 2.0 + index * 1.2
    return [(x_near, y), (cfg.walk_xmax, y)]


def _blocker_route(cfg: SpawnConfig, index: int) -> List[Tuple[float, float]]:
    """A person standing still ON the robot's lane, forcing a go-around (a zero-length route is
    stationary in pose_at_time)."""
    x = 4.5 + index * 1.5
    return [(x, 0.0), (x, 0.0)]


def _stop_go_route(cfg: SpawnConfig, index: int) -> List[Tuple[float, float]]:
    """A person crossing the lane; paired with a stop schedule (set by the caller) they pause
    mid-crossing, right on the robot's path."""
    x = cfg.crossing_x + index * 1.0
    return [(x, cfg.crossing_north), (x, cfg.crossing_south)]


def _plan_merge(cfg: SpawnConfig) -> List[Pedestrian]:
    """Two small groups on separate lanes whose routes bend toward a shared centreline, so they
    converge into one stream. This approximates merge dynamics with converging poly-line routes;
    group membership itself is fixed (the model has no time-varying membership)."""
    people: List[Pedestrian] = []
    per_group = max(2, cfg.num_humans // 2)
    merge_x = 0.5 * (2.0 + cfg.walk_xmax)
    idx = 0
    for gid, lane_y in enumerate((1.2, -1.2)):
        for k in range(per_group):
            sx = 2.0 + k * 0.6  # stagger members along the lane
            route = [(sx, lane_y), (merge_x, lane_y), (cfg.walk_xmax, 0.0)]
            people.append(Pedestrian(id=idx, name=f"pedestrian_{idx}", behavior=MERGE,
                                     waypoints=route, speed=cfg.speed, group_id=gid))
            idx += 1
    return people


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
        cluster_r = cfg.group_spacing * (1 + (size - 1) // 2)
        center = _sample_spawn(cfg, rng, placed, extra_margin=cluster_r)
        if center is None:
            raise ValueError(
                f"could not place group {gid} (size {size}) in region {cfg.spawn_region}; "
                "loosen min_separation/group_spacing or widen the region")
        (x0, y0), (x1, y1) = _walk_route(cfg, center)
        seg = math.hypot(x1 - x0, y1 - y0)
        ux, uy = ((x1 - x0) / seg, (y1 - y0) / seg) if seg > 1e-6 else (1.0, 0.0)
        px, py = -uy, ux  # perpendicular to travel
        for j in range(size):
            # Compact 2-wide cluster: columns spread laterally, rows trail behind, so the
            # group stays about one lane wide instead of a single wide row.
            lateral = (j % 2 - 0.5) * cfg.group_spacing
            along = -(j // 2) * cfg.group_spacing
            dx, dy = px * lateral + ux * along, py * lateral + uy * along
            route = [(x0 + dx, y0 + dy), (x1 + dx, y1 + dy)]
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
    if cfg.behavior_profile == MERGE:
        return _plan_merge(cfg)
    placed: List[Tuple[float, float]] = []
    people: List[Pedestrian] = []
    for i in range(cfg.num_humans):
        behavior = _behavior_for(cfg.behavior_profile, i)
        stop_start, stop_duration = -1.0, 0.0
        if behavior == CROSSING:
            x = _place_crossing_x(cfg, placed)
            route = [(x, cfg.crossing_north), (x, cfg.crossing_south)]
            spawn = route[0]
        elif behavior == HEAD_ON:
            route = _head_on_route(cfg, i)
            spawn = route[0]
        elif behavior == TURNING:
            route = _turning_route(cfg, i)
            spawn = route[0]
        elif behavior == SAME_DIRECTION:
            route = _same_direction_route(cfg, i)
            spawn = route[0]
        elif behavior == BLOCKER:
            route = _blocker_route(cfg, i)
            spawn = route[0]
        elif behavior == STOP_GO:
            route = _stop_go_route(cfg, i)
            spawn = route[0]
            # Pause once the crosser reaches the robot's lane (~y=0), so it stops in the path.
            stop_start = cfg.crossing_north / max(cfg.speed, 1e-6)
            stop_duration = 4.0
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
                                 waypoints=route, speed=cfg.speed,
                                 stop_start=stop_start, stop_duration=stop_duration))
    return people


def _pose_on_route(ped: Pedestrian, t: float) -> Tuple[float, float, float, float, float]:
    """Position along the poly-line at continuous time t, walked start->end->start, repeating."""
    pts = ped.waypoints
    seglens = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1)]
    total = sum(seglens)
    if total < 1e-6 or ped.speed <= 0.0 or len(pts) < 2:
        x0, y0 = pts[0]
        return (x0, y0, 0.0, 0.0, 0.0)

    period = 2.0 * total / ped.speed  # out and back
    dist = ped.speed * (t % period)
    if dist <= total:
        s, forward = dist, 1.0
    else:
        s, forward = 2.0 * total - dist, -1.0

    acc = 0.0
    for i, seglen in enumerate(seglens):
        if seglen < 1e-9:
            continue
        if acc + seglen >= s or i == len(seglens) - 1:
            local = min(max((s - acc) / seglen, 0.0), 1.0)
            (ax, ay), (bx, by) = pts[i], pts[i + 1]
            ux, uy = (bx - ax) / seglen, (by - ay) / seglen
            x, y = ax + (bx - ax) * local, ay + (by - ay) * local
            vx, vy = ux * ped.speed * forward, uy * ped.speed * forward
            return (x, y, math.atan2(vy, vx), vx, vy)
        acc += seglen
    x0, y0 = pts[0]
    return (x0, y0, 0.0, 0.0, 0.0)


def pose_at_time(ped: Pedestrian, t: float) -> Tuple[float, float, float, float, float]:
    """Pose at elapsed sim time t (s). Walks the whole waypoint poly-line start->end->start at
    constant speed (routes may bend). If a sudden-stop schedule is set, the pedestrian holds
    position (zero velocity) during [stop_start, stop_start+stop_duration) and resumes after.
    Returns (x, y, yaw, vx, vy). For a plain 2-point route with no stop this is a straight
    out-and-back."""
    if ped.stop_start >= 0.0 and ped.stop_duration > 0.0:
        if ped.stop_start <= t < ped.stop_start + ped.stop_duration:
            x, y, yaw, _, _ = _pose_on_route(ped, ped.stop_start)
            return (x, y, yaw, 0.0, 0.0)
        if t >= ped.stop_start + ped.stop_duration:
            t = t - ped.stop_duration
    return _pose_on_route(ped, t)
