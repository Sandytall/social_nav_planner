"""Deterministic factory-worker planning: which workers exist and how they move.

Like pedestrian_model.py this module has no ROS or Gazebo imports so the role/route/dwell
logic can be unit tested on its own. worker_manager.py feeds each plan to both Gazebo (a
visible body) and /social_nav/humans (planner input) from one trajectory, so the two can
never drift.

Workers differ from urban pedestrians in that they are not always walking. A worker walks to
a workstation, stops, dwells (works) for a while, then leaves for a new destination. That
temporal stop-and-go is modelled with a trapezoidal speed profile per travel leg (accelerate
from rest, cruise, decelerate to rest at the station) plus explicit dwell segments. A single
timeline of segments is precomputed per worker so pose_at_time stays a pure function of the
elapsed time and the manager can remain stateless between ticks.

Coordinates are the shared map/world frame in metres (the factory launch spawns the robot at
the world origin so map == odom == Gazebo world; see factory_demo.launch.py). The Box keep-out
type is reused from pedestrian_model to describe machinery/rack footprints.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

from social_nav_tools.pedestrian_model import Box

WORKSTATION = "workstation"  # walk -> dwell -> next station -> dwell ... looping
CROSSING = "crossing"        # cross the robot lane back and forth, brief pause each side
PATROL = "patrol"            # continuous loop, no dwell (transport/inspection style)
ROLES = (WORKSTATION, CROSSING, PATROL)

Point = Tuple[float, float]


@dataclass
class Segment:
    """One leg of a worker timeline: either travel (dwell=False, trapezoidal profile) or a
    stationary dwell (dwell=True). Active for elapsed time t in [t0, t1)."""

    x0: float
    y0: float
    x1: float
    y1: float
    t0: float
    t1: float
    dwell: bool
    length: float
    speed: float   # cruise speed for a travel leg (0 for a dwell)
    accel: float   # acceleration for a travel leg (0 for a dwell)
    heading: float  # facing direction; travel legs point along the leg, dwells keep the last


@dataclass
class Worker:
    """A planned worker: identity, role, the ordered stations it visits, and the precomputed
    cyclic timeline of segments that drives it."""

    id: int
    name: str
    role: str
    route: List[Point]
    speed: float
    accel: float
    dwell_time: float
    segments: List[Segment] = field(default_factory=list)
    period: float = 0.0

    @property
    def start(self) -> Point:
        return self.route[0]


@dataclass
class WorkerConfig:
    """Inputs that fully determine a worker plan. Same config + same seed => same workers."""

    num_workstation: int
    num_crossing: int
    seed: int = 42
    num_patrol: int = 0
    workstations: Sequence[Point] = ()               # pool of dwell stations (world coords)
    crossing_routes: Sequence[Tuple[Point, Point]] = ()  # (north, south) lane crossings
    patrol_routes: Sequence[Sequence[Point]] = ()    # explicit patrol loops
    obstacles: Sequence[Box] = ()
    robot_start: Point = (0.0, 0.0)
    robot_keepout: float = 1.5
    walkable: Tuple[float, float, float, float] = (-100.0, 100.0, -100.0, 100.0)
    min_separation: float = 1.2
    speed: float = 0.9
    accel: float = 0.6
    dwell_time: float = 6.0
    dwell_jitter: float = 2.0
    speed_jitter: float = 0.15
    stations_per_worker: int = 3
    crossing_dwell: float = 1.5


def _leg_time(length: float, v: float, a: float) -> float:
    """Total time to cover a straight leg with a trapezoidal (accel/cruise/decel) profile that
    starts and ends at rest. Falls back to a triangular profile when the leg is too short to
    reach cruise speed."""
    if length <= 1e-9:
        return 0.0
    if v <= 0.0 or a <= 0.0:
        return length / max(v, 1e-6)
    d_acc = v * v / (2.0 * a)
    if 2.0 * d_acc <= length:
        return 2.0 * (v / a) + (length - 2.0 * d_acc) / v
    v_peak = math.sqrt(a * length)
    return 2.0 * v_peak / a


def _leg_state(length: float, v: float, a: float, tau: float) -> Tuple[float, float]:
    """Distance covered and speed magnitude at local leg time tau, for the same trapezoidal
    profile as _leg_time. tau is clamped to the leg duration so the end state is (length, 0)."""
    if length <= 1e-9:
        return 0.0, 0.0
    if v <= 0.0 or a <= 0.0:
        return min(v * tau, length), v
    d_acc = v * v / (2.0 * a)
    if 2.0 * d_acc <= length:
        t_acc = v / a
        t_cruise = (length - 2.0 * d_acc) / v
        total = 2.0 * t_acc + t_cruise
        tau = max(0.0, min(tau, total))
        if tau <= t_acc:
            return 0.5 * a * tau * tau, a * tau
        if tau <= t_acc + t_cruise:
            return d_acc + v * (tau - t_acc), v
        td = tau - (t_acc + t_cruise)
        return d_acc + v * t_cruise + (v * td - 0.5 * a * td * td), max(v - a * td, 0.0)
    v_peak = math.sqrt(a * length)
    t_acc = v_peak / a
    total = 2.0 * t_acc
    tau = max(0.0, min(tau, total))
    if tau <= t_acc:
        return 0.5 * a * tau * tau, a * tau
    td = tau - t_acc
    d_acc = 0.5 * a * t_acc * t_acc
    return d_acc + (v_peak * td - 0.5 * a * td * td), max(v_peak - a * td, 0.0)


def _build_segments(route: Sequence[Point], speed: float, accel: float, dwell_time: float
                    ) -> Tuple[List[Segment], float]:
    """Build a cyclic timeline for a closed loop over `route` (returning to route[0]). Each
    arrival triggers a dwell of dwell_time; the home station is dwelled once per cycle. A
    route of one point (or all-zero motion) yields a single stationary dwell."""
    pts = list(route)
    if len(pts) < 2:
        x0, y0 = pts[0] if pts else (0.0, 0.0)
        seg = Segment(x0, y0, x0, y0, 0.0, max(dwell_time, 1.0), True, 0.0, 0.0, 0.0, 0.0)
        return [seg], seg.t1

    loop = pts + [pts[0]]  # travel back to the start so the timeline wraps cleanly
    segs: List[Segment] = []
    t = 0.0
    last_heading = 0.0
    for i in range(len(loop) - 1):
        (x0, y0), (x1, y1) = loop[i], loop[i + 1]
        length = math.hypot(x1 - x0, y1 - y0)
        heading = math.atan2(y1 - y0, x1 - x0) if length > 1e-9 else last_heading
        dt = _leg_time(length, speed, accel)
        segs.append(Segment(x0, y0, x1, y1, t, t + dt, False, length, speed, accel, heading))
        t += dt
        last_heading = heading
        is_closing_leg = i == len(loop) - 2
        if dwell_time > 0.0 and not is_closing_leg:
            segs.append(Segment(x1, y1, x1, y1, t, t + dwell_time, True, 0.0, 0.0, 0.0, heading))
            t += dwell_time
    if dwell_time > 0.0:
        hx, hy = loop[0]
        segs.append(Segment(hx, hy, hx, hy, t, t + dwell_time, True, 0.0, 0.0, 0.0, last_heading))
        t += dwell_time
    return segs, (t if t > 1e-9 else 1.0)


def _clear(pt: Point, cfg: WorkerConfig, placed: Sequence[Point], separation: float) -> bool:
    """A point is a safe spawn/station if it is inside the walkable area, out of every keep-out
    footprint, clear of the robot start, and `separation` from already-placed spawns."""
    x, y = pt
    xmin, xmax, ymin, ymax = cfg.walkable
    if not (xmin <= x <= xmax and ymin <= y <= ymax):
        return False
    if any(o.contains(x, y, margin=separation * 0.5) for o in cfg.obstacles):
        return False
    if math.hypot(x - cfg.robot_start[0], y - cfg.robot_start[1]) < cfg.robot_keepout:
        return False
    if any(math.hypot(x - px, y - py) < separation for px, py in placed):
        return False
    return True


def _jitter_speed(cfg: WorkerConfig, rng: random.Random) -> float:
    lo = max(0.2, cfg.speed - cfg.speed_jitter)
    return rng.uniform(lo, cfg.speed + cfg.speed_jitter)


def _plan_crossing(cfg: WorkerConfig, rng: random.Random, idx: int, placed: List[Point]
                   ) -> Worker:
    """Plan one crossing worker. Its route is a north/south pair straddling the robot lane;
    successive crossers are staggered along x so they do not overlap, and the north spawn is
    validated clear. Raises ValueError if no clear lane can be found."""
    if not cfg.crossing_routes:
        raise ValueError("crossing worker requested but crossing_routes is empty")
    base = cfg.crossing_routes[idx % len(cfg.crossing_routes)]
    (nx, ny), (sx, sy) = base
    step = max(cfg.min_separation, 0.6)
    offsets = [0.0]
    for k in range(1, 10):
        offsets.extend((k * step, -k * step))
    for off in offsets:
        north = (nx + off, ny)
        south = (sx + off, sy)
        if not _clear(north, cfg, placed, cfg.min_separation):
            continue
        if any(o.contains(south[0], south[1], margin=0.0) for o in cfg.obstacles):
            continue
        placed.append(north)
        speed = _jitter_speed(cfg, rng)
        segs, period = _build_segments([north, south], speed, cfg.accel, cfg.crossing_dwell)
        return Worker(id=idx, name=f"worker_cross_{idx}", role=CROSSING,
                      route=[north, south], speed=speed, accel=cfg.accel,
                      dwell_time=cfg.crossing_dwell, segments=segs, period=period)
    raise ValueError(
        f"could not place crossing worker {idx} clear of machinery/others near "
        f"x={nx}; widen the walkway or reduce num_crossing")


def _plan_workstation(cfg: WorkerConfig, rng: random.Random, idx: int, placed: List[Point]
                      ) -> Worker:
    """Plan one workstation-dwell worker: pick distinct stations from the pool, verify each is
    a clear standing spot and the first is a clear spawn, then build a walk/dwell loop over
    them. Raises ValueError if the pool cannot supply a clear set."""
    pool = list(cfg.workstations)
    n = min(cfg.stations_per_worker, len(pool))
    if n < 1:
        raise ValueError("workstation worker requested but workstations pool is empty")
    for _ in range(200):
        stations = rng.sample(pool, n) if n < len(pool) else rng.sample(pool, len(pool))
        if any(o.contains(sx, sy, margin=0.0) for (sx, sy) in stations for o in cfg.obstacles):
            continue
        if not _clear(stations[0], cfg, placed, cfg.min_separation):
            continue
        placed.append(stations[0])
        speed = _jitter_speed(cfg, rng)
        dwell = max(1.0, cfg.dwell_time + rng.uniform(-cfg.dwell_jitter, cfg.dwell_jitter))
        segs, period = _build_segments(stations, speed, cfg.accel, dwell)
        return Worker(id=idx, name=f"worker_ws_{idx}", role=WORKSTATION,
                      route=list(stations), speed=speed, accel=cfg.accel,
                      dwell_time=dwell, segments=segs, period=period)
    raise ValueError(
        f"could not assign workstation worker {idx} a clear station set from the pool; "
        "add stations clear of machinery/robot or reduce stations_per_worker")


def _plan_patrol(cfg: WorkerConfig, rng: random.Random, idx: int, placed: List[Point]
                 ) -> Worker:
    """Plan one continuously-walking patrol worker over an explicit loop (no dwell)."""
    if not cfg.patrol_routes:
        raise ValueError("patrol worker requested but patrol_routes is empty")
    loop = list(cfg.patrol_routes[idx % len(cfg.patrol_routes)])
    if not _clear(loop[0], cfg, placed, cfg.min_separation):
        raise ValueError(f"patrol worker {idx} start {loop[0]} is not clear")
    placed.append(loop[0])
    speed = _jitter_speed(cfg, rng)
    segs, period = _build_segments(loop, speed, cfg.accel, 0.0)
    return Worker(id=idx, name=f"worker_patrol_{idx}", role=PATROL,
                  route=loop, speed=speed, accel=cfg.accel,
                  dwell_time=0.0, segments=segs, period=period)


def plan_workers(cfg: WorkerConfig) -> List[Worker]:
    """Deterministically plan the requested crossing, workstation and patrol workers. Crossing
    workers are planned first so a crossing is guaranteed even under tight separation. Same
    config and seed reproduce the same workers. Raises ValueError on infeasible placement."""
    rng = random.Random(cfg.seed)
    placed: List[Point] = []
    workers: List[Worker] = []
    next_id = 0
    for i in range(cfg.num_crossing):
        w = _plan_crossing(cfg, rng, next_id, placed)
        workers.append(w)
        next_id += 1
    for i in range(cfg.num_workstation):
        w = _plan_workstation(cfg, rng, next_id, placed)
        workers.append(w)
        next_id += 1
    for i in range(cfg.num_patrol):
        w = _plan_patrol(cfg, rng, next_id, placed)
        workers.append(w)
        next_id += 1
    return workers


def active_segment(worker: Worker, t: float) -> Segment:
    """The segment active at elapsed time t (cyclic). Useful for tests and introspection."""
    if worker.period <= 0.0 or not worker.segments:
        raise ValueError("worker has no timeline")
    tt = t % worker.period
    for seg in worker.segments:
        if tt < seg.t1:
            return seg
    return worker.segments[-1]


def is_dwelling(worker: Worker, t: float) -> bool:
    """True when the worker is stopped at a station at elapsed time t (the 'working' phase)."""
    if worker.period <= 0.0 or not worker.segments:
        return True
    return active_segment(worker, t).dwell


def pose_at_time(worker: Worker, t: float) -> Tuple[float, float, float, float, float]:
    """Pose of a worker at elapsed sim time t (s), cyclic over the worker's timeline. Returns
    (x, y, yaw, vx, vy). A dwelling worker is stationary and keeps its last heading."""
    if worker.period <= 0.0 or not worker.segments:
        x0, y0 = worker.route[0]
        return (x0, y0, 0.0, 0.0, 0.0)
    seg = active_segment(worker, t)
    if seg.dwell or seg.length <= 1e-9:
        return (seg.x0, seg.y0, seg.heading, 0.0, 0.0)
    tau = (t % worker.period) - seg.t0
    s_dist, speed = _leg_state(seg.length, seg.speed, seg.accel, tau)
    ux = (seg.x1 - seg.x0) / seg.length
    uy = (seg.y1 - seg.y0) / seg.length
    x = seg.x0 + ux * s_dist
    y = seg.y0 + uy * s_dist
    return (x, y, seg.heading, ux * speed, uy * speed)
