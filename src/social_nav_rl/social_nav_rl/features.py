"""Pure geometry features shared by the observation adapter and the reward.

Mirrors the concepts the classical planner already uses (time-to-collision / closest approach
and anisotropic social zones) so the RL policy is evaluated on the same footing. NumPy only -
no ROS, no torch - so every feature is unit-testable in isolation.
"""
import math

import numpy as np

BIG_TTC = 1e3  # "no collision predicted" sentinel (seconds)


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def to_robot_frame(px: float, py: float, rx: float, ry: float, ryaw: float):
    """World point (px, py) expressed in the robot frame (robot at rx,ry heading ryaw)."""
    dx, dy = px - rx, py - ry
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    return c * dx - s * dy, s * dx + c * dy


def time_to_collision(rel_pos, rel_vel, contact_dist: float) -> float:
    """Time until the human (relative position/velocity w.r.t. the robot) first comes within
    contact_dist, assuming constant velocity. Returns BIG_TTC if they never do.

    rel_pos = human_xy - robot_xy; rel_vel = human_vxy - robot_vxy (world frame).
    """
    px, py = float(rel_pos[0]), float(rel_pos[1])
    vx, vy = float(rel_vel[0]), float(rel_vel[1])
    v2 = vx * vx + vy * vy
    if v2 < 1e-9:                       # not closing (or identical velocity)
        return BIG_TTC
    # Solve |rel_pos + t*rel_vel|^2 = contact_dist^2 for the first t >= 0.
    b = 2.0 * (px * vx + py * vy)
    c = px * px + py * py - contact_dist * contact_dist
    disc = b * b - 4.0 * v2 * c
    if disc < 0.0:                      # closest approach never reaches contact_dist
        return BIG_TTC
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * v2)
    t2 = (-b + sqrt_disc) / (2.0 * v2)
    for t in sorted((t1, t2)):
        if t >= 0.0:
            return min(t, BIG_TTC)
    return BIG_TTC


def social_zone_cost(rx_h: float, ry_h: float, sigma_front: float = 1.0,
                     sigma_side: float = 0.7, sigma_rear: float = 0.5) -> float:
    """Anisotropic personal-space cost in [0, 1] for the robot at (rx_h, ry_h) expressed in the
    HUMAN's frame (+x = the human's facing direction). Front space costs more than rear."""
    sx = sigma_front if rx_h >= 0.0 else sigma_rear
    return float(math.exp(-(rx_h * rx_h) / (2.0 * sx * sx)
                          - (ry_h * ry_h) / (2.0 * sigma_side * sigma_side)))


def relative_heading(robot_yaw: float, human_yaw: float) -> float:
    """Human heading relative to the robot heading, wrapped to (-pi, pi]."""
    return wrap_angle(human_yaw - robot_yaw)


def goal_polar(robot_x, robot_y, robot_yaw, goal_x, goal_y):
    """(distance, bearing) of the goal in the robot frame."""
    gx, gy = to_robot_frame(goal_x, goal_y, robot_x, robot_y, robot_yaw)
    return math.hypot(gx, gy), math.atan2(gy, gx)


def safe_array(values, fill=0.0):
    """Turn a sequence into a float array with NaN/Inf replaced by `fill` (robust observations)."""
    arr = np.asarray(values, dtype=np.float32)
    return np.nan_to_num(arr, nan=fill, posinf=fill, neginf=fill)
