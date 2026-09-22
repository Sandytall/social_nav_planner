"""Deterministic planning for secondary AMR agents (the multi-robot extension).

Extra robots are NOT part of the social-navigation problem: they are simple moving obstacles
the main robot must avoid via its ordinary obstacle costmap. Each agent paces a lane; the motion
is evaluated with pedestrian_model.pose_at_time (a Pedestrian carries the route), so the two
share one, tested trajectory engine. ROS/Gazebo-free so it can be unit tested on its own.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

from social_nav_tools.pedestrian_model import Pedestrian

AMR = "amr"


def plan_robot_agents(num_agents: int, speed: float = 0.4,
                      x_range: Tuple[float, float] = (3.0, 11.0),
                      lane_ys: Sequence[float] = (0.0, -1.0, 1.0)) -> List[Pedestrian]:
    """Plan `num_agents` AMRs pacing lanes between x_range on cycling `lane_ys`. Even-indexed
    agents travel east-first, odd ones west-first, so oncoming/overtaking traffic both occur.
    Deterministic: same arguments => same agents. Returns Pedestrian objects (behavior "amr")."""
    x0, x1 = x_range
    lanes = list(lane_ys) or [0.0]
    agents: List[Pedestrian] = []
    for i in range(max(0, num_agents)):
        y = lanes[i % len(lanes)]
        route = [(x0, y), (x1, y)] if i % 2 == 0 else [(x1, y), (x0, y)]
        agents.append(Pedestrian(id=i, name=f"amr_{i}", behavior=AMR,
                                 waypoints=route, speed=speed))
    return agents
