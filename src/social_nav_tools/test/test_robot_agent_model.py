import math

from social_nav_tools.pedestrian_model import pose_at_time
from social_nav_tools.robot_agent_model import AMR, plan_robot_agents


def test_count_and_behavior():
    agents = plan_robot_agents(3)
    assert len(agents) == 3
    assert all(a.behavior == AMR for a in agents)
    assert [a.name for a in agents] == ["amr_0", "amr_1", "amr_2"]


def test_zero_agents():
    assert plan_robot_agents(0) == []


def test_directions_alternate():
    agents = plan_robot_agents(2, x_range=(3.0, 11.0))
    assert agents[0].waypoints[0][0] < agents[0].waypoints[-1][0]   # even: east-first
    assert agents[1].waypoints[0][0] > agents[1].waypoints[-1][0]   # odd: west-first


def test_lanes_cycle():
    agents = plan_robot_agents(3, lane_ys=(0.0, -1.0, 1.0))
    assert [a.waypoints[0][1] for a in agents] == [0.0, -1.0, 1.0]


def test_agent_moves():
    agent = plan_robot_agents(1, speed=0.5)[0]
    _, _, _, vx, vy = pose_at_time(agent, 1.0)
    assert math.hypot(vx, vy) > 0.0
