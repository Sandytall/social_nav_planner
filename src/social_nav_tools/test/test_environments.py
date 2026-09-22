import math

import pytest

from social_nav_tools import environments as E
from social_nav_tools.pedestrian_model import PROFILES


def test_registry_integrity():
    for name, env in E.ENVIRONMENTS.items():
        assert env.name == name
        assert env.world.endswith(".world")
        assert env.default_goal in env.goals
        assert len(env.obstacles) % 4 == 0  # flat [xmin,ymin,xmax,ymax,...]
        xmin, xmax, ymin, ymax = env.spawn_region
        assert xmin < xmax and ymin < ymax


def test_all_scenarios_map_to_real_profiles():
    for scenario, (profile, count) in E.SCENARIO_BASE.items():
        assert profile in PROFILES
        assert count >= 0


def test_generate_for_every_env_scenario_difficulty():
    for env in E.ENVIRONMENT_NAMES:
        for scenario in E.SCENARIO_TYPES:
            for diff in E.DIFFICULTIES:
                cfg = E.generate_scenario(env, scenario, diff, seed=7)
                assert cfg["world"].endswith(".world")
                assert cfg["sensor_profile"] in ("clean", "realistic", "stress")
                assert cfg["num_robots"] >= 1
                assert cfg["num_dynamic_obstacles"] >= 0
                assert cfg["pedestrian"]["behavior_profile"] in PROFILES
                assert cfg["pedestrian"]["num_humans"] >= 0


def test_deterministic():
    a = E.generate_scenario("warehouse", "crossing", "hard", seed=99)
    b = E.generate_scenario("warehouse", "crossing", "hard", seed=99)
    assert a == b


def test_difficulty_scales_humans_and_sensor():
    easy = E.generate_scenario("urban", "high_density", "easy")
    stress = E.generate_scenario("urban", "high_density", "stress")
    assert stress["pedestrian"]["num_humans"] > easy["pedestrian"]["num_humans"]
    assert easy["sensor_profile"] == "clean" and stress["sensor_profile"] == "stress"
    assert stress["pedestrian"]["speed"] > easy["pedestrian"]["speed"]


def test_multi_robot_and_dynamic_obstacle_scenarios():
    mr = E.generate_scenario("warehouse", "multi_robot", "medium")
    assert mr["num_robots"] == 3
    do = E.generate_scenario("warehouse", "dynamic_obstacle", "medium")
    assert do["num_dynamic_obstacles"] >= 2


def test_empty_scenario_has_no_humans():
    assert E.generate_scenario("urban", "empty", "hard")["pedestrian"]["num_humans"] == 0


def test_human_count_override():
    cfg = E.generate_scenario("urban", "normal", "medium", human_count=11)
    assert cfg["pedestrian"]["num_humans"] == 11


def test_goals_within_costmap_window():
    # Goals must sit inside the ~10 m rolling global-costmap window around the start.
    for env in E.ENVIRONMENTS.values():
        sx, sy, _ = env.robot_start
        for gx, gy in env.goals.values():
            assert math.hypot(gx - sx, gy - sy) <= 10.0


def test_warehouse_crossers_stay_in_aisle():
    # Warehouse racks flank y in [1.6, 3.4] / [-3.4, -1.6]; crossers must stay between.
    env = E.ENVIRONMENTS["warehouse"]
    assert -1.6 < env.crossing_south and env.crossing_north < 1.6


def test_unknown_inputs_raise():
    for bad in [("nope", "normal", "medium"), ("urban", "nope", "medium"),
                ("urban", "normal", "nope")]:
        with pytest.raises(ValueError):
            E.generate_scenario(*bad)
