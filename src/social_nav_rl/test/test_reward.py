from social_nav_rl.reward import DEFAULT_WEIGHTS, RewardComputer, RewardConfig


def test_all_components_present():
    total, comps = RewardComputer().compute({})
    assert set(comps.keys()) == set(DEFAULT_WEIGHTS.keys())
    assert isinstance(total, float)


def test_goal_progress_positive_when_closer():
    _, comps = RewardComputer().compute({"prev_goal_dist": 5.0, "goal_dist": 4.5})
    assert comps["goal_progress"] > 0.0


def test_collision_is_large_negative():
    total, comps = RewardComputer().compute({"collision": True})
    assert comps["collision"] == -50.0 and total < 0.0


def test_human_collision_worse_than_obstacle():
    _, ch = RewardComputer().compute({"human_collision": True})
    _, co = RewardComputer().compute({"collision": True})
    assert ch["human_collision"] < co["collision"]


def test_goal_completion_reward():
    _, comps = RewardComputer().compute({"reached": True})
    assert comps["goal_completion"] == 50.0


def test_ttc_penalty_only_when_dangerous():
    _, safe = RewardComputer().compute({"min_ttc": 10.0})
    _, danger = RewardComputer().compute({"min_ttc": 0.5})
    assert safe["ttc"] == 0.0 and danger["ttc"] < 0.0


def test_stopping_penalized_in_open_space():
    # stationary, not at goal, no human nearby -> dawdling -> penalized
    _, c = RewardComputer().compute({"v": 0.0, "reached": False, "min_clearance": 5.0})
    assert c["stopping"] < 0.0


def test_stopping_not_penalized_when_yielding_to_close_human():
    # stationary with a human within wait_clearance -> waiting/yielding -> NOT penalized
    _, c = RewardComputer().compute({"v": 0.0, "reached": False, "min_clearance": 1.0})
    assert c["stopping"] == 0.0


def test_zero_weights_zero_total():
    cfg = RewardConfig(weights={k: 0.0 for k in DEFAULT_WEIGHTS})
    total, _ = RewardComputer(cfg).compute({"collision": True, "reached": True})
    assert total == 0.0
