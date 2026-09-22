from social_nav_rl import config as C


def test_obs_override_and_defaults(tmp_path):
    p = tmp_path / "observation.yaml"
    p.write_text("n_humans: 9\nuse_ttc: false\n")
    cfg = C.load_obs_config(str(p))
    assert cfg.n_humans == 9 and cfg.use_ttc is False
    assert cfg.max_range == 8.0          # untouched -> default


def test_missing_file_is_defaults():
    cfg = C.load_obs_config("/no/such/file.yaml")
    assert cfg.n_humans == 5


def test_reward_weights_merge(tmp_path):
    p = tmp_path / "reward.yaml"
    p.write_text("weights:\n  collision: -99.0\nparams:\n  comfort_dist: 2.0\n")
    cfg = C.load_reward_config(str(p))
    assert cfg.weights["collision"] == -99.0
    assert cfg.weights["goal_completion"] == 50.0   # default kept
    assert cfg.params["comfort_dist"] == 2.0


def test_unknown_keys_ignored(tmp_path):
    p = tmp_path / "action.yaml"
    p.write_text("max_lin_vel: 1.5\nbogus_key: 3\n")
    cfg = C.load_action_limits(str(p))
    assert cfg.max_lin_vel == 1.5


def test_load_experiment_from_shipped_config():
    exp = C.load_experiment()   # uses the package's config/ dir
    assert exp["observation"].n_humans >= 1
    assert set(("observation", "action", "reward", "safety")) <= set(exp.keys())
