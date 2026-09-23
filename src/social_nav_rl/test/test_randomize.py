import random

from social_nav_rl.randomize import DRConfig, Randomizer, default_profile


def test_disabled_ranges_are_identity():
    r = Randomizer(DRConfig(enabled=True), random.Random(0))     # all ranges zero
    assert r.delay_action(0.5, -0.2) == (0.5, -0.2)              # no latency, no scale
    beams = [1.0, 2.0, 3.0]
    assert r.noisy_lidar(beams, 5.0) == beams                    # noise/dropout off -> unchanged


def test_latency_returns_delayed_action():
    cfg = DRConfig(enabled=True, action_latency=3)
    r = Randomizer(cfg, random.Random(1))
    r.latency = 2                                                # force a known delay
    r._buf = []
    r.delay_action(1.0, 0.0)                                     # buffer fills
    r.delay_action(2.0, 0.0)
    v, _ = r.delay_action(3.0, 0.0)                              # should echo the action 2 steps ago
    assert v == 1.0


def test_speed_offset_scales_executed_velocity():
    r = Randomizer(DRConfig(enabled=True, speed_scale=0.2), random.Random(3))
    r.speed = 1.2                                                # force a known offset, no track noise
    v, w = r.delay_action(0.5, 0.4)
    assert abs(v - 0.6) < 1e-9 and abs(w - 0.48) < 1e-9


def test_lidar_dropout_reads_max_range():
    r = Randomizer(DRConfig(enabled=True, lidar_dropout=1.0), random.Random(4))  # always drop
    assert r.noisy_lidar([0.5, 1.0, 2.0], 5.0) == [5.0, 5.0, 5.0]


def test_lidar_noise_perturbs_but_stays_nonnegative():
    r = Randomizer(DRConfig(enabled=True, lidar_noise=0.05), random.Random(5))
    out = r.noisy_lidar([0.0, 1.0, 2.0], 5.0)
    assert all(v >= 0.0 for v in out) and out != [0.0, 1.0, 2.0]


def test_reset_episode_is_seed_deterministic():
    cfg = default_profile()
    a = Randomizer(cfg, random.Random(42))
    b = Randomizer(cfg, random.Random(42))
    assert (a.latency, a.speed, a.human_speed) == (b.latency, b.speed, b.human_speed)


def test_default_profile_enabled_and_bounded():
    p = default_profile()
    assert p.enabled and 0 < p.speed_scale < 1 and p.action_latency >= 1
