"""Tests for the small pure modules: curriculum, latency, replay, checkpoint, hybrid, policies."""
import math
import random

import numpy as np
import pytest

from social_nav_rl.checkpoint import CheckpointMeta, load_meta
from social_nav_rl.curriculum import SEED_SPLITS, Curriculum
from social_nav_rl.hybrid import HybridConfig, HybridController
from social_nav_rl.latency import LatencyMeter
from social_nav_rl.policies import ConstantPolicy, StraightToGoalPolicy
from social_nav_rl.replay import FailureReplay, load


# --- curriculum ---
def test_curriculum_sample_in_level():
    c = Curriculum(split="train")
    ep, seed = c.sample_episode(random.Random(0))
    lv = c.current()
    assert ep.environment in lv["envs"] and ep.scenario in lv["scenarios"]
    lo, hi = SEED_SPLITS["train"]
    assert lo <= seed < hi


def test_curriculum_advances_on_success():
    c = Curriculum(split="train", window=5, advance_threshold=0.8)
    advanced = any(c.record(True) for _ in range(5))
    assert advanced and c.level == 1


def test_curriculum_holds_on_failure():
    c = Curriculum(window=5)
    for _ in range(5):
        c.record(False)
    assert c.level == 0


def test_seed_splits_disjoint():
    assert SEED_SPLITS["train"][1] <= SEED_SPLITS["val"][0] <= SEED_SPLITS["val"][1] \
        <= SEED_SPLITS["test"][0]


def test_bad_split_raises():
    with pytest.raises(ValueError):
        Curriculum(split="nope")


# --- latency ---
def test_latency_summary():
    m = LatencyMeter()
    assert m.summary()["count"] == 0
    for x in [1, 2, 3, 4, 5]:
        m.record(x)
    s = m.summary()
    assert s["count"] == 5 and s["max"] == 5.0 and s["p50"] == 3.0
    with m.measure():
        pass
    assert m.summary()["count"] == 6


# --- replay ---
def test_replay_roundtrip(tmp_path):
    r = FailureReplay("urban", "crossing", "hard", 42, [9.0, 0.0])
    r.add(t=0, action=[0.1, 0.0], events={"reached": False})
    r.finalize("collision")
    d = load(r.save(str(tmp_path / "f.json")))
    assert d["outcome"] == "collision" and d["seed"] == 42 and len(d["steps"]) == 1


# --- checkpoint ---
def test_checkpoint_meta(tmp_path):
    mp = str(tmp_path / "model.zip")
    CheckpointMeta(environment="factory", training_steps=1000, seed=7).save(mp)
    d = load_meta(mp)
    assert d["environment"] == "factory" and d["training_steps"] == 1000 and d["project_version"]


# --- hybrid ---
def test_hybrid_blend():
    (v, w), a = HybridController(HybridConfig(alpha=0.5, risk_ttc=2.0)).combine(
        (1.0, 0.0), (0.0, 0.0), min_ttc=100.0)
    assert a == 0.5 and abs(v - 0.5) < 1e-9


def test_hybrid_defers_under_risk():
    (v, w), a = HybridController(HybridConfig(alpha=0.9, risk_ttc=2.0)).combine(
        (1.0, 1.0), (0.2, 0.0), min_ttc=0.5)
    assert a == 0.0 and v == 0.2


# --- policies ---
def test_straight_turns_toward_goal():
    obs = np.zeros(80, dtype=np.float32)
    obs[5] = 1.0                       # goal bearing to the left
    assert StraightToGoalPolicy().act(obs)[1] > 0.0


def test_constant_policy():
    assert ConstantPolicy(-1.0, 0.0).act(np.zeros(10))[0] == -1.0
