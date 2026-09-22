"""Pure-logic tests for the planner registry (no ROS required)."""

import os

import pytest

from social_nav_benchmarks import planners


def test_known_planners_present():
    names = planners.planner_names()
    assert "social_nav" in names
    assert "rpp" in names


def test_plugin_strings():
    assert planners.plugin_for("social_nav") == "social_nav_controller/SocialNavController"
    assert "RegulatedPurePursuitController" in planners.plugin_for("rpp")
    assert planners.plugin_for("dwb") == "dwb_core::DWBLocalPlanner"
    assert planners.plugin_for("mppi") == "nav2_mppi_controller::MPPIController"


def test_params_filename_matches_name():
    assert planners.params_filename("rpp") == "rpp.yaml"


def test_unknown_planner_raises():
    with pytest.raises(KeyError):
        planners.plugin_for("does_not_exist")


def test_resolve_params_path_exists():
    # Falls back to the in-tree config/planners when ament is not sourced.
    path = planners.resolve_params_path("social_nav")
    assert os.path.isfile(path)
    assert path.endswith("social_nav.yaml")
