"""Registry of benchmark planner profiles for baseline comparison.

Each profile is a full nav2 params file under ``config/planners/`` that differs from the
next only in ``controller_server.FollowPath.plugin`` (and that controller's own tuning);
the costmaps, global planner, behaviours and BT are identical, so a comparison isolates the
LOCAL controller. The runner selects a profile by name and points the launch file at it via
the ``SOCIAL_NAV_PARAMS`` environment variable (see navigation.launch.py).

The registry itself (names, plugin strings, filenames) is pure Python so it can be unit
tested without ROS. Only path resolution and the installed-package check touch the ament
index, and both fail soft.
"""

import os

# name -> (params filename, controller plugin string, ROS package that provides the plugin)
PLANNERS = {
    "social_nav": (
        "social_nav.yaml",
        "social_nav_controller/SocialNavController",
        "social_nav_controller",
    ),
    "rpp": (
        "rpp.yaml",
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "nav2_regulated_pure_pursuit_controller",
    ),
    "dwb": (
        "dwb.yaml",
        "dwb_core::DWBLocalPlanner",
        "dwb_core",
    ),
    "mppi": (
        "mppi.yaml",
        "nav2_mppi_controller::MPPIController",
        "nav2_mppi_controller",
    ),
}


def planner_names():
    """All registered planner names."""
    return list(PLANNERS.keys())


def plugin_for(name):
    """The controller plugin string a profile selects."""
    return PLANNERS[_require(name)][1]


def params_filename(name):
    """The bare params filename for a profile (no directory)."""
    return PLANNERS[_require(name)][0]


def _require(name):
    if name not in PLANNERS:
        raise KeyError(
            f"unknown planner '{name}'. Known: {planner_names()}")
    return name


def _config_dir():
    """Directory holding the profile YAMLs.

    Prefer the installed share directory (populated by setup.py data_files); fall back to
    the in-tree source so the runner also works from a plain checkout / editable install.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("social_nav_benchmarks")
        cand = os.path.join(share, "config", "planners")
        if os.path.isdir(cand):
            return cand
    except Exception:  # noqa: BLE001 - ament not sourced / package not installed yet
        pass
    # <pkg>/social_nav_benchmarks/planners.py -> <pkg>/config/planners
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "config", "planners"))


def resolve_params_path(name):
    """Absolute path to a profile's params file. Raises if the file is missing."""
    path = os.path.join(_config_dir(), params_filename(name))
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"params file for planner '{name}' not found at {path}. "
            "Build the package (colcon build) so config/planners is installed.")
    return path


def is_installed(name):
    """True if the ROS package providing this planner's controller is discoverable.

    Fails soft to False if the ament index cannot be queried (e.g. not sourced).
    """
    pkg = PLANNERS[_require(name)][2]
    try:
        from ament_index_python.packages import get_package_prefix

        get_package_prefix(pkg)
        return True
    except Exception:  # noqa: BLE001 - PackageNotFoundError or ament unavailable
        return False


def validate(names):
    """Return the requested planner names after checking each is known and installed.

    Raises KeyError for an unknown name and RuntimeError for a known-but-not-installed one,
    so a comparison never silently skips a baseline the user asked for.
    """
    checked = []
    for name in names:
        _require(name)
        if not is_installed(name):
            raise RuntimeError(
                f"planner '{name}' needs package "
                f"'{PLANNERS[name][2]}' which is not installed / sourced.")
        checked.append(name)
    return checked
