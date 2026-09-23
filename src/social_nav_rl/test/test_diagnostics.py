from social_nav_rl import diagnostics as D


def test_refuses_to_move_and_crawling():
    r = {"success": False, "path_length": 0.1, "navigation_time_s": 30.0, "outcome": "timeout"}
    trig = D.diagnose(r)["triggered"]
    assert "refuses_to_move" in trig and "crawling" in trig and "timeout_no_progress" in trig


def test_excessive_detour_flagged_even_on_success():
    r = {"success": True, "path_length": 25.0, "navigation_time_s": 30.0,
         "straight_dist": 9.0, "outcome": "reached"}
    assert "excessive_detour" in D.diagnose(r)["triggered"]


def test_clean_run_has_no_flags():
    r = {"success": True, "path_length": 9.5, "navigation_time_s": 18.0, "avg_speed": 0.53,
         "straight_dist": 9.0, "outcome": "reached", "oscillations": 1}
    assert D.diagnose(r)["triggered"] == []


def test_oscillation_flag():
    r = {"success": True, "path_length": 12.0, "navigation_time_s": 20.0, "avg_speed": 0.6,
         "straight_dist": 9.0, "outcome": "reached", "oscillations": 40}
    assert "oscillating" in D.diagnose(r)["triggered"]


def test_summarize_rates():
    rs = [{"success": False, "path_length": 0.1, "navigation_time_s": 10, "outcome": "timeout"},
          {"success": True, "path_length": 9.0, "navigation_time_s": 18, "avg_speed": 0.5,
           "straight_dist": 9.0, "outcome": "reached", "oscillations": 1}]
    s = D.summarize(rs)
    assert s["episodes"] == 2 and s["rates"]["refuses_to_move"] == 0.5
