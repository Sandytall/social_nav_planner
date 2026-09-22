"""Deterministic benchmark scenarios (MASTER_PROMPT §28), sized for the SocialNav demo
world (10x10 m, corridor gap y in [0,2] at x=0, robot spawns at (-3.5,-3.5, yaw 0.9)).

Each scenario: humans as "x,y,vx,vy" (map frame, m and m/s; vx=vy=0 => stationary),
a goal, an expectation string, a per-run timeout, and a seed for reproducibility (§39).
"""

GOAL = (2.5, 2.5)

SCENARIOS = {
    "empty": dict(
        humans=[], goal=GOAL, timeout=60, seed=0,
        expect="Behaves like a normal local planner; reaches goal."),
    "stationary_side": dict(
        humans=["-1.0,0.2,0,0"], goal=GOAL, timeout=60, seed=1,
        expect="Passes a person beside the route with social clearance."),
    "blocker_on_path": dict(
        humans=["-1.3,-0.7,0,0"], goal=GOAL, timeout=75, seed=2,
        expect="Routes AROUND a person standing on the path (SocialLayer)."),
    "crossing": dict(
        humans=["0.2,-0.5,0,0.35"], goal=GOAL, timeout=75, seed=3,
        expect="Slows/yields to a person crossing the path."),
    "head_on": dict(
        humans=["0.4,1.4,-0.35,-0.2"], goal=GOAL, timeout=75, seed=4,
        expect="Detects the approaching person (TTC) and gives way."),
    "same_direction": dict(
        humans=["0.0,1.0,0.25,0.25"], goal=GOAL, timeout=75, seed=5,
        expect="Follows/overtakes without excessive caution."),
    "group": dict(
        humans=["1.3,1.7,0,0", "1.7,1.5,0,0"], goal=GOAL, timeout=75, seed=6,
        expect="Treats the pair as a group and goes around it."),
    "corridor_pair": dict(
        humans=["-1.8,-1.4,0,0", "-1.2,-0.2,0,0"], goal=GOAL, timeout=90, seed=7,
        expect="Negotiates two people on the way to the corridor, keeping clearance."),
}


def scenario_names():
    return list(SCENARIOS.keys())


def get_scenario(name):
    if name not in SCENARIOS:
        raise KeyError(f"unknown scenario '{name}'. Known: {scenario_names()}")
    return SCENARIOS[name]
