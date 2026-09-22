# SocialNav — Parameter & Tuning Guide

A guide to each parameter on the `SocialNavController`, what it does, and which way
to turn it. Change a value, relaunch, and watch the behaviour change.

All parameters live in one file:

```
social_nav_ws/src/social_nav_bringup/config/nav2_params.yaml   ->  section  controller_server:  FollowPath:
```

## How to change a value and see it

The workspace is built with `colcon build --symlink-install`, so the params file is a
**symlink to the source** — edit it and just relaunch, **no rebuild needed**:

```bash
cd ~/Social_planner/social_nav_ws
# 1. edit the value under FollowPath: in
#    src/social_nav_bringup/config/nav2_params.yaml
nano src/social_nav_bringup/config/nav2_params.yaml

# 2. run and watch (the edit is already live)
./scripts/run_demo.sh                          # no people
HUMANS="-1.3,-0.7,0,0" ./scripts/run_demo.sh   # a person on the route
```

> First time only (or after a `git pull`): `colcon build --symlink-install`. If you ever
> see a "failed to create symbolic link … Is a directory" error, it means the build dir was
> made in non-symlink mode — clear and rebuild: `rm -rf build install log && colcon build --symlink-install`.

To quit cleanly: **Ctrl-C** in the demo terminal. If Gazebo ever gets stuck: `./scripts/stop.sh`.

---

## The mental model (read this first)

The controller works in two layers:

1. **Pure-pursuit base** — a reliable "aim at a point ahead on the global path and drive
   toward it" command. This is what makes the robot actually follow the plan. Knobs:
   `desired_linear_vel`, `lookahead_dist`, `curvature_threshold`, `rotate_*`.
2. **Social layer** — the controller simulates a handful of candidate moves *around* that
   base command and picks the one with the lowest **social cost + deviation cost**:
   - **social cost** = how much it intrudes on people (`weights.human`, `weights.ttc`,
     `weights.group`, the `*_social_sigma` sizes).
   - **deviation cost** = how far it strays from the reliable base command
     (`deviation_weight`).

   With **no people**, social cost is 0 everywhere, so it picks the base command → clean
   path following. With people, it trades a little deviation to keep its distance.

On top of that, **behavior modes** scale the whole speed down near people
(NORMAL → CAUTIOUS → CROWDED → EMERGENCY).

---

## Speed knobs

| Param | What it does | Turn UP → | Turn DOWN → |
|---|---|---|---|
| `desired_linear_vel` | Cruise speed on a clear path (m/s) | faster travel | slower, gentler |
| `max_linear_velocity` | Hard speed cap (m/s) | allows higher bursts | caps everything lower |
| `max_angular_velocity` | Hard turn-rate cap (rad/s) | sharper turns | lazier turns |
| `min_approach_speed` | Speed floor while turning through curves | keeps momentum in turns | can crawl/stall in tight turns |

## Path-following knobs (pure-pursuit base)

| Param | What it does | Turn UP → | Turn DOWN → |
|---|---|---|---|
| `lookahead_dist` | How far ahead on the plan it aims (m) | smoother, but **cuts corners** | hugs the path, but can **wobble** |
| `curvature_threshold` | Curvature above which it slows (1/m) | keeps speed in curves | slows earlier for curves |
| `rotate_in_place_threshold` | If the aim point is more than this angle off (rad), spin in place first | turns while driving (may swing wide) | spins in place for sharp turns (tidier, slower) |
| `rotate_gain` | How hard it spins toward the aim point | snappier turns | sluggish turns |

## Social knobs (people)

| Param | What it does | Turn UP → | Turn DOWN → |
|---|---|---|---|
| `weights.human` | How strongly it avoids personal space | gives people a **wider berth** (may stop for a blocker) | passes people closer |
| `weights.ttc` | How strongly it avoids **predicted** collisions with moving people | reacts earlier/harder to crossers | reacts later |
| `weights.group` | How strongly it avoids driving through a group | routes around groups | may clip a group |
| `front_social_sigma` | Personal-space size in **front** of a person (m) | bigger bubble ahead of people | tighter |
| `side_social_sigma` | Personal-space size to the **side** (m) | wider side passing | tighter |
| `rear_social_sigma` | Personal-space size **behind** (m) | more room behind people | tighter |
| `ttc_horizon` | Only worry about collisions within this time (s) | anticipates further ahead | only near-term |
| `ttc_danger_distance` | A predicted pass closer than this counts as dangerous (m) | treats more passes as risky | only very close passes |
| `deviation_weight` | Cost of leaving the reliable base command | **hugs the plan** (slows/stops for a blocker) | **detours more freely** (but too low = wanders) |

## Behavior-mode knobs

Modes scale the whole speed: NORMAL = 1.0, CAUTIOUS = 0.6, CROWDED = 0.4, EMERGENCY = 0.

| Param | What it does |
|---|---|
| `cautious_clearance` | Nearest person closer than this (m) → CAUTIOUS (0.6× speed) |
| `crowded_clearance` | Nearest person closer than this (m) → CROWDED (0.4× speed) |
| `emergency_clearance` | Nearest person closer than this (m) → EMERGENCY (stop) |
| `emergency_ttc` | Predicted collision sooner than this (s) → EMERGENCY (stop) |
| `crowded_num_humans` | This many people or more → at least CROWDED |
| `mode_hysteresis` | How much a reading must clear a threshold before de-escalating, so the mode doesn't flicker |

---

## Recipes — "I want the robot to..."

- **Move faster** → raise `desired_linear_vel` (and `max_linear_velocity` if you hit the cap).
- **Give people more space** → raise `weights.human` and/or the `*_social_sigma` sizes.
- **Slow down sooner near people** → raise `cautious_clearance` / `crowded_clearance`.
- **Stop hugging people's front** → raise `front_social_sigma` (people care most about space ahead of them).
- **Take smoother, wider turns** → raise `lookahead_dist`.
- **Turn more precisely in tight spots** → lower `lookahead_dist`, lower `rotate_in_place_threshold`.
- **Detour around a person instead of stopping** → lower `deviation_weight` so the controller
  is freer to leave the pure-pursuit line. The global planner also routes around people via the
  `SocialLayer` (`social_nav_costs`), which paints anisotropic human cost into the global
  costmap — raise its `weight` to push global paths wider around people.

## Behaviour by scenario

- No people: follows the plan to the goal reliably (~17 s in the demo world).
- Person **beside** the route: passes, keeping clearance.
- Person **standing on** the route: the `SocialLayer` reroutes the global path around them
  (~1.5 m clearance in the demo); if the corridor is too narrow to pass, the controller slows
  and holds at a respectful distance.
- Moving/crossing person: `weights.ttc` makes it slow/yield; tune `ttc_horizon` for how
  early it reacts.

## Try-it scenarios

```bash
# person standing to the side of the route (robot passes, keeping clearance)
HUMANS="-1.0,0.2,0,0" ./scripts/run_demo.sh

# two people = a group near the route
HUMANS="1.2,1.6,0,0" ./scripts/run_demo.sh    # (add more via the humans param, see below)

# a person walking across the route (watch it slow / yield)
HUMANS="0.0,0.4,0,0.3" HUMAN_LOOP=10 ./scripts/run_demo.sh
```

Multiple people directly (bypassing run_demo's single-human shortcut):

```bash
ros2 run social_nav_tools human_publisher --ros-args \
  -p humans:="['-1.0,0.2,0,0', '1.2,1.6,0,0', '1.6,1.4,0,0']"
```
Each entry is `x,y,vx,vy` in the map frame (metres, m/s). `vx=vy=0` = standing still.
