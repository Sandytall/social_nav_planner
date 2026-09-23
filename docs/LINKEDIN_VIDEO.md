# LinkedIn Video — Storyboard & Script

A ~90-second technical showcase of the RL social-navigation work. The angle is **rigorous ML
engineering and honest evaluation**, not a hype "RL beats everything" reel — that reads far stronger
to a robotics/ML audience, and it's what the results actually support.

Everything below is recordable from this repo. Suggested total length 75–100 s.

---

## Post caption (paste into LinkedIn)

> I trained a reinforcement-learning local planner for social robot navigation in ROS 2 + Gazebo —
> a robot getting to its goal down a factory aisle shared with people.
>
> The interesting part wasn't the training. It was what happened when I stopped trusting the number.
>
> My accelerated sim said the policy hit ~0.84 success. Then I ran it in Gazebo and it scored 0.00 —
> it drove straight into pallet racks. The "great" result was measured in an environment with almost
> no obstacles and a policy that literally couldn't see them.
>
> So I rebuilt the evaluation to be fair: real static geometry in the training sim, a lidar in the
> observation, matched scenarios and seeds against classical baselines (a Social-Force planner and a
> straight-line controller). The honest finding: on a fair benchmark the learned policy is *on par*
> with classical reactive planners — and the hard head-on cases in a 1.7 m aisle are unsolved by
> everything. No free lunch.
>
> The deliverable I'm proud of isn't the policy — it's the benchmark that tells the truth about it.
>
> Stack: ROS 2 Humble, Gazebo Classic, PPO (Stable-Baselines3), custom Gymnasium env with an
> accelerated backend + a live-sim backend behind one interface.
>
> #robotics #reinforcementlearning #ROS2 #machinelearning #simulation #navigation

---

## Video structure & shot list

Record screen clips first, then narrate/caption over them. Captions matter — most LinkedIn video is
watched muted.

**0:00–0:08 — Hook (Gazebo clip).**
Show the robot driving down the factory aisle weaving past a pedestrian.
Caption: *"An RL robot learning to walk through a crowd — and the number that lied to me."*
Record with:
```
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch social_nav_bringup rl_sim.launch.py environment:=factory scenario:=crossing difficulty:=medium gui:=true
# then in another terminal, the RL eval (see docs/COMMANDS.md, "Run it in Gazebo")
```

**0:08–0:20 — The problem.**
Screen: the RViz/Gazebo aisle with racks + a person. Caption:
*"Task: reach the goal down a 1.7 m factory aisle shared with people. Reward safety AND progress."*

**0:20–0:38 — The trap (the story beat).**
Screen: split — training log showing `success ~0.84` (accelerated sim) next to the first Gazebo run
ending in a rack collision. Captions:
*"Accelerated sim: 0.84 success."* → *"Same policy in Gazebo: 0.00 — it hit racks it couldn't see."*
*"The training world had almost no obstacles, and the policy had no obstacle sensor."*

**0:38–0:55 — The fix (engineering montage).**
Screen: quick cuts — the world→obstacle extractor output (21 boxes), the 12-beam lidar debug line,
the `[inputs] odom=OK scan=OK humans=2 lidar=OK` log, a `--debug` eval showing the robot reacting to
an approaching person. Captions:
*"Added the real geometry to the training sim + a lidar the policy can see."*
*"Retrained bottom-up: empty aisle → 2 people → 4 people. Trained without the safety filter so the
policy actually learns to steer, not lean on it."*

**0:55–1:12 — The honest result (benchmark).**
Screen: the table from `docs/EVALUATION_REPORT.md` §4 (RL vs Social-Force vs straight). Captions:
*"Fair benchmark, matched scenarios + seeds, vs classical planners."*
*"On solvable scenarios: RL is on par with classical. Head-on cases in the tight aisle: nobody solves
them. No free lunch — and that's the point."*

**1:12–1:25 — Payoff + takeaway (Gazebo clip again).**
Screen: the obstacle-aware policy navigating the aisle in Gazebo, keeping distance from a person.
Captions:
*"The win: a policy trained in an accelerated sim that transfers to Gazebo — and a benchmark that
tells the truth about when learning actually helps."*
*"Built with ROS 2, Gazebo, PPO. Full write-up + reproducible benchmark in the repo."*

---

## B-roll / assets to capture

- 2–3 Gazebo clips: crossing (medium), a clean success, and one near-miss where it yields.
- The `--debug` terminal output scrolling (humans_seen / nearest / lidar_min changing).
- The `[inputs]` log line (proves the sensors are live — nice credibility detail).
- The benchmark table (screenshot `docs/EVALUATION_REPORT.md` §4, or the CSV).
- Optional: the failure replay of the obstacle-blind run hitting a rack (great "before" shot).

## Talking points (if you narrate)

- "The headline number came from an over-easy environment — I only caught it by running the real sim."
- "I rebuilt the evaluation to be fair before I trusted any comparison."
- "Trained bottom-up and without the safety filter, because the filter was silently teaching the
  policy to freeze."
- "The honest result is that learning is competitive, not magic — and knowing *where* it helps is the
  actual engineering value."

## Tone

Confident, precise, no overclaiming. The strongest possible impression here is: *this person measures
carefully, catches their own mistakes, and reports honestly.* That is exactly what robotics/ML teams
hire for.
