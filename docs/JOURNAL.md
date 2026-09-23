# SocialNav RL — Project Journal

A chronological, honest record of building a reinforcement-learning social-navigation policy: what
we built, what broke, how we diagnosed it, and what we learned. Written so it can be turned into a
technical article or a LinkedIn post. **No fabricated numbers — every figure here came from an
actual run.**

---

## 0. What the project is

A learned local planner for a mobile robot navigating around people. The robot must reach a goal in
cluttered indoor maps (factory, warehouse, office, urban, hospital) while avoiding static obstacles
(racks, walls, furniture) and moving pedestrians (crossing, approaching head-on, stopping suddenly,
changing direction). The comparison baseline is classical reactive navigation (straight-to-goal and
a Social-Force model), and — as a transfer target — real Nav2 in Gazebo.

**Stack:** ROS 2 Humble, Gazebo Classic 11, Python, Gymnasium, Stable-Baselines3 (PPO/SAC),
sb3-contrib (RecurrentPPO), a custom accelerated kinematic simulator ("mock", ~3000–6000 steps/s)
that shares the pedestrian model and scenario registry with Gazebo, plus (new) NVIDIA Isaac Sim 4.5
+ Isaac Lab for high-fidelity, GPU-parallel training.

**Core design:** one env interface, two backends — a fast MockBackend for bulk RL training and a
GazeboBackend that steps the real simulation for transfer testing. Observation, reward, safety, and
action logic are pure and unit-tested (85+ tests).

---

## 1. The first "win" that wasn't (the honesty lesson)

- First trained PPO policy hit **0.84 success** in the accelerated env. Looked great.
- Ran it in Gazebo → **0.00**. It drove straight into pallet racks.
- **Diagnosis (failure replay):** the accelerated env was nearly empty of static obstacles and the
  policy had *no obstacle perception* at all. The 0.84 was an artifact of testing in open space.

**Lesson #1 — an impressive number can be an artifact of an easy test.** Always evaluate in (or
close to) the real target environment. A benchmark that flatters you is worse than no benchmark.

**Fix:** added real static geometry to the maps and a 12-beam down-sampled lidar to the observation,
computed *identically* by ray-cast in the mock and from the real `/scan` in Gazebo, so an
obstacle-aware policy sees the same input shape in both.

---

## 2. The freeze/crawl bug (credit assignment)

After adding obstacles, the policy learned to **freeze or crawl** instead of navigating.

- **Root cause:** the safety supervisor was rewriting the policy's actions *during training*.
  PPO was crediting actions it never actually took → broken credit assignment → the policy learned
  that inching along was "safe."

**Lesson #2 — don't let a safety filter mutate actions during RL.** Train the raw policy
(`--no-safety`), teach safety through the reward's collision penalties, and keep the supervisor as a
*deployment* backstop only.

---

## 3. The matched benchmark, and the honest finding

Built `scripts/rl_benchmark.py`: RL vs Straight vs Social-Force on the *same* obstacle-aware maps and
held-out seeds. Result: **on a fair benchmark, RL ≈ classical.** The earlier 0.84-vs-classical gap
was the open-space artifact, not a real advantage.

**Lesson #3 — a rigorous evaluation is the deliverable, not a leaderboard number.** The scientific
value is showing *where* a learned planner helps and where it doesn't.

(Also fixed a packaging bug where an illegal `--` inside an XML comment made colcon build the
benchmark package as plain Python, hiding its entry points.)

---

## 4. Feasibility: separating "impossible" from "policy failure"

Many "failures" were episodes where **no collision-free path existed** in the time budget (a group
blocking a narrow corridor, a head-on encounter in a 1.7 m aisle). Penalizing the policy for those
is meaningless.

**Built a feasibility oracle** (`feasibility.py`): for each scenario/seed it checks whether *any*
wait-then-go strategy solves it. The benchmark now reports **`success | solvable`** — success only
over episodes that are actually winnable.

This reframed everything: e.g. `approaching` is ~0% feasible everywhere (unavoidable head-on), so
0.00 there is correct, not a failure. And it later revealed the training-instability cause (§8).

**Lesson #4 — measure the ceiling before you grade the policy.** Feasibility-aware metrics turn
noise into signal.

---

## 5. Cheap perception features + the silent obstacle-blindness bug

- Added derived, stateless features on top of the lidar: corridor clearances (front/left/right), a
  lateral "which way is more open" offset, and a crossing time-to-conflict. Obs grew 78 → 86.
- **Found a latent bug:** under multi-scenario / curriculum training the mock backend was rebuilt
  each episode *without its observation config*, so the lidar silently went blind after episode 1.
  Earlier "obstacle-aware" multi-scenario runs had effectively been obstacle-blind.

**Lesson #5 — verify your observation is actually populated during training,** not just at
construction. A silent `None` sensor looks like "clear everywhere."

---

## 6. Domain randomization + system-ID (for transfer)

The mock scored ~1.0 on crossing but Gazebo ~0.2 — a big sim-to-sim gap. Added:
- **Domain randomization** (`randomize.py`, `--domain-rand`): per-episode control latency,
  velocity-tracking noise, speed offset, lidar noise/dropout, crowd-speed variation.
- **System-ID** (`scripts/sysid.py`): drives an identical open-loop command program through both
  backends and fits the real gap (velocity/yaw gain, latency, lidar noise) into DR ranges — so the
  randomization is measured, not guessed.

**Lesson #6 — randomize to bracket the *measured* reality, not arbitrarily.**

---

## 7. The crawler-base disaster (warm-starts inherit their sins)

A long series of retrains kept collapsing to **0.00 success**. Deep dive:

- The curriculum warm-started every stage from an "empty corridor" base. That base had hit its
  `--target-success 0.9` and **stopped at just 75k steps** — as a **crawler** (v ≈ 0.17 m/s). It
  reached the empty goal slowly, so it "succeeded," and quit before ever learning to drive.
- Every downstream stage inherited the crawl → 0.00.

**Fix:** train the base for a *fixed* budget with no early-stop trap. The proper base reached
**+157 mean reward**, drives at full speed across all five maps, and became a solid warm-start.

**Lesson #7 — an early-stopping criterion that's too easy will freeze a bad policy, and every
warm-start inherits it.** Don't let "reached the goal" stand in for "learned the skill."

---

## 8. Algorithms aren't silver bullets; the eval was the real bug

- Tried **SAC** (added `--algorithm sac`) and **RecurrentPPO** (added via sb3-contrib). Neither
  fixed the underlying pipeline/reward/eval issues. RecurrentPPO on CPU ran at **~164 steps/s**
  (vs ~3700 for PPO) — a 20 M-step run would have taken ~34 hours — so we pivoted to **PPO +
  frame-stack 4** for memory of recent motion at 20× the speed.
- **Feasibility-filtered training** (`--require-feasible`): training on impossible episodes was
  feeding PPO noisy negative signal and destabilizing it (a **0.75 → 0.40 collapse**). Skipping them
  stabilized training without faking any geometry.
- **The subtle killer:** the eval / best-checkpoint selection ran on a **single scenario
  (factory/normal)**. So training optimized that one scenario and the "best" model it saved was a
  *specialist* — direction_change and sudden_stop actually got **worse** the longer it trained,
  because they got zero eval feedback. **Fixed the eval to score across the whole distribution**
  (every map × scenario × difficulty), so "best" is chosen as a generalist.

**Lesson #8 — you get the policy your eval selects for.** A single-scenario eval silently breeds a
specialist and can make "more training" actively harmful. This was the highest-leverage fix of the
whole project, and it wasn't an algorithm or a reward — it was *what we measured*.

---

## 9. Making the world honest (maps, density, unpredictable people, waiting)

- **Straight-line-on-empty test:** confirmed every map's static geometry is navigable (straight-line
  reaches the goal on an empty corridor in all 5). So low success was people, not broken maps.
- **Per-map crowd density** (`human_scale`): narrow maps (office/urban/hospital, ~2.8 m corridors)
  were packing the *same* 4-person crowd as the wide warehouse (6.4 m). Scaling people to corridor
  width raised feasibility from **0.30–0.50 → 0.80–0.85** — fair, without touching geometry.
- **Unpredictable pedestrians** (`--human-unpredictable`): added smooth non-linear "wander" so
  people don't walk in perfectly predictable straight lines, breaking the constant-velocity
  prediction and forcing the policy to *react* rather than extrapolate.
- **Waiting is allowed** (conditional `stopping` penalty): the reward used to penalize the robot for
  stopping — which discouraged the correct behavior of *waiting* for a group to pass in a tight
  aisle. Made the penalty not fire when a person is close (yielding ≠ dawdling).

**Lesson #9 — align the simulator and the reward with the behavior you actually want.** "Don't
dawdle" quietly forbade "yield to a group." Realistic, width-matched crowds and non-linear motion
matter as much as the algorithm.

---

## 10. Final honest result — the generalist

Model: PPO + frame-stack 4, trained across all 5 maps × 4 feasible scenarios × 3 difficulties, with
feasibility filtering, unpredictable humans, and a multi-scenario generalist eval. Best generalist
eval score **0.54** (honest, multi-map, unpredictable-human — the earlier "0.90" was the
single-scenario mirage).

Per-environment benchmark, **success on solvable episodes (`succ|solv`)**, medium difficulty,
20 episodes/cell. RL = our generalist, vs Straight-line and Social-Force (SFM):

| Env | Scenario | Straight | SFM | **RL** |
|---|---|---|---|---|
| factory | normal | 0.50 | 0.71 | **0.86** |
| factory | crossing | 1.00 | 1.00 | **0.00** |
| factory | direction_change | 1.00 | 0.00 | **1.00** |
| warehouse | normal | 0.50 | 0.86 | **0.71** |
| warehouse | crossing | 1.00 | 1.00 | **1.00** |
| warehouse | direction_change | 1.00 | 1.00 | **1.00** |
| urban | normal | 0.53 | 0.65 | **0.35** |
| urban | crossing | 1.00 | 1.00 | **1.00** |
| urban | **sudden_stop** | 0.00 | 0.00 | **1.00** |
| urban | direction_change | 1.00 | 1.00 | **0.00** |
| office | normal / crossing | 0.00 | 0.00 | **0.00** |
| hospital | normal / crossing | 0.00 | 0.00 | **0.00** |

(`approaching` and `dense_crowd` are ~0% feasible everywhere — excluded as unsolvable. Full CSVs in
`~/social_nav_rl_results/benchmark/`.)

**What the table says, honestly:**
- **RL wins or ties** on the aisle maps it resembles: factory normal (best, 0.86), warehouse normal,
  crossing and direction_change on factory/warehouse.
- **RL's standout:** urban `sudden_stop` = **1.00** while both baselines are 0.00 — it learned to
  **stop and wait** for a pedestrian, which reactive planners can't. Direct payoff of the
  waiting-reward fix (§9).
- **RL keeps more social distance** — larger clearance to people almost everywhere (e.g. ~2 m on
  head-on approaches vs ~0.75 m for baselines).
- **RL loses / is inconsistent:** factory crossing regressed to 0.00 (yet warehouse crossing is
  1.00 — map-to-map inconsistency), urban normal below baselines, and direction_change fails on the
  narrow maps.
- **Every method fails office/hospital** normal/crossing — narrow, furniture-filled corridors are
  hard for classical *and* learned planners alike.

**Verdict:** a credible generalist, not a dominant one. It matches or beats classical on the maps it
was built for, does something classical can't (yield/wait), and keeps better social distance — but
it doesn't win across the board, and the hardest maps defeat everyone. The honest deliverable is the
evaluation itself: feasibility-aware, multi-map, showing exactly where a learned social planner helps
and where it doesn't.

---

## 11. Infrastructure & tooling built along the way

- Two-backend Gym env (fast mock + Gazebo), pure unit-tested core (85+ tests).
- Early-stopping trainer that always keeps the best checkpoint (a late collapse can't destroy a run).
- Feasibility oracle + feasibility-aware benchmark.
- Domain randomization + a system-ID harness.
- Multi-environment / multi-difficulty / multi-scenario training and a matching **generalist eval**.
- Per-map crowd density, non-linear pedestrian motion, waiting-aware reward.
- `--algorithm {ppo,sac,recurrent_ppo}`, `--frame-stack`, warm-start, LR/entropy overrides.

---

## 12. Isaac Sim port — the same robot, GPU-parallel, in a real prebuilt scene

Took the learned-policy pipeline into **NVIDIA Isaac Sim 4.5 + Isaac Lab** (companion scripts in
`~/isaac`), on a laptop **RTX 4050 (6 GB)**:

- **Imported the same Gazebo MiR** (URDF → USD via Isaac's importer). One real gotcha: USD prim names
  can't contain a hyphen, and the importer names a prim after each mesh file — so `sick_lms-100.stl`
  aborted the whole import until the mesh was renamed hyphen-free.
- **Built a manager-based social-nav task** (`ManagerBasedRLEnvCfg`): Isaac's own **open-source
  prebuilt `Simple_Warehouse.usd`** as the world, the MiR as the agent, a custom kinematic diff-drive
  action, moving pedestrians, goal command, and a social reward (goal progress + clearance +
  collision).
- **Trained it GPU-parallel** with rsl_rl PPO: reward 79.5, goal error 0.27 m, **zero pedestrian
  collisions** over 1.54M steps in ~13 min. Same goal-vs-avoidance tuning lesson from the Gazebo side
  replayed here (undertraining + weak goal pull → hedging), fixed the same way.
- **Findings worth keeping:** Isaac's detailed "People" characters are rigged/instanced and **cannot**
  be tensorized GPU physics bodies — the training pedestrians must be simple shapes (capsules), with
  detailed humans reserved for a rendered demo. And a detailed-warehouse **collision** mesh is pure
  cost here (the robot drives kinematically and never touches it) — leaving it on crashed the 6 GB
  GPU's PhysX narrowphase under GUI; making it visual-only fixed that and sped training.

**Takeaway:** the accelerated Gazebo/mock sim is still the right place to *train* (thousands of
steps/s vs full-physics ~real-time); Isaac's value is high-fidelity scenes + GPU-parallel envs, and
the port proves the same robot and task move over cleanly.

---

## 13. Hooks for the write-up (article / LinkedIn)

- "My RL robot scored 0.84 in sim and 0.00 in the real simulator. Here's what that taught me about
  honest benchmarking."
- "The bug wasn't in my algorithm, my reward, or my robot — it was in what I chose to *measure*."
  (single-scenario eval → specialist, §8)
- "A reward that said 'don't dawdle' quietly forbade the robot from yielding to a group. Sometimes
  the fix is one conditional." (§9)
- "Feasibility-aware evaluation: stop grading your policy on problems that have no solution." (§4)
- "The honest result: my learned social planner matched classical navigation — and the *evaluation*
  was the real contribution." (§10)
- Concrete win to show: **urban sudden_stop, RL 1.00 vs classical 0.00** — the robot learned to wait.
