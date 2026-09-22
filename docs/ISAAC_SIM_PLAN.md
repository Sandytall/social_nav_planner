# Isaac Sim Showcase Track — Plan

Goal: a **professional-looking demo** of the SocialNav planner in a realistic street
environment with animated pedestrians, for the final video (§60). Isaac Sim is a
*swappable simulation backend* — the SocialNav planner, controller, and SocialLayer are
**unchanged**; Isaac just replaces Gazebo as the physics/render/sensor source.

## Reality check (read first)

| Isaac Sim 4.5 minimum | This machine | Verdict |
|---|---|---|
| GPU 8 GB VRAM (RTX 3070) | RTX 4050 Laptop **6 GB** | **below minimum** |
| 32 GB RAM | 23 GB | below recommended |
| 50 GB free SSD | 67 GB | ok (tight) |
| Ubuntu 22.04, driver 535+ | 22.04, 575 | ok |

**6 GB VRAM is below spec, and street + crowd scenes are the heaviest.** Isaac may launch
and handle a simple scene, but a full street + pedestrians could stutter, OOM, or freeze
the machine. Therefore this plan is **gated by a compatibility spike** — do not invest in
the full integration until the spike passes.

Interfaces we must reproduce from Isaac (everything downstream already consumes these):
`/scan` (LaserScan), `/odom` (Odometry), `/cmd_vel` (Twist, in), `/clock`, TF
`map->odom->base_footprint->...`, and `/social_nav/humans` (from Isaac pedestrians).

---

## Phase 0 — Compatibility spike (GO/NO-GO gate) ⏱️ ~1–2 h

Prove Isaac Sim runs *at all* on this GPU before building anything.

1. **Run NVIDIA's compatibility checker** (from the Isaac Sim requirements page) — it
   reports pass/warn/fail for this exact machine.
2. **Install Isaac Sim 4.5** (pick one; see "Install" below).
3. **Launch, load a simple scene** (an empty stage + a ground plane, or the built-in
   `simple_room`). Watch VRAM with `nvidia-smi -l 1` in another terminal.
4. **Add a small street/warehouse asset** from the Isaac asset browser and watch VRAM/fps.

**GO** if: it loads a modest scene, stays under ~5.5 GB VRAM, and runs ≥ ~15 fps.
**NO-GO** if: it OOMs, freezes the machine, or a street scene is < ~5 fps.

> **NO-GO fallback (still "professional streets", guaranteed to run on 6 GB):** keep our
> Gazebo Classic stack and drop in a city/street world (e.g. an OSM-generated world or a
> free Gazebo city model) + the existing pedestrian publisher. ~½ day, zero planner
> changes, no new heavy dependency. I can do this immediately if the spike fails.

---

## Install (for the spike) — Python 3.10, Ubuntu 22.04

Follow the official page for the exact current command (it changes between releases):
`https://docs.isaacsim.omniverse.nvidia.com/latest/installation/`. Two options:

- **pip (fastest to try):**
  ```bash
  python3.10 -m venv ~/isaacsim_venv && source ~/isaacsim_venv/bin/activate
  pip install --upgrade pip
  pip install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com
  isaacsim            # first launch downloads shaders/assets (slow, one-time)
  ```
- **Binary workstation install** (better for the GUI/scene editing): download the Isaac Sim
  4.5 Linux zip from the NVIDIA download page, extract, run `./isaac-sim.sh`.

Notes: needs a (free) NVIDIA account; first launch compiles shaders (minutes); set
`export OMNI_KIT_ACCEPT_EULA=YES` for headless.

---

## Phase 1 — Bring the MiR into Isaac ⏱️ ~½ day  *(only if Phase 0 = GO)*

1. Import `mir_social.urdf.xacro` via Isaac's **URDF Importer** (xacro → urdf first:
   `xacro mir_social.urdf.xacro > mir.urdf`). Set base_footprint as the base, wheels as
   revolute, fix the mesh paths (Isaac reads the file:// paths we already use).
2. Add a **differential drive controller** (Isaac `DifferentialController` / articulation)
   driven by `/cmd_vel`.
3. Add an **RTX Lidar** on `front_laser_link` → publish `/scan`; publish `/odom` + TF from
   the base; publish `/clock`.

## Phase 2 — ROS 2 bridge ⏱️ ~½ day

1. Enable the `omni.isaac.ros2_bridge` extension (Humble). Configure RMW to match our
   system Humble, or use Isaac's bundled Humble libs.
2. Wire OmniGraph action-graph nodes to publish/subscribe exactly:
   `/cmd_vel` (sub) · `/scan` · `/odom` · `/tf` · `/clock`.
3. Verify with `ros2 topic echo` that each matches what Nav2 expects (frames, rates).

## Phase 3 — Street scene + pedestrians ⏱️ ~½–1 day

1. Load a street/city environment (Isaac sample env or an Omniverse asset). Keep it small
   enough for 6 GB.
2. Add **animated pedestrians** via `omni.anim.people`. Publish their world poses to
   `/social_nav/humans` (a small OmniGraph or Python node) — feeds the SocialLayer directly.
3. Provide a static `map->odom` TF (or Isaac odom) so our mapless Nav2 config works.

## Phase 4 — Run SocialNav against Isaac ⏱️ ~½ day

1. Launch our Nav2 (`navigation.launch.py`) with `use_sim_time:=true` pointing at Isaac's
   `/clock`. No controller/SocialLayer changes.
2. Send a `NavigateToPose` goal; confirm the robot follows the plan, and the SocialLayer
   routes it around the Isaac pedestrians.
3. Record the video (§60).

---

## Effort & recommendation

- **Spike: ~1–2 h** (do this first, today).
- **Full integration if GO: ~2–3 days** (URDF import, ROS 2 bridge graph, scene, peds).
- Keep **Gazebo Classic as the primary dev/CI/benchmark sim** (headless, cheap, reproducible
  — §29 needs hundreds of runs). Isaac is the *showcase* backend only.

**Bottom line:** worth trying for the wow-factor video, but the 6 GB GPU is the gating risk.
Run Phase 0 first; if it can't hold a street scene, the Gazebo-city fallback gives you
"streets" today with zero risk.
