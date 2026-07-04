# WildSeed worlds — procedural terrain for the Husky sim

[WildSeed](https://github.com/ricardodeazambuja/WildSeed) generates seeded,
byte-reproducible outdoor Gazebo-Harmonic worlds (procedural terrain, 8 biomes,
CC0 assets, per-instance ground truth). This repo can swap its Clearpath
`pipeline` world for any WildSeed world, so M3/M4/M5 metrics can be run across
many terrains (ATE *distribution* vs terrain complexity) instead of one
hand-made world. Same seed ⇒ same world ⇒ reproducible runs.

WildSeed stays a **separate repo** (AGPL-3.0): we consume only its *generated
worlds* (plain SDF + meshes — data, not linked code), which keeps the license
boundary clean.

## Workflow

```bash
# 1. Generate a world in the WildSeed checkout (see its README; needs its
#    deps or docker image — generation does NOT need a GPU, only rendering).
cd ~/GitStuff/WildSeed
wildseed scenario --seed 42                # or generate / terraingen+ground+generate

# 2. Package it for this repo (host-side; snapshots world+models TOGETHER —
#    WildSeed overwrites models/ground per run, so bundle right after generating).
cd ~/GitStuff/WildSeedAutonomy
./scripts/prepare_wildseed_world.sh ~/GitStuff/WildSeed \
    ~/GitStuff/WildSeed/worlds/scenario_42.world wildseed_42 --spawn "0,0,0"

# 3. Point the sim at the bundle and restart.
./scripts/deploy.sh world wildseed_42
./scripts/deploy.sh restart compute        # or: up compute

# 4. Gate it before trusting metrics from it.
./scripts/deploy.sh m3-smoke

# back to the Clearpath world:
./scripts/deploy.sh world default
```

## What a bundle is

`prepare_wildseed_world.sh` writes `worlds_external/<name>/` (gitignored):

| file | what |
|------|------|
| `world.sdf` | the world, **shell-injected** (`wildseed rig --inject --shell-only --no-labels`): Sensors(ogre2)/Imu/NavSat/AirPressure/Magnetometer systems + `<spherical_coordinates>`; any WildSeed `sensor_rig` include is stripped (the Husky is the robot); then **RTF-tuned** (`tune_world_bundle.sh`: leftover Label plugins stripped, physics step → 0.002 — see the RTF wall below) |
| `models/` | only the model categories the world references, hardlinked from the WildSeed checkout when same-filesystem (≈0 extra disk) |
| `spawn.json` | `{x, y, z, yaw, world_name}` — `z` sampled from the terrain mesh (`wildseed height`) + 0.3 m clearance, because WildSeed terrain is NOT flat and Clearpath's default `z=0.15` would bury or drop the robot |

Bundles prepared **before** the RTF tuning existed migrate in place:
`./scripts/tune_world_bundle.sh <bundle> --step 0.002` (idempotent), or just
re-run `prepare_wildseed_world.sh`.

## How the pieces connect

- `.env`: `deploy.sh world <bundle>` sets `SIM_WORLD` (bundle name) **and**
  `SIM_WORLD_NAME` (the `<world name>` inside the SDF). Always set them
  together — `SIM_WORLD_NAME` drives the gtbridge ground-truth topic
  `/world/<name>/dynamic_pose/info`, and a mismatch silently kills ATE/RPE
  ground truth.
- `docker/compose.yaml`: mounts `../worlds_external:/worlds_external:ro` into
  the husky service; `SIM_WORLD_NAME` is in the shared env anchor so scripts in
  any container (e.g. `m3_vio_demo.py`) resolve the right world topics.
- `husky_sim.launch.py`: when `SIM_WORLD` is set it loads
  `/worlds_external/$SIM_WORLD/world.sdf`, prepends the bundle's `models/` to
  `GZ_SIM_RESOURCE_PATH`, parses the world name for the world-ready poll, and
  passes `spawn.json`'s `x,y,z,yaw` to Clearpath's `robot_spawn.launch.py`.
  Unset/empty ⇒ the original `pipeline` behaviour, unchanged.

## Gotchas

- **Dense worlds collapse the real-time factor — and RTF < ~0.1 kills
  controller activation.** gz_ros2_control's sim-time-paced activation
  handshake starves (`Failed to activate controller:
  platform_velocity_controller`, no odom, no drive); the launch's
  `controller_watchdog` self-heals it and `SLOW_SIM_FACTOR` widens the wall
  budgets ([operations.md](operations.md) "Slow machines / low RTF"). Check
  RTF any time: `./scripts/deploy.sh rtf` (or the `[rtf_probe]` launch log).
  **Where the RTF actually goes — measured** (`scripts/bench_rtf.sh`,
  RTX 2070 Max-Q laptop; mean ±sd of 10 samples):

  | world | variant | RTF |
  |---|---|---|
  | wildseed_42 (330 includes) | robot, untuned | 0.310 ±0.03 |
  | wildseed_42 | world-only (no robot) | 0.591 ±0.16 |
  | wildseed_42 | robot, shadows off | 0.280 ±0.05 — **no-op** |
  | wildseed_42 | world-only, labels stripped | 0.680 ±0.12 — within noise |
  | wildseed_42 | robot, **tuned** (step 2 ms + lidar 512×32) | **0.5–0.66**, m3-smoke PASS |
  | forest (2,849 includes) | world-only | 0.034 ±0.01 |
  | forest | world-only, shadows off / labels stripped | 0.034 / 0.035 — **no-ops** |
  | forest | world-only, **step 4 ms** | **0.149 ±0.05** (~linear in step) |

  Conclusions: (1) dense worlds are **physics-step-bound** — density
  (`wildseed scenario --density-scale`) and `max_step_size` are the levers,
  shadows/sky/Label plugins are not; (2) with the robot present its **render
  sensors cost ~half the throughput** — hence the OS1 512×32 default in
  `Dockerfile.sim`. `prepare_wildseed_world.sh` bakes the evidence in
  (labels stripped + step 0.002 by default, via `tune_world_bundle.sh`).
- **Heavy worlds load slowly.** A dense WildSeed world is thousands of mesh
  includes; the world-ready poll backstop is 180 s. If the spawn still races,
  check `deploy.sh logs husky` for the `/world/<name>/create` service.
- **Rendering needs the real GPU** (ogre2/EGL) — same M3 wall as always: if
  cameras give solid-colour frames, check `GL_RENDERER` first
  (sim-debugging-notes #8).
- **Bundle right after generating.** `wildseed scenario/terraingen` overwrite
  `models/ground`; a world bundled against a *newer* terrain mesh gets wrong
  spawn heights and mismatched ground.
- **Terrain slope is capped at generation (WildSeed ≥ `f1abe58`).** Scenario
  worlds rescale relief to a mean surface slope of 20° (`--max-slope`, 0 =
  off): older alpine seeds drew amplitude ≈ feature wavelength → mean mesh
  slope 52°, >90 % of the map steeper than the Husky's ~20–25° gradeability —
  the robot terrain-traps in the first gully. Worlds generated before the cap
  must be **regenerated** (same seed → same layout, drivable relief; seed 42:
  <25°-slope area 9 % → 76 %, drive progress 0.63 → 2.7 m per 12 s, m3-smoke
  PASS). Diagnose a suspect world with the collision mesh: mean facet slope
  over ~30° ⇒ regenerate.
- **`<scale>` include warnings** at world load are expected (SDF `<include>`
  has no `<scale>`; gz copies it through) — they come from WildSeed's placement
  scaling and are harmless.
