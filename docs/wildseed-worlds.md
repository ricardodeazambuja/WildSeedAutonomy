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
| `world.sdf` | the world, **shell-injected** (`wildseed rig --inject --shell-only`): Sensors(ogre2)/Imu/NavSat/AirPressure/Magnetometer systems + `<spherical_coordinates>`; any WildSeed `sensor_rig` include is stripped (the Husky is the robot) |
| `models/` | only the model categories the world references, hardlinked from the WildSeed checkout when same-filesystem (≈0 extra disk) |
| `spawn.json` | `{x, y, z, yaw, world_name}` — `z` sampled from the terrain mesh (`wildseed height`) + 0.3 m clearance, because WildSeed terrain is NOT flat and Clearpath's default `z=0.15` would bury or drop the robot |

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
  controller activation.** Measured on the laptop (RTX 2070 Mobile, 8-CPU
  cap): the dense `forest_world` demo (2,850 includes) runs at **RTF 0.04**,
  and gz_ros2_control's sim-time-paced activation handshake starves —
  `Failed to activate controller: platform_velocity_controller`, no odom, no
  drive. Use `wildseed scenario` worlds (~600–700 includes) for closed-loop
  runs and check RTF after load:
  `gz topic -e -n1 -t /world/<name>/stats | grep real_time_factor`.
- **Heavy worlds load slowly.** A dense WildSeed world is thousands of mesh
  includes; the world-ready poll backstop is 180 s. If the spawn still races,
  check `deploy.sh logs husky` for the `/world/<name>/create` service.
- **Rendering needs the real GPU** (ogre2/EGL) — same M3 wall as always: if
  cameras give solid-colour frames, check `GL_RENDERER` first
  (sim-debugging-notes #8).
- **Bundle right after generating.** `wildseed scenario/terraingen` overwrite
  `models/ground`; a world bundled against a *newer* terrain mesh gets wrong
  spawn heights and mismatched ground.
- **`<scale>` include warnings** at world load are expected (SDF `<include>`
  has no `<scale>`; gz copies it through) — they come from WildSeed's placement
  scaling and are harmless.
