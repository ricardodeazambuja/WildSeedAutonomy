# Documentation index

One line per document. Each doc **owns its topic** — everything else cross-links
to it instead of copying, so updates happen in exactly one place.

## The two entry points

- [`PLAN.md`](PLAN.md) — the master **design & rationale** (timeless): architecture,
  verified stack, milestones (§12, §18), Docker design (§7), datasets (§9),
  pre-flight walls + smoke tests (§17), risk register (§14).
- [`status-and-testing.md`](status-and-testing.md) — the single **per-session-updated**
  home: milestone status table, the tiered testing manual, laptop environment
  verification, the per-milestone verification log (evidence), and next steps.

## Operating the stack

- [`operations.md`](operations.md) — commands, roles → Compose profiles, M1 smoke
  tests, resource caps, verified configurations, known issues.
- [`headless-gui.md`](headless-gui.md) — seeing Gazebo & RViz when the sim is
  headless: the local-GPU GL (`/dev/dri` + NVIDIA caps) gotcha, the namespaced-TF
  RViz gotcha, CycloneDDS unicast fallback.
- [`nav-n1-teleop.md`](nav-n1-teleop.md) — teleoperation + e-stop (N1), incl. the
  stale-timestamp "robot randomly won't drive" gotcha.
- [`wildseed-worlds.md`](wildseed-worlds.md) — procedural WildSeed terrain: bundle
  workflow (`prepare_wildseed_world.sh` → `deploy.sh world`), the RTF wall.

## Fusion / estimation

- [`gps-fusion-conventions.md`](gps-fusion-conventions.md) — the sourced deep-dive
  behind PLAN §17.4: `navsat_transform` inputs, REP-103/105 conventions, IMU
  message rules, top-5 "GPS fusion silently wrong" causes + fixes.
- [`m3-vio.md`](m3-vio.md) — M3 stereo OpenVINS VIO on the sim: pipeline, how to
  run, the `m3-smoke` gate, results (ATE 0.069 m raw / 0.077 m fused).
- [`m4-lio.md`](m4-lio.md) — M4 KISS-ICP lidar odometry on the sim: pipeline, the
  `m4-smoke` gate, the VIO-vs-LIO A/B, the four-world terrain sweep
  (complementary failure modes), and the bring-up war stories.
- [`kiss-icp-failure-modes.md`](kiss-icp-failure-modes.md) — where KISS-ICP breaks
  and how M4 provokes & measures each failure in sim; measured outcomes + two
  newly discovered modes are in [`m4-lio.md`](m4-lio.md).
- [`costmap-deep-dive.md`](costmap-deep-dive.md) — how the Nav2 2D costmap is
  built/updated from the Ouster + OAK-D; the off-road/traversability problem.

## Debugging lore

- [`sim-debugging-notes.md`](sim-debugging-notes.md) — Husky headless bring-up
  postmortem: #7 (`/clock` bridge), #8 (solid-colour camera = llvmpipe EGL fallback
  + camera below the deck), and the meta-learnings.

## Real hardware (separate from the sim path)

- [`oak-d-lite-guide.md`](oak-d-lite-guide.md) — OAK-D Lite field guide: BMI270
  IMU characterisation, camera↔IMU sync, depthai version guide, the 3.7.1
  firmware regression investigation. (Probe scripts live at the repo root.)
