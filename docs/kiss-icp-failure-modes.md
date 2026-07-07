# Where KISS-ICP breaks — and how we'll test it in sim

Companion to PLAN §3.2 (lidar frontend decision). We chose **KISS-ICP (lidar-only)**
as the primary lidar frontend so the IMU stays free for `fusion_core` (no
double-counting). KISS-ICP is excellent but it is **pure lidar odometry with a
constant-velocity model and no loop closure** — so it has known, predictable
failure modes. This catalogs them, how to **provoke each in simulation**, what a
healthy fusion should look like instead, and the **mitigation/improvement** path.

The point: don't discover these on real data — **manufacture them in Gazebo**,
where we have **perfect ground-truth pose** to measure against.

> **M4 status (2026-07-07): measured.** Mode **#1** materialized on its own in
> the four-world terrain sweep (`results/m4_terrain_sweep.png`), and the M4
> bring-up surfaced **two modes this catalogue didn't predict**: (a) the
> **slow-UGV translation under-report bias** (~40–60 % across every
> rate/voxel/threshold/world — per-scan motion sits below KISS-ICP's
> automotive design regime; read RPE, not ATE, across worlds) and (b)
> **adaptive-threshold collapse when stationary** if `min_motion_th` is
> lowered (the pose freezes at the origin once motion starts). Numbers, the
> ruled-out hypotheses, and the final config: [`m4-lio.md`](m4-lio.md).

## Test rig (applies to every mode below)
- **Ground truth:** Gazebo publishes the robot's true pose. Log KISS-ICP `/odom`
  vs GT and compute **ATE / RPE with `evo`** (PLAN §3.2 tooling).
- **Isolate odometry from fusion:** first measure raw KISS-ICP error, then turn on
  `fusion_core` (KISS-ICP + IMU + GPS) and re-measure → the delta *is* the story.
- **GPS instrument:** use the `gps_conditioner` (PLAN §11) to add/remove GPS so we
  can show the failure with lidar-only and the recovery with fusion.
- **Reproducible:** each stress case is a world + a trajectory + a scenario file,
  committed under `eval_tools/` so a chart regenerates on demand.

## Failure modes (ranked by relevance to an off-road UGV)

### 1. Geometric degeneracy — the headline (open / featureless scenes)
- **Mechanism:** point-to-point ICP needs geometric structure to constrain all
  DoF. In open flat fields, long straight tracks, tunnels, or smooth-walled
  corridors there's no constraint along one or more axes → the pose **slides**
  (unbounded translational drift along the degenerate direction). This is the
  off-road case (open terrain) and is independent of the IMU question.
- **Provoke in sim:** a large flat ground plane with sparse/no features; drive a
  long straight line, then a featureless gentle curve. Compare against a
  feature-rich world (the `pipeline` hills/structures) as the baseline.
- **What "holds":** IMU prediction in `fusion_core` constrains the slide between
  scans; GPS bounds it absolutely. The money chart: lidar-only drifts in the open,
  IMU+GPS fusion holds (PLAN M4 bonus + §2 chart #2).
- **Improve / reduce:** detect degeneracy online — monitor the registration's
  conditioning (information-matrix eigenvalues / correspondence count & spread) and
  **inflate the lidar covariance per-axis** in the degenerate direction so the
  filter leans on IMU/GPS there. Stretch: `kinematic-icp` (wheeled motion prior,
  already in §3.2 ladder) adds an along-track constraint a ground vehicle should
  obey.

### 2. Aggressive / non-constant motion (constant-velocity model breaks)
- **Mechanism:** KISS-ICP assumes velocity is ~constant across a sweep (true at
  10–20 Hz for smooth motion). Hard accel/decel, sharp skid-steer yaw, or sudden
  jolts on rough terrain *within* a sweep violate it → bad deskew + bad initial
  guess → degraded or failed registration.
- **Provoke in sim:** a rough heightmap (rocks/ruts) at increasing speed; rapid
  skid-steer spin-in-place; aggressive stop-go. Sweep speed up and plot error.
- **What "holds":** the IMU sees the jolt at high rate; in `fusion_core` it carries
  the prediction so a momentary lidar miss doesn't propagate.
- **Improve / reduce:** raise the lidar rate; `kinematic-icp` prior; cap mission
  speed for the demo; if motion is genuinely too aggressive to own, this is the
  trigger to switch that run to the **`rko_lio` (LIO) fallback** (IMU-in-frontend).

### 3. Deskew with no per-point time (a *sim* artifact — PLAN §17.2)
- **Mechanism:** deskewing (constant-velocity *or* IMU) needs per-point timestamps.
  Gazebo `gpu_lidar` emits `xyz/intensity/ring` but **no per-point time** → can't
  deskew → skew error grows with speed.
- **Provoke in sim:** same straight run at increasing speed with **deskew-off**;
  plot error vs speed.
- **What "holds" / fix:** synthesize per-point time from azimuth/column + update
  rate with the **`gz_lidar_timestamp` node** (`ros2_ws/src/gz_lidar_timestamp`),
  then re-run and compare; or keep deskew-off and stay at moderate speed (small
  error). Not present on real datasets (Ouster native time+ring, Livox per-point
  time) — so this is about quantifying the *sim* penalty, not a real limitation.

### 4. Unbounded global drift over long runs (no loop closure)
- **Mechanism:** KISS-ICP is odometry only — small per-scan errors accumulate; with
  no loop closure, global error grows without bound on long trajectories. **By
  design**, not a bug.
- **Provoke in sim:** a long loop; measure end-to-end position error with GPS off.
- **What "holds":** GPS in `fusion_core` bounds the drift absolutely — this is the
  whole reason GPS is fused. Show drift-with-GPS-off vs bounded-with-GPS-on.
- **Improve / reduce:** GPS fusion (primary); place recognition / loop closure is
  explicitly out of scope for the odometry frontend (PLAN keeps loop closure out to
  avoid trajectory jumps in fusion).

### 5. Dynamic objects in the scene
- **Mechanism:** ICP assumes a static world. Many moving objects (vehicles, people)
  add wrong correspondences → bias/drift.
- **Provoke in sim:** spawn moving actors crossing the path; vary density.
- **What "holds":** off-road scenes are usually sparse; KISS-ICP's robust kernel
  tolerates a few movers.
- **Improve / reduce:** dynamic-point filtering before registration; tighten the
  robust threshold. Low priority unless the scenario is crowded.

### 6. Sparse / low-density returns & range limits
- **Mechanism:** few correspondences (low-line lidar, long range, low-reflectivity
  surfaces like snow/water) → weak, noisy registration.
- **Provoke in sim:** reduce lidar rings/range/point density in the Ouster config;
  measure error.
- **What "holds":** denser lidar, scan accumulation, fusion.
- **Improve / reduce:** voxel-size tuning for the environment scale; accumulate
  multiple sweeps before registering.

## How this drives the roadmap
- Modes **1–4** are the ones worth instrumenting now; **1** is the headline that
  produces the M4 degeneracy money chart and motivates the whole IMU+GPS fusion.
- Each mode's "improve / reduce" line is a **future-work hook**, not first-pass
  work — we measure first (is it actually a problem on the Husky in `pipeline`?),
  then fix only what the charts show is real (the "don't over-react" principle).
- Modes **5–6** are noted for completeness; provoke them only if a target scenario
  needs them.
