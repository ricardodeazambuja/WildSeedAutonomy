# M4 — Lidar Odometry (LIO) on the sim

The second odometry frontend of the sensing spine, validated **in sim first**
(PLAN §12 M4). KISS-ICP estimates ego-motion from the Ouster OS1 point clouds
alone; `ego_localizer` fuses that as a relative-motion source through a hook
with the **same shape as the M3 visual one** — proving the sensor-agnostic
architecture (PLAN §15). Scored against the Gazebo ground-truth pose, A/B'd
against the M3 stereo VIO **in the same drive**. The real-dataset tier (NTU
VIRAL) stays a later milestone.

## Refresher: where LIO sits in the stack

- **Why lidar-only (KISS-ICP), not lidar-inertial (`rko_lio`)?** PLAN §3.2:
  the point is to show off **our** fusion. A LIO package fuses the IMU
  *internally* and hands back an already-fused pose — the interesting fusion
  happens in someone else's code, and re-using the IMU downstream would
  double-count it. KISS-ICP is lidar-only, so `fusion_core` does the real work
  and the spine IMU stays free. Its known failure modes are catalogued in
  [`kiss-icp-failure-modes.md`](kiss-icp-failure-modes.md) — and M4's job is
  to *provoke and measure* them against sim ground truth, not hide them.
- **Loosely coupled, exactly like M3.** KISS-ICP emits an absolute pose in its
  own drifting lidar-odometry frame. `ego_localizer` consumes only the
  **body-frame increment** between consecutive poses
  (`estimator.lidar_delta_update` — the identical measurement model as
  `visual_delta_update`), so the arbitrary frame offset cancels. One new
  adapter, **zero changes to `fusion_core`** — that is the architectural
  point of the whole milestone.

## Pipeline (what runs where)

```
husky sim ── /…/lidar3d_0/points (512×32 @20 Hz) ─► cloud_decimator ─ /…/points_decimated (5 Hz) ─► KISS-ICP ─ /kiss/odometry ─► ego_localizer ─► /ego_localizer/odom
           └ /…/imu_0/data ──────────────────────────────────────────────────────────────────────────────────────► (IMU yaw-rate) ──┘
   gz model pose ─ /world/<world>/dynamic_pose/info ─(ros_gz bridge)─► ground truth ─► eval_tools ATE/RPE
```

- **kissicp** runs in its own image (`sensing-node/kissicp:local`,
  `Dockerfile.kissicp` — source build pinned to `1ffa7d7`; not in the Jazzy
  apt distribution). Deskew **off**: the gz `gpu_lidar` cloud has no per-point
  time (PLAN §17.2; route via `gz_lidar_timestamp` to exercise the deskew
  path). `mapping.voxel_size 0.5`; everything else at defaults — see the
  tuning war stories below.
- **cloud_decimator** (`sensing_bringup/scripts/cloud_decimator.py`, `lio`
  profile) feeds KISS-ICP every 4th scan (20 → 5 Hz), count-based so it is
  RTF-proof. At the Husky's 0.5 m/s, 20 Hz means 2.5 cm per scan — far below
  KISS-ICP's designed automotive motion regime.
- **ego_localizer** consumes `/kiss/odometry` deltas (with a `lidar_min_dt:
  0.25` baseline) + the IMU yaw-rate. LIO alone — no wheel odom, no GNSS —
  same isolation logic as the M3 chart. Config:
  `config/ego_localizer_lidar.yaml`; `sigma_lidar_v: 0.35` is **fit from
  data** (p68 of the measured delta-velocity residual vs truth), not copied
  from the VIO config.

## How to run

```bash
./scripts/deploy.sh world default        # or a WildSeed bundle (vio_lio_recipe)
./scripts/m4_lio_eval.sh /results/m4     # fresh stack -> RTF gate -> FRESH kissicp
                                         # -> ego (lidar cfg) -> M3-chart drive -> chart
./scripts/deploy.sh m4-smoke             # the fail-fast gate (see below)
```

`m4_lio_eval.sh` encodes two hard-won bring-up rules: it waits for **steady
RTF > 0.4** (the load-transient trap), and then **restarts the kissicp
container fresh** — KISS-ICP builds its local map from scan #1 and never
resets, so a map polluted by slow-load-transient scans poisons the entire run
(measured: 12.8 m of spurious pose before the drive even began).

## Smoke gate — `./scripts/deploy.sh m4-smoke`

The lidar twin of `m3-smoke`, same SIM-seconds contract: **(1)** the Ouster
cloud arrives with enough finite returns, **(2)** the cloud has 3-D structure
(not a degenerate slab), **(3)** `/kiss/odometry` is live, **(4)** over a
short driven window KISS-ICP's translation agrees with gz truth within a loose
band (0.3–2.0×; the low floor is deliberate — see the under-report finding).
Script: `ros2_ws/src/sensing_bringup/scripts/m4_smoke.py`.

## Result (sim, laptop-only) — final config, same drive as the M3 chart

Over the M3-chart drive (0.5 m/s, wz 0.1, 45 sim-s, ~23 m) vs gz truth, all
three estimators recorded **in the same run**:

| trajectory | pipeline world (`results/m4_lio.png`) | WildSeed forest (`results/m4_recipe_lio.png`) |
|---|---|---|
| raw KISS-ICP (lidar-only) | ATE **0.985 m** / RPE 0.263 m | ATE **4.008 m** / RPE **0.100 m** |
| raw OpenVINS (stereo VIO) | ATE **0.045 m** / RPE 0.006 m | ATE **0.046 m** / RPE 0.006 m |
| fused ego_localizer (LIO + IMU yaw-rate) | ATE **1.029 m** / RPE 0.269 m | ATE **3.856 m** / RPE 0.101 m |

Two things to notice. **RPE is the stable frontend metric here**: the tree-rich
forest gives KISS-ICP 2.6× better local consistency (0.100 vs 0.263) — geometry
helps ICP exactly as the failure catalogue predicts. **ATE is dominated by the
systematic under-report bias** (war story #5) accumulating along the path — it
varied 2.3–4.0 m across repeated recipe runs while RPE stayed ~0.06–0.10, so
single-run ATE comparisons across worlds mostly sample how the bias happened
to integrate, not frontend quality.

**Reading the A/B honestly:** in this regime (slow planar UGV, feature-rich
sim imagery) the stereo VIO is the far stronger odometry — that is a real
result, not a failure. The lidar frontend's value is *redundancy across
conditions* (darkness, dust, texture-less scenes kill cameras, not lidar) and
it becomes load-bearing in the M5 keystone, where an absolute source bounds
its drift. The fused LIO output tracks its raw input (the fusion does not
degrade its best source once σ is fit from data) — with only relative sources,
no filter can *correct* a biased frontend; that is GNSS's job (M5).

## Terrain-complexity sweep — VIO vs LIO vs fused across worlds

`results/m4_terrain_sweep.png` — the same drive and spine on four worlds
(pipeline + three WildSeed bundles; `plot_m4_sweep.py` aggregates the
per-world metrics CSVs):

| world | raw VIO ATE/RPE | raw LIO ATE/RPE | fused-LIO ATE/RPE |
|---|---|---|---|
| pipeline (structured flat) | 0.045 / 0.006 | 0.985 / 0.263 | 1.029 / 0.269 |
| open terrain (`vio_lio_bare`) | 0.097 / 0.006 | 1.928 / 0.116 | 2.727 / 0.142 |
| forest (`vio_lio_recipe`) | 0.046 / 0.006 | 4.008 / 0.100 | 3.856 / 0.101 |
| alpine (`wildseed_42`) | **45.0 / 1.83 — DIVERGED** | 5.358 / 0.464 | 4.993 / 0.616 |

**The headline is complementary failure modes.** On alpine terrain the stereo
VIO *diverges* (position runs to 150+ m on IMU dead-reckoning while heading
stays sane — the m3-vio.md starvation signature): verified cause is **texture
starvation along the route** — at a mid-terrain pose both cameras render fine
(std ≈ 48) but yield only **57–58 Shi-Tomasi corners, under the 80-corner KLT
floor** (the spawn point had 98/95, so a spawn-point texture gate passes; the
*drive* crosses starved patches — texture is a runtime property, measured this
time). The lidar frontend keeps working exactly there. Conversely, lidar local
consistency is best in the forest (RPE 0.100) and worst on the alpine slopes
(0.464), while VIO is superb wherever texture exists. **No single frontend
survives all terrains — the case for the multi-frontend spine + absolute
anchoring (M5), made with numbers instead of assertion.**

Reading guidance: compare RPE across worlds (local odometry quality); ATE
integrates the slow-UGV under-report bias path-dependently (war story #5) and
mostly reflects how that bias happened to accumulate.

## The M4 war stories (what actually bit, in order)

Each was diagnosed from the recorded CSVs (path lengths, update cadence,
per-update step stats — `results/m4*_lio.csv`) before any knob was touched:

1. **Cloud size is the throughput lever.** At the original 1024×64 (65k pts)
   cloud, registration fell behind the 20 Hz scans under motion (6.6 Hz
   effective, jittery poses). Shrinking the voxel to 0.4 made it *worse*
   (1.8 Hz, 1.6 s gaps — a denser local map means slower ICP). After each gap
   the EKF's position–velocity cross-covariance is inflated, so one erratic
   velocity innovation yanked position by meters (22 m single-step jump).
2. **The "512×32 lidar" cut had never actually applied.** The Clearpath
   description *generator* hard-codes `samples_h=1024, samples_v=64` into the
   generated URDF, silently overriding the xacro defaults that
   `Dockerfile.sim` patched. Fixed by patching the generator too (guarded sed
   — the build fails if upstream changes shape). Lesson: when a sim sensor
   ignores a xacro tweak, grep the **generated** tree in the container.
3. **KISS-ICP must start on a settled world.** Its local map never resets; a
   container that comes up during the world's slow-load transient registers
   garbage scans and wanders for the whole run. (OpenVINS is inherently safe —
   static init waits for the jerk-start.) Hence the fresh restart in
   `m4_lio_eval.sh`.
4. **Do not "adapt" `min_motion_th` down.** Lowering it to 0.01 to "match the
   slow robot" froze KISS-ICP at the origin: while the robot sat still, sub-cm
   noise adapted the correspondence threshold toward zero, and once real
   motion started every true correspondence fell outside the search radius —
   ICP returned identity forever. The 0.1 default exists to prevent exactly
   that.
5. **The persistent finding — systematic translation under-report at UGV
   speeds.** Across every rate (20/6.6/5 Hz), voxel (1.0/0.5/0.4), threshold
   setting and world, KISS-ICP under-reports translation by ~40–60% on this
   robot: per-scan motion (2.5–10 cm) sits at or below its designed motion
   regime, and the robust kernel treats a good share of the real motion as
   error. Higher speed does not cure it (a 1.5 m/s run still read ~0.3×
   before the robot met a tree). Ruled out along the way: self-hits (the
   512×32 cloud has zero returns under 1.2 m), deskew (off, and speeds are
   low), QoS drops (cadence is clean at 5 Hz). This is failure-catalogue
   mode #1/#2 territory measured on our own robot — the M4 degeneracy story,
   and the concrete motivation for absolute anchoring (M5) and the
   `kinematic-icp`/`rko_lio` rungs of the PLAN §3.2 ladder.

## What M4 sim-first proves (and what it doesn't)

Proves: the **sensor-agnostic spine** — a second, physically different
frontend lands as one more relative hook (`lidar_delta_update`), no
`fusion_core` change, same eval harness, honest A/B against the first
frontend. Also: a reproducible failure catalogue entry with numbers.

Doesn't prove: lidar odometry accuracy on real data (needs native per-point
time + real geometry — NTU VIRAL, the M4 real-lidar tier), or drift-bounding
(that is M5's GNSS keystone, where the lidar frontend earns its keep).
