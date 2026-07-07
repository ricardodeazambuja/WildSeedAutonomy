# Status & Testing Manual

The **single per-session-updated home** for project status: where we stand against
the plan (§1), a brief manual for exercising the system on this laptop (§2), the
laptop environment verification (§3), the per-milestone **verification log** with
evidence (§4), and the next-steps roadmap (§5). Design and rationale live in
[`PLAN.md`](PLAN.md) — references like "PLAN §17.4" point there.

## 1. Where we are vs. the plan

The project is a sensor-agnostic edge-fusion + GPS-denied localization stack on
ROS 2 Jazzy + Gazebo Harmonic, fully Dockerized. Against the 12-milestone
roadmap (PLAN §12) and the navigation milestones (PLAN §18):

| Milestone | Status |
|---|---|
| **M1** — Docker/compose/DDS/GPU infra, headless render, Husky sim w/ lidar+cam+IMU+GPS | ✅ done, laptop-verified (`ROLE=all`) — §3 |
| **N1** — teleop + e-stop nav mode | ✅ done — [`nav-n1-teleop.md`](nav-n1-teleop.md) |
| **M2** — `fusion_core` ROS-free EKF library | ✅ done (14 pytest green) |
| **M3** — visual frontend: stereo OpenVINS VIO → `ego_localizer` | ✅ **done sim-first** (raw ATE 0.069 m / fused 0.077 m vs gz truth) — [`m3-vio.md`](m3-vio.md); **M3b** (EuRoC real-imagery comparison) deferred, needs download |
| **eval_tools** — ATE/RPE metrics + chart CLI | ✅ done (the "money-chart" backbone) |
| **GPS-denied keystone (in sim)** | ✅ **demonstrated end-to-end** — `results/gps_denied_keystone.png`: mean \|ego − GPS\| **on = 0.12 m → denied = 0.20 m → reacquire = 0.14 m**; ✅ **re-verified on WildSeed procedural terrain** — `results/gps_denied_wildseed.png` (wildseed_42, RTF 0.6): err **0.07 → 0.61 m over a 16 m denied stretch → snaps back to 0.07 m** |
| **WildSeed procedural worlds** | ✅ integrated + verified (m3-smoke PASS on `scenario --seed 42`) — [`wildseed-worlds.md`](wildseed-worlds.md) |
| **M4** — lidar frontend: KISS-ICP → `ego_localizer` lidar hook | ✅ **done sim-first** (raw LIO ATE 0.99 m/RPE 0.26 m pipeline · RPE 0.10 m WildSeed forest; fused tracks raw; VIO A/B same-run) — [`m4-lio.md`](m4-lio.md); **M4 real-lidar tier** (NTU VIRAL) deferred, needs download |
| **M6** — GTSAM factor-graph variant of the fusion core + A/B vs EKF | ✅ done (`fusion_core.factor_graph.PlanarFactorGraph`, same estimator interface; accuracy parity, EKF 3.5× cheaper on the GNSS-heavy scenario — `results/m6_ab.md`) |
| M5 (real) / M7–M12 | not started — gated on external data / hardware — §5 |

Evidence for every ✅ is in the verification log (§4). The big PLAN §17.4 wall
(6-axis IMU heading not anchored to the GPS ENU frame, causing the estimate to
spiral) was hit as predicted and **fixed** via course-aided heading (fuse IMU
yaw-rate only; anchor absolute heading from GPS course-over-ground).

## 2. Brief manual — testing on this laptop

Everything runs through `scripts/deploy.sh` (Docker is the only prerequisite;
`ROLE=all` = whole stack on one box). Grouped into tiers: **setup**, **fast smoke
tests**, **the live sim**, **the keystone demo**, and **VIO / procedural worlds**.

> **Slow machines / sim-seconds contract.** Every demo and smoke gate below
> defines its durations in **SIM seconds** and self-reports the measured RTF
> (`[simtime] RTF≈…`), so they run correctly — just slower in wall time — on
> weak machines or dense worlds. CSV `t` columns are sim-seconds too (the
> committed reference artifacts were recorded at RTF≈1, where the two clocks
> coincide). `./scripts/deploy.sh rtf` reads the sim speed any time;
> [`operations.md`](operations.md) "Slow machines / low RTF" has the knobs
> (`SLOW_SIM_FACTOR`, `SIM_RTF_FLOOR`) and the measured tuning ladder.

### Tier 0 — First-run setup (one-time)

```bash
./scripts/deploy.sh check      # PASS/WARN/FAIL on Docker, NVIDIA GPU, disk, X
./scripts/deploy.sh init       # writes .env tuned to this host (already ROLE=all)
./scripts/deploy.sh build      # builds sim + fusion images
```

**Expect:** `check` green on Docker + NVIDIA passthrough. `build` is heavy
(~15 GB pull) but is cached after the first run.
**Caveat:** if `.env` already exists, `init` is a no-op — fine.

### Tier 1 — Fast unit/smoke tests (seconds–minute, no GUI)

```bash
./scripts/deploy.sh smoke      # cross-container DDS talker→listener
./scripts/deploy.sh render     # headless GPU render (EGL, smoke world)
```

**Expect:** `smoke` greps the listener for "I heard" and returns non-zero on
failure. `render` proves ogre2/EGL works without a display.

The pure-logic core (the EKF, the node math, the metrics) — run the 29-test
suite in the fusion image:

```bash
./scripts/deploy.sh shell fusion
# inside the container:
cd /ros2_ws && colcon test --packages-select fusion_core ego_localizer eval_tools && colcon test-result --verbose
```

**Expect:** **29 tests, 0 failures** (fusion_core 14 + ego_localizer 9 +
eval_tools 6). This is the fastest proof the fusion math is correct and is fully
laptop-deterministic.

### Tier 2 — The live sim (visual)

```bash
./scripts/deploy.sh viz        # sim + RViz, all local (add --gz for the Gazebo GUI)
./scripts/deploy.sh teleop     # drive the Husky with the keyboard (separate shell)
./scripts/deploy.sh down       # stop everything
```

**Expect:** a Clearpath Husky (Ouster lidar, OAK-D stereo, Microstrain IMU,
swiftnav GPS) in the off-road `pipeline` world; RViz shows the lidar cloud +
odom trail; teleop drives it.
**Caveats:**

- Needs an X display (you're on `DISPLAY=:1`). The GUI uses hardware GL on the dGPU.
- ⚠️ Known gotcha: if the robot won't move / nodes log "No clock received", check
  `ros2 node list` for `clock_bridge` and do a clean `deploy.sh restart` — don't
  chase server count (documented in [`operations.md`](operations.md)).

To confirm sensors are live (with the sim up):

```bash
./scripts/deploy.sh shell fusion
ros2 topic list | grep -E 'lidar3d|oakd|imu|gps'   # expect namespaced a200_0000 topics
```

### Tier 3 — The keystone: GPS-denied drift→reacquire demo

This is the marquee result. It drives the robot through **GPS on → denied →
reacquire** and records ego-estimate vs GPS truth. With the husky sim up, run
`ego_localizer` in relative+GNSS mode **directly from source** (the
`ego_localizer_gnss.launch.py` route needs a colcon-built/installed workspace,
which the mounted `ros2_ws` doesn't ship):

```bash
# inside the fusion container (deploy.sh shell fusion):
PYTHONPATH=/ros2_ws/src/fusion_core:/ros2_ws/src/ego_localizer:$PYTHONPATH \
  python3 /ros2_ws/src/ego_localizer/ego_localizer/node.py --ros-args \
  --params-file /ros2_ws/src/ego_localizer/config/ego_localizer_gnss.yaml &
# then (scripts/ isn't mounted in fusion — docker cp it in, or run from a bind mount):
python3 gps_denied_demo.py /results/gps_denied_keystone.csv
python3 plot_gps_denied.py /results/gps_denied_keystone.csv /results/gps_denied_keystone.png
```

**Expect:** a printed summary `mean|ego-gps| on≈0.2 denied≈1.6 reacq≈0.45`, and
a chart whose error envelope **rises through the shaded GPS-denied window
(≈0.2 → 3.3 m) and snaps back at reacquire**, with a top-down track that bulges
off GPS during the outage and re-locks.
**Caveats (honest, documented):**

- These numbers are from the 2026-07-04 sim-time rework re-run
  (`results/gps_denied_verify.png`): the demo now publishes at a sustained
  50 Hz, so the robot actually holds the commanded 0.4 m/s — more distance in
  the outage, more dead-reckoning drift, a clearer keystone. The original
  committed artifact `results/gps_denied_keystone.png` (on=0.12 / denied=0.20 /
  reacq=0.14) predates this; its gentler drift came from the old ~18 Hz
  publisher stuttering through twist_mux timeouts.
- GPS is ~1 Hz so the error sawtooths slightly. The demo is parameterised —
  `gps_denied_demo.py <csv> [v wz on_s denied_s reacq_s]` (all SIM seconds).
- Render charts **inside the fusion image**, not host conda (host has a numpy
  2.x/matplotlib mismatch).

### Tier 4 — VIO/LIO smoke + procedural worlds

```bash
./scripts/deploy.sh m3-smoke                 # stereo OpenVINS live-VIO gate
./scripts/deploy.sh m4-smoke                 # KISS-ICP lidar-LIO gate (clouds rich, LIO live+sane)
./scripts/m4_lio_eval.sh [/results/prefix]   # one-shot M4 eval: RTF gate -> fresh kissicp -> drive -> chart
./scripts/deploy.sh world <bundle>           # swap in a WildSeed world, then re-run the smokes
./scripts/deploy.sh rtf                      # measured sim speed + tier hint
./scripts/bench_rtf.sh [--world-only] [bundle]   # where does the RTF go? (4 variants)
```

**Expect:** `m3-smoke` reports stereo feature corners, 0 ms stereo sync, and a
live OpenVINS `/odomimu`; `m4-smoke` reports a rich Ouster cloud, live
`/kiss/odometry`, and a translation-vs-truth ratio around 0.4–0.6 (KISS-ICP
under-reports at UGV speeds — measured finding, see [`m4-lio.md`](m4-lio.md)).
Full VIO pipeline: [`m3-vio.md`](m3-vio.md); lidar pipeline + the M4 war
stories: [`m4-lio.md`](m4-lio.md); world bundling workflow + the RTF wall:
[`wildseed-worlds.md`](wildseed-worlds.md).

## 3. Laptop-only verification (environment)

The stack was first brought up in the two-box split (workflow B) — a desktop **server**
(`compute`) running headless sim + fusion, the laptop only acting as the `gui` client. This
section verifies the **laptop-only** path (`ROLE=all`, workflow A), where a single laptop
(hybrid **Intel UHD 630 + NVIDIA RTX 2070 Max‑Q**, driver 535, `DISPLAY=:1`) runs the *whole*
stack alone — a path that hadn't been exercised end-to-end before. It now has — **all green**:

| Step | Result on the laptop |
|---|---|
| `deploy.sh check` | 8/8 pass — Docker, nvidia‑ctk, **GPU passthrough into a container**, 436 GB free, `DISPLAY=:1` |
| `deploy.sh init` | `.env` → `ROLE=all`, `DISPLAY=:1` (captured from the live env), caps 10 CPU / 23 GB |
| `deploy.sh build` | both images (`sim` 6.5 GB, `fusion` 3.85 GB) |
| `deploy.sh smoke` | cross‑container DDS (talker→listener) on localhost |
| `deploy.sh render` | **EGL headless render on the dGPU** — `/smoke/camera` emits 640×480 RGB |
| `diag_sim.sh` | controllers up ~6 s, sensors live, `/clock` 1 pub ~461 Hz, odom ~10 Hz, **movement** (odom x 0 → 1.57 m) |
| `deploy.sh viz` | **RViz `OpenGl version: 4.6` — hardware GL** (no MESA fallback) + Gazebo GUI (now opt-in via `viz --gz`), on the local GPU |

### What changed (laptop-only bring-up session)
- **`scripts/remote.sh` guards against a missing server.** When the configured server is
  unreachable (e.g. a desktop that isn't on the LAN), every `remote.sh` subcommand used to hang on SSH.
  Added a `require_server` preflight (TCP‑probe `:22`) that, when unreachable, prints a
  clear message pointing back to `deploy.sh` for the laptop‑only path. `help` still works
  with no server. `remote.sh` is now explicitly **workflow B only**.
- **`README.md`** reordered so **laptop‑only (workflow A) is the default**; server+laptop
  (workflow B) is marked "only if you have a separate GPU box".
- **`scripts/deploy.sh`** — fixed a mislabel (`cmd_viz` said "option B"; local viz is
  workflow A everywhere else).

### Learnings / caveats
- **Hybrid Intel+NVIDIA GL works as‑is.** The `/dev/dri` mount + `NVIDIA_DRIVER_CAPABILITIES=all`
  recipe (already baked into the `rviz`/`gzgui` services for the cross‑host case) is exactly
  what a PRIME laptop needs: X on the iGPU (`:1`), GL on the dGPU. RViz reports OpenGL 4.6 —
  no MESA software fallback. The headless `husky` sim needs none of this (EGL offscreen,
  compute caps only) and renders fine on the dGPU.
- **`DISPLAY` is `:1` here, not `:0`.** `.env.example` defaults to `:0`, but `deploy.sh init`
  captures the live `DISPLAY` into `.env`, so this is handled automatically *as long as init
  runs in the graphical session* (it did). If you ever `init` over a bare SSH session, fix
  `DISPLAY` in `.env` by hand.
- **`diag_sim.sh`'s movement number is partial by design.** It drives with `ros2 topic pub`
  (stamp 0), which the `use_sim_time` diff_drive controller drops once sim time passes the
  cmd_vel timeout — so it only moves for the first moments after bring‑up (here x 0 → 1.57 m,
  not the full 0.8 m/s × 6 s). That's the documented timestamp gotcha (PLAN §18/N1, sim‑debugging #7),
  *not* a regression — the test passes because the robot moved and `/clock` is healthy. The
  N1 demo (`demo_n1_teleop.sh`) drives via `n1_drive.py` (sim‑time stamps) for the full path.
- **`remote.sh` / workflow B needs a second box to exercise fully.** With only a single
  machine, the two-box path isn't re-run here; the only change on the laptop-only path is the
  graceful `require_server` guard, which *is* verified (fires cleanly, exits 1, points to `deploy.sh`).

## 4. Milestone verification log

Each milestone is "done" only when its smoke test passes with **evidence** on this
laptop (`ROLE=all`), not when it merely launches.

**M2 — `fusion_core` + pytest.** Built `ros2_ws/src/fusion_core/`
(ROS‑free numpy EKF: `predict`/`update`, Joseph‑form covariance, NIS + Mahalanobis;
constant‑velocity + white‑noise‑acceleration models). Verified two ways in the
`fusion` image: (1) `python3 -m pytest test/` → **14 passed**; (2) `colcon build`
+ `colcon test --packages-select fusion_core` → **14 tests, 0 failures** (so it
integrates as an ament_python package M3's `ego_localizer` can depend on). Tests
go beyond "runs": covariance grows on predict / shrinks on update, Joseph form keeps
`P` symmetric‑PSD over 500 steps, a static state converges to truth, the **fused
estimate beats the raw measurements** (RMSE < 0.7× on a noisy CV trajectory), and the
filter is **consistent** (mean NIS ≈ measurement dim). *Server note: ROS‑free + pure
numpy → identical on the server; nothing here is laptop‑specific.*

**M3 foundation — `ego_localizer` node.** Built
`ros2_ws/src/ego_localizer/`: a `PlanarPoseEstimator` (ROS‑free, wraps the M2 EKF;
state `[px,py,yaw,vx,vy,wz]`, CV predict, IMU + wheel‑odom updates with wrapped yaw
innovations) + a thin ROS node (`node.py`) that predicts‑to‑now and republishes a
fused `nav_msgs/Odometry`. Verified three ways: (1) **6 offline pytest** — fused
heading ~**12×** better than raw odom heading, fused position beats raw odom and
tracks truth to ~10 cm, covariance stays sym‑PSD; (2) **colcon build + test** of
both packages together → **20 tests, 0 failures** (ego_localizer correctly
`exec_depend`s fusion_core); (3) **live on the Husky sim** — ran the node against
`/a200_0000/sensors/imu_0/data` + `/platform/odom`, drove ~4 m forward, and
`/ego_localizer/odom` tracked `/platform/odom` to **~1 mm** (−1.96→2.02 vs
−2.00→2.02). *Run note: in the fusion image, append ROS to PYTHONPATH
(`PYTHONPATH=/fc:/el:$PYTHONPATH`) — overriding it drops rclpy.* **Remaining for full
M3:** OpenVINS visual frontend + EuRoC + ATE/RPE vs robot_localization & Vicon. That
eval layer is dataset/source‑build heavy (OpenVINS is the one source build on the
critical path, PLAN §14) — a separate sub‑task. *Server note: ego_localizer is a normal
DDS node; identical on the server.*

**eval_tools — ATE/RPE + chart backbone.** Built
`ros2_ws/src/eval_tools/` (the PLAN §6 eval layer), needed before any milestone can
produce its "money chart": `metrics.py` (ATE with Umeyama SE(3)/Sim(3) alignment,
RPE local‑drift) + `evaluate.py` CLI (TUM/CSV in → aligned top‑down trajectory plot
+ ATE/RPE bars + metrics.csv). Verified: **6 pytest** (alignment recovers a known
transform; ATE≈0 for a pure rigid offset; ATE≈noise level; Sim(3) absorbs a scale
error rigid can't; RPE ignores a global offset but catches drift) and an
**end‑to‑end CLI run** on synthetic trajectories → ego ATE 0.051 m vs a drifting
odom 0.455 m, chart rendered. Full workspace now **colcon test = 26 tests, 0
failures** (fusion_core 14 + ego_localizer 6 + eval_tools 6). Pure numpy/matplotlib
→ identical on the server. This unblocks the real‑data charts for M3/M4/M5; what
those still need is the data + frontends (EuRoC+OpenVINS, aerial lidar), not tooling.

**Sim GNSS — keystone slice 1.** Added a `swiftnav_duro` GPS to
`config/robot.yaml` (`sensors: gps:`). Pleasant surprise: the `pipeline` world
**already** ships `<spherical_coordinates>` (datum 57.0271, −115.4268) + the
`gz::sim::systems::NavSat` plugin, so the PLAN §17.2 world‑side wall did **not** apply.
Verified live: the sim now publishes a ROS `sensor_msgs/NavSatFix` on
`/a200_0000/sensors/gps_0/fix` (Clearpath auto‑bridges the gz `navsat` sensor) at
~1 Hz sim‑time with a **valid fix** — lat 57.02712, lon −115.42677, alt 600.4 m,
status FIX. This is the absolute, *droppable* input the GPS‑denied keystone needs,
**fully in sim (no download)**. *Server note: identical — it's sim + a DDS topic.*

**Sim GNSS — keystone slice 2: machinery built + offline‑verified;
live run hit the PLAN §17.4 heading wall (as predicted).** Added to `ego_localizer`:
`odom_twist_update` (consume wheel odom as **relative** velocity → dead‑reckons),
`gnss_update` (absolute ENU fix), an `odom_mode: absolute|relative` switch, a NavSatFix
subscription with first‑fix ENU conversion, and a runtime **dropout toggle**
(`std_msgs/Bool` on `~/set_gps_enabled`) + `config/ego_localizer_gnss.yaml` /
launch + `scripts/gps_denied_demo.py`. **Offline‑verified:** a deterministic
keystone pytest (GPS on → error bounded; denied → drifts >4×; reacquire → snaps
back) passes; full workspace **colcon test = 27 green**. **Live on the sim:** drove
a curved path with a GPS on→denied→reacquire timeline; the recorded |ego − GPS|
error **grew monotonically (0.2 → 0.45 → 0.65 m) and did NOT recover on reacquire**.
Root cause (confirmed, = the PLAN §17.4 wall): the **gz IMU orientation is in the gz
world frame, not anchored to the GPS ENU frame**, so the relative‑odom velocity is
rotated by a heading that's offset from ENU → the estimate **spirals**, and a
**position‑only** GPS update can't correct a heading‑frame error. The robot.yaml
IMU is effectively 6‑axis here (no magnetometer wired) — exactly the PLAN §17.4 failure.
**Honest status: live keystone NOT passing yet** — blocked on heading anchoring (fixed in
slice 2.1 below). The algorithm is right (offline proof); the sim needs a heading source.

**Sim GNSS — keystone slice 2.1: PLAN §17.4 heading spiral FIXED via
course‑aided heading.** Implemented the no‑new‑sensor fix: in
relative mode the node fuses the IMU's **yaw‑rate only** (`imu_rate_update` —
frame‑independent) and anchors absolute heading from the **GPS course‑over‑ground**
(`heading_update`, computed from consecutive ENU fixes when moving > 0.25 m). Seed
yaw from the first course. **Offline‑verified:** new `test_..._course_aided...`
keystone test (IMU absolute yaw never fused) → bounded → drifts >2× → recovers to
~on‑level; **ego_localizer 9 tests, full workspace 29 tests, 0 failures.** **Live on
the sim:** the spiral is **gone** — ego now tracks GPS (vs the slice‑2 monotonic
0.2→0.45→0.65 with no recovery): a clean run gave on ≈ 0.21 m, denied drift to
~0.5–0.7 m, and reacquire pulling back (0.69→0.33 m over the window). ⚠️ The live
*chart* isn't a clean showcase yet: GPS is sparse (~1 Hz, and the sim runs <1× real
time) so the error sawtooths and post‑reacquire recovery is gradual — a demo‑tuning
issue (slower drive / higher GPS rate), not an algorithm one. Tools committed:
`scripts/gps_denied_demo.py` (drive/drop/reacquire recorder),
`scripts/plot_gps_denied.py` (chart). *Server note: pure DDS + numpy; identical on
the server.*

**Sim GNSS — keystone slice 3: drift→reacquire chart produced
(`results/gps_denied_keystone.png`).** A slow‑drive (0.4 m/s), long‑denial (40 s)
run gives a clean, honest keystone: mean |ego − GPS| **on = 0.12 m → denied = 0.20 m
→ reacquire = 0.14 m**. The error‑vs‑time envelope **rises through the shaded
GPS‑denied window and falls after reacquire**; the top‑down plot shows the fused
track bulging off GPS during the outage and snapping back. The keystone (PLAN
chart #1 / PLAN §11) is now **demonstrated end‑to‑end in sim, laptop‑only, no download**.
Honest caveats: (1) the drift is *modest* (~0.2 m over 40 s) because the sim wheel
odometry is accurate at low speed — bigger drift needs slip/bias or a longer outage;
(2) the ~1 Hz GPS leaves a visible sawtooth — bumping `swiftnav_duro` `update_rate`
1→10 Hz (realistic for that receiver; a `Dockerfile.sim` sed + rebuild) would sharpen
the chart. Both are polish, not correctness. Demo is parameterised
(`gps_denied_demo.py <csv> [v wz on denied reacq]`).

**M3 — visual frontend live on the sim: stereo OpenVINS VIO → ego_localizer →
ATE/RPE vs gz ground truth (`results/m3_vio.png`).** The M3 milestone, validated
**sim‑first** (no download). Built OpenVINS (rpng/open_vins, **master** pinned to
`69488123`) as a dedicated image — cleared two PLAN §17.1 walls: Ceres 2.2's removed
`LocalParameterization` (master has the fix) **and** the Jazzy `.h`→`.hpp` header
rename (cv_bridge / image_transport / point_cloud2_iterator / tf2_geometry_msgs ship
only as `.hpp`; thin forwarding shims let OpenVINS build unmodified). Runs **stereo**
on the sim OAK‑D Lite L/R pair (two RGB 320×240, ideal pinhole, intrinsics from the
gz HFOV 1.25 → fx=fy=221.8; cam‑IMU extrinsics derived analytically from the URDF
mount, kalibr `cam0`/`cam1` y‑sep = the 0.075 m mount baseline). `ego_localizer`
consumes `/odomimu` as a **relative body‑frame increment** (`visual_delta_update`, the
VIO frame cancels — loosely coupled, no IMU double‑count) fused with the IMU yaw‑rate;
**VIO alone**, no wheel odom, **no GNSS** (`gps_enabled: false`; GNSS is M5), so the
chart reflects the visual frontend. Ground truth = gz model pose bridged from
`dynamic_pose/info`. **Result:** over a 20.5 m curved drive, **raw stereo OpenVINS ATE
0.069 m / RPE 0.004 m, fused ego_localizer ATE 0.077 m / RPE 0.009 m** — both hug truth,
fusion ≈ its input (not worse), rigid‑SE(3) Umeyama so the 0.069 m is genuine **metric**
scale. **Why stereo (not mono):** monocular VIO is **scale‑degenerate on a smooth planar
UGV** — scale comes only from IMU accel excitation, which a constant‑velocity Husky on
flat ground barely provides; mono OpenVINS wouldn't even **initialise** (static init
needs an accel jerk the ramp never makes), and when forced it diverged to km. The
stereo baseline makes scale directly observable — a **geometric, not parametric** fix.
**Two motion lessons baked into `m3_vio_demo.py`:** (1) a **jerk‑start** (two sharp
forward jabs) to fire static init; (2) publish cmd_vel at **50 Hz** or twist_mux times
it out and the robot stutters (3.5 m instead of 20 m) → no translation → VIO starves.
(The earlier "featureless terrain/sky, position runs to >7000 m" episode was the
camera‑render bug #8 + the mono/no‑motion combo, all now resolved.) Tooling:
`Dockerfile.openvins`, `config/openvins/`, `scripts/m3_vio_demo.py`, compose `vio`
profile (`deploy.sh up vio`), `config/ego_localizer_visual.yaml`. **Remaining for M3b
(deferred, needs download):** OpenVINS on EuRoC + ATE/RPE vs Vicon & `robot_localization`.
Full doc: [`m3-vio.md`](m3-vio.md). *Server note: OpenVINS is an ordinary DDS
node; identical on the server.*

**M3 sub‑step — sim camera "solid‑colour at certain yaw angles" — SOLVED.**
The OAK‑D colour image flipped to a **solid uniform whole‑frame fill** = the scene
clear/`<background>` colour at **diagonal robot yaws**. Root‑caused to **two stacked,
independent bugs**, both now fixed and verified (`pipeline` + lidar, **0/8 headings blank,
std 84–90, NVIDIA renderer**):

1. **ogre2 was rendering in software (llvmpipe), not on the GPU.** `~/.gz/rendering/
   ogre2.log` showed `GL_RENDERER = llvmpipe` + `Texture memory budget exceeded`. The
   NVIDIA driver libs were injected but the **glvnd EGL vendor ICD `10_nvidia.json` was
   missing**, so headless EGL fell back to Mesa. **Fix:** write `10_nvidia.json`
   (`Dockerfile.sim`) → renders on the RTX 2070. (`NVIDIA_DRIVER_CAPABILITIES=all` is
   necessary but **not** sufficient — the vendor ICD file must exist too.)
2. **The camera was mounted below the chassis deck.** Even on the GPU a clean blank
   remained at the diagonals. Isolation: a **bare camera (no Husky) never blanks at any
   yaw**; with the Husky it does → the **robot** is required. Raising the camera removes
   it, with a **sharp threshold between z=0.20 (blanks) and z=0.25 (clean)** = the **A200
   top‑deck height (~0.245 m)**. Below the deck the chassis enters the frustum at diagonal
   yaws and trips a gz cull. **Fix:** mount the camera at **z=0.30** (`robot.yaml`),
   forward‑facing; OpenVINS `T_imu_cam` updated accordingly.

**Disproven along the way** (cautionary): `<sky>` (only set the blank *colour*), terrain‑
mesh frustum culling, world‑axis "diagonal resonance", rgbd vs plain camera, lidar↔camera
render‑thread contention, "old gz" (we're on Harmonic 8.11.0). The 4‑camera "contention"
result was **confounded by bug 1** (software rendering) and dissolved on the GPU. **Moral:
check `GL_RENDERER` first; isolate with the simplest scene (bare sensor) before theorising.**
Full write‑up: [`sim-debugging-notes.md`](sim-debugging-notes.md) "#8".

The `<sky>` strip was **removed** (sky re‑enabled — it never caused the blanks) and
`image_guard.py`/`camera_guard` are now **redundant** (no blank frames left to guard) but
kept as harmless. With the sim camera reliable across all headings, the M3 sim VIO run was
**re‑done on `pipeline`** with the **stereo** OAK‑D Lite pair (ATE 0.069 m raw / 0.077 m
fused — replaces the retired mono 0.148 m); M3b/EuRoC remains the real‑imagery
comparison.


**WildSeed procedural worlds — INTEGRATED + VERIFIED (2026-07-04).** The sim can
swap the `pipeline` world for any seeded
[WildSeed](https://github.com/ricardodeazambuja/WildSeed) world:
`scripts/prepare_wildseed_world.sh` packages world+models+terrain-sampled spawn z
into `worlds_external/<bundle>/`; `deploy.sh world <bundle>` selects it (world name
flows to gtbridge/scripts via `SIM_WORLD_NAME`). **Verified end-to-end on
`scenario --seed 42`** (alpine, 330 models): Husky spawns ON the terrain
(z=145.0), drives + terrain-follows, and **`deploy.sh m3-smoke` PASSES** (236/201
corners, 0 ms stereo sync, OpenVINS live; frames `results/wildseed_42_camera_*.png`).
Unlocks ATE-vs-terrain-complexity sweeps for M3/M4/M5 across seeds/biomes
(orchard/vineyard = loop-closure stress). Walls + workflow:
[`wildseed-worlds.md`](wildseed-worlds.md) (RTF wall: dense demo worlds → RTF 0.04
→ controller activation starves; ~330-model scenario worlds run RTF ≈0.35; a
controller watchdog in `husky_sim.launch.py` self-heals slow activations).

**Slow-sim robustness + measured RTF optimization — re-baselined (2026-07-04).**
Made the stack robust to slow sims (low RTF, weak machines) and optimized RTF on
*measured* evidence — `scripts/bench_rtf.sh`, the 4-variant discriminating bench
(full table: [`wildseed-worlds.md`](wildseed-worlds.md)). **What the measurement
killed:** shadows-off and Label-strip were *no-ops* for RTF (the "obvious"
render fruit doesn't exist here); dense worlds are **physics-step-bound**
(forest 0.034 → 0.149 at step 1→4 ms) and the robot's **render sensors cost
~half the throughput** (0.31 robot vs 0.59 world-only on wildseed_42). **What
shipped on that evidence:** (1) *sim-seconds contract* — all demo/smoke
durations (jerk-start, drive/record windows, CSV `t`) converted from wall to
SIM time via an identical helper block in `m3_vio_demo.py` / `gps_denied_demo.py`
/ `n1_drive.py` / `m3_smoke.py` (measures + prints RTF, aborts below
`SIM_RTF_FLOOR`); (2) `SLOW_SIM_FACTOR` multiplies the wall-clock control-plane
budgets (spawner handshake — the actual starvation knob — watchdog, world-ready);
(3) `deploy.sh rtf` + an info-only `rtf_probe` in the launch (WARN < 0.1);
(4) `tune_world_bundle.sh` (labels stripped + `--step 0.002` default in
`prepare_wildseed_world.sh`); (5) OS1 gpu_lidar **1024×64 → 512×32** in
`Dockerfile.sim`; (6) WildSeed-side `rig --no-labels` + `configs/sim-fast.yaml`
(WildSeed commit `3e9ec63`). **Net RTF: wildseed_42 robot 0.31 → 0.50–0.66** —
and it holds 0.66 even at CPUS=2 (weak-machine viable). **Verified:** at RTF≈1
(pipeline) everything re-passed — diag VERDICT PASS (now computed from Δt_sim),
N1 demo PASS (312 odom pts), m3-smoke PASS, **M3 re-baseline raw ATE 0.076 m /
fused 0.082 m** over 20.6 m (vs logged 0.069/0.077 — cameras untouched by the
lidar cut, within run variance), **keystone re-run rows=1381, on=0.21 →
denied=1.59 → reacq=0.45 m** (`results/gps_denied_verify.png` — drift is ~8×
the old reference because the 50 Hz sustained drive no longer stutters; the
error now visibly *snaps* back at reacquisition). On a genuinely slowed sim:
diag at **RTF 0.204** auto-scaled its windows and PASSed; m3-smoke at
**RTF 0.390** (wildseed_42 + vio at CPUS=2) printed its SLOW-SIM banner and
PASSed. *Honest caveats:* a pre-change m3-smoke FAIL at low RTF was not
re-recorded (the mechanism — wall jerk = 16 sim-ms at RTF 0.04, VIO never
initializes — is documented instead); the untuned-forest (RTF ~0.03) end-to-end
soak wasn't run (below the practical floor; `SLOW_SIM_FACTOR` is the escape
hatch there); **lidar recordings made before 512×32 are not comparable to new
ones** (M4 hasn't started, so nothing recorded is invalidated).

**Seed-42 terrain regenerated with the WildSeed slope cap (2026-07-04).**
Interactive driving exposed that the alpine terrain was unnaturally steep —
mean mesh slope **52.6°**, >90 % of the map beyond the Husky's ~20–25°
gradeability (the `mountainous` preset drew amplitude ≈ feature wavelength);
the robot terrain-trapped in the first gully (0.63 m progress per 12 s at
0.8 m/s commanded). Fixed at the generator (WildSeed `f1abe58`): scenario
worlds now rescale relief to a **20° mean surface slope** by default
(`--max-slope`, exact — slope is linear in height scale; consumes no RNG, so
the same seed keeps its layout). Regenerated + re-bundled `wildseed_42`:
mean slope 52.6° → **18.1°**, Husky-traversable (<25°) area 9 % → **76 %**,
relief 152 → 37.5 m, spawn z 145 → 36; drive test 0.63 → **2.7 m** per 12 s
(uphill), RTF 0.69, **m3-smoke PASS** (98/95 corners). Also fixed en route:
the Gazebo GUI needed the bundle models mounted + on its resource path
(client-side mesh resolution — commit `30b4bc5`), or a bundle world shows as
empty sky/grid while the sensor topics are fine.

**M4 — lidar frontend live on the sim: KISS-ICP → `ego_localizer` lidar hook →
ATE/RPE vs gz truth, A/B'd against the M3 stereo VIO in the same drive
(`results/m4_lio.png` pipeline · `results/m4_recipe_lio.png` WildSeed forest,
2026-07-06).** The architectural point landed exactly as designed: the second
frontend is `estimator.lidar_delta_update` — the *identical* body-frame-delta
measurement model as the visual hook — plus a subscription; **zero
`fusion_core` changes** (its 14 tests untouched-green; `ego_localizer` 11
green incl. a hook-equivalence test). New: `Dockerfile.kissicp` (source build,
pinned `1ffa7d7`, `lio` compose profile), `cloud_decimator.py` (20→5 Hz,
count-based/RTF-proof), `config/ego_localizer_lidar.yaml` (σ **fit from
measured residuals**, not copied from VIO), `deploy.sh m4-smoke` gate (PASS:
9.5k-pt clouds, LIO live, ratio 0.42 in the recalibrated 0.3–2.0 band),
`m4_lio_eval.sh` (steady-RTF gate + **fresh kissicp restart** — its local map
never resets, so load-transient scans poison whole runs; 12.8 m spurious pose
measured). **Numbers (same 23 m drive as M3, all streams same-run):** pipeline
— raw LIO ATE 0.985 m/RPE 0.263 m, fused 1.029/0.269, raw VIO 0.045/0.006;
forest — raw LIO ATE 4.008/RPE **0.100**, fused 3.856/0.101, VIO 0.046/0.006.
**Findings (full war stories: [`m4-lio.md`](m4-lio.md)):** (1) the *"OS1
512×32 in `Dockerfile.sim`"* logged above was a **silent no-op** — the
Clearpath *generator* hard-codes 1024×64 into the generated URDF over the
xacro defaults; now patched at the generator (guarded), so the cut is real
from this entry on (the RTF gains previously credited to it came from
step-size/labels); (2) cloud size is the registration-throughput lever
(65k-pt clouds → 6.6 Hz effective under motion; smaller voxel made it worse);
(3) KISS-ICP systematically **under-reports translation ~40–60 % at UGV
speeds** across every rate/voxel/threshold/world tried — per-scan motion sits
below its automotive design regime (its own `min_motion_th` guard implies
≥10 cm/scan); forest geometry fixes *local* consistency (RPE 2.6× better),
not the bias; ruled out: self-hits, deskew, QoS drops. This is the
failure-catalogue #1/#2 story measured on our robot — the M4 degeneracy
evidence, and the concrete case for absolute anchoring (M5). *Honest
caveats:* fused-LIO tracks its raw input (a relative-only filter cannot
correct a biased frontend); recipe-world ATE varies 2.3–4.0 m run-to-run as
the bias integrates path-dependently (RPE stable 0.06–0.10) — cross-world
comparisons should read RPE.

**M5-keystone re-run on WildSeed procedural terrain (2026-07-06,
`results/gps_denied_wildseed.png`).** The GPS-denied drift→reacquire demo,
previously verified on the hand-made pipeline world, now passes on the
`wildseed_42` bundle (alpine scenario, RTF 0.60): with the same protocol
(0.4 m/s, 40 sim-s denial) the error envelope rises **0.07 → 0.61 m across a
16.2 m dead-reckoned stretch (3.8 % of distance) and snaps back to 0.07 m at
reacquisition**; phase means on = 0.250 / denied = 0.368 / reacq = 0.212 m
(rows = 1304). The NavSat + `<spherical_coordinates>` shell-injection done by
`prepare_wildseed_world.sh` is what makes this work on any bundle unchanged.
Absolute drift is milder than the pipeline reference (denied mean 1.59 m
there) — dead-reckoning error is terrain-dependent; the qualitative keystone
(bounded → drift → snap back) is what the milestone claims. New:
`scripts/m5_keystone_eval.sh` (same steady-RTF bring-up gate as the M4 eval;
one-shot: world → gate → ego GNSS config → drive → chart). Optional keystone
polish (GPS 10 Hz, magnetometer, dual-EKF `map→odom`) remains open in §5 —
none blocking.

**Terrain-complexity sweep — VIO vs LIO vs fused across four worlds
(2026-07-07, `results/m4_terrain_sweep.png`).** Same drive + spine on pipeline
/ `vio_lio_bare` (open) / `vio_lio_recipe` (forest) / `wildseed_42` (alpine).
**Headline: complementary failure modes, measured.** The stereo VIO is superb
wherever texture exists (ATE 0.045–0.097 m) but **diverges on the alpine
world (45 m)** — verified texture starvation: a mid-route probe shows healthy
frames (std≈48) with only **57–58 corners, under the 80-corner KLT floor**
(spawn-point gate had 98/95 — texture is a runtime property, now measured).
The lidar frontend keeps working exactly there (5.36 m, fused 4.99 m). Lidar
local consistency is best in the forest (RPE 0.100) and worst on alpine
slopes (0.464). No single frontend survives all terrains — the
multi-frontend-spine + absolute-anchoring thesis with numbers. Sweep details
+ reading guidance (RPE vs ATE): [`m4-lio.md`](m4-lio.md).

**M6 — GTSAM factor-graph variant of the fusion core, A/B vs the hand-rolled
EKF (2026-07-07, `results/m6_ab.md` / `.csv`).** New
`fusion_core/factor_graph.py`: `PlanarFactorGraph`, an **ISAM2 pose graph
behind the exact `PlanarPoseEstimator` interface** (seed/predict/imu_rate/
odom_twist/visual_delta/lidar_delta/gnss/heading/state/covariance), so both
backends run on identical measurement streams. The relative hooks land in
their *native* factor form — every body-frame increment is a
`BetweenFactorPose2`, GNSS/heading are partial unary priors, the integrated
gyro a yaw-only between per interval. gtsam via the PyPI cp312 wheel 4.2.1 in
`Dockerfile.fusion` (apt `ros-jazzy-gtsam` ships no python bindings). Tests:
**fusion_core 18 passed** (14 EKF/models + 4 factor-graph behavioural twins:
frame cancellation, keystone drift→reacquire, covariance response).
**A/B (`scripts/m6_ab_benchmark.py`, same seeds through both):** accuracy
parity — keystone pos RMSE 0.925 m (EKF) vs 0.910 m (GTSAM), frontend-deltas
0.194 vs 0.184 — while the EKF is **3.5× cheaper on the GNSS-heavy scenario**
(update mean 188 µs / p95 279 vs 662 / 1383; near-parity 210 vs 221 µs on
delta-only streams, where the graph stays small). Supports the PLAN §3.2
choice: hand-rolled EKF as the live spine, GTSAM as the comparison branch.
*Modeling lesson (in the tests):* a smoother has no process noise to absorb
an overconfident measurement sigma — feed a biased odometry at an honest σ or
you measure your modeling error, not the backend.

## 5. Next steps — where the loop stops being laptop-closable

**The laptop-closable arc is closed (§4): M4 sim-first, the keystone on
WildSeed terrain, the terrain-complexity sweep, and the M6 GTSAM A/B are all
done.** What remains is gated by **external data / source builds**, which are
slow, failure-prone, and hard to "close the loop" on in one sitting — each
wants a focused session:
- **M3b — OpenVINS on EuRoC (deferred dataset comparison).** EuRoC ships as
  rosbag2 with OpenVINS: bag → OpenVINS → `ego_localizer` → `eval_tools` ATE/RPE
  vs **Vicon** truth and vs the `robot_localization` baseline — the recognizable
  *real-imagery* VIO numbers that complement the sim-first M3. Needs the EuRoC
  download.
- **M4 (real aerial) — KISS-ICP (apt) on NTU VIRAL.** The real-lidar tier (after
  the sim-first M4 above): lighter build (apt) but a large aerial download; the
  lidar relative update + the `gz_lidar_timestamp` deskew path on a real Ouster
  (native t+ring).
- **M4b — `dataset_publishers/`.** Buildable now, but only *verifiable* against a
  real dataset's native format (Livox `CustomMsg`, etc.) — pair it with M4's
  download.
- **M5 (real) — the keystone on MARS-LVIG RTK-GNSS aerial data + dual-EKF.** The
  sim version is done (§4).
- **M6 — GTSAM**; **M7 — `object_tracker` + YOLO** (KITTI/Boreas).
- **Keystone polish (optional, none blocking):** raise the sim GPS `update_rate`
  1→10 Hz (sharper chart — realistic for the `swiftnav_duro`; a `Dockerfile.sim`
  sed + rebuild), add a `magnetometer` (PLAN §5.1, closest to real HW), and
  `navsat_transform`/dual-EKF (PLAN §11 — absorb the reacquire jump in `map→odom`).

**Server reminder:** everything built so far is ROS-free libs or ordinary DDS
nodes — nothing laptop-specific, so it will run unchanged in the `compute`+`gui`
split once a server is available again. The only server-gated thing remains
`remote.sh` (guarded).
