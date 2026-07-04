# Operations

Docker-only lifecycle for the sensing-node sim stack, driven by
`scripts/deploy.sh`. The same commands work on the laptop and the server; the
only per-host difference is `.env`.

## First run on a new machine

```bash
./scripts/deploy.sh check        # PASS/WARN/FAIL on Docker, NVIDIA, disk, X
./scripts/deploy.sh init         # creates .env from .env.example, tuned to host
$EDITOR .env                     # set ROLE (all | compute | gui)
./scripts/deploy.sh build        # builds sim + fusion images
./scripts/deploy.sh smoke        # DDS talker→listener sanity check
```

`init` auto-fills `CPUS` (= nproc − 2), `MEM_LIMIT` (≈ 75% RAM) and `DISPLAY`
from the host. Re-tune later without editing by hand:

```bash
./scripts/deploy.sh set --cpus 20 --mem 24g
./scripts/deploy.sh restart
```

## Commands

| Command | Does |
|---|---|
| `deploy.sh check [gui]` | host readiness; `gui` also checks for an X display |
| `deploy.sh init` | write `.env` (host-tuned); no-op if it exists |
| `deploy.sh build [svc]` | build images (`sim`, `fusion`) |
| `deploy.sh up [role]` | start services for `role` (defaults to `.env` ROLE) |
| `deploy.sh down` | stop all services |
| `deploy.sh smoke` | cross-container DDS test, returns non-zero on failure |
| `deploy.sh logs [svc]` | follow logs |
| `deploy.sh shell <svc>` | shell into a fresh container of that service |
| `deploy.sh set --cpus/--mem/--shm` | edit resource caps in `.env` |
| `deploy.sh restart [role]` | down + up |

## Roles → services (Compose profiles)

| ROLE | services | run it on |
|---|---|---|
| `compute` | `husky` (Clearpath Husky in the off-road `pipeline` world, headless) + `fusion` | the GPU box (server) |
| `gui` | `rviz` + `gzgui` (render locally) | the machine you sit at |
| `all` | compute + gui together | laptop-only |

Plus a standalone `render` profile (not tied to a ROLE): `deploy.sh render` runs
the minimal `smoke.sdf` world — the fastest GPU-render check, independent of the
heavy Clearpath stack.

### The simulated robot (`husky`)
Clearpath **Husky A200** carrying the three spine sensors (see
`ros2_ws/src/sensing_bringup/config/robot.yaml`):
- **Ouster OS1** 3D lidar on top → KISS-ICP lidar frontend
- **Luxonis OAK-D** stereo+depth, front → OpenVINS visual-inertial frontend (mirrors the real OAK-D)
- **Microstrain IMU** → the spine IMU (not the OAK-D BMI270; PLAN §8/§17.4)

in the outdoor **`pipeline`** world (rugged hills, river/bridge, cave). Change
the world by editing the `husky` service `command` (`world:=orchard`, `solar_farm`, …).

> **Headless (verified on the server):** `clearpath_gz`'s combined launch starts a
> GUI that dies without a display and drags the server down, so the `husky` service
> runs gz **server-only** (`gz sim -s --headless-rendering`) + `robot_spawn.launch.py`
> only, with `robot.yaml` copied into a writable `/clearpath`. Confirmed topics:
> `lidar3d_0/points`, camera, `imu_0/data`, `/clock`, `/tf`. Details in
> [`headless-gui.md`](headless-gui.md).

## Milestone-1 smoke tests (what "ready" means)

Per the PLAN, a milestone is done when its smoke tests pass — not when the
container merely launches.

1. **GPU compute in a container** — `deploy.sh check` runs
   `docker run --gpus all ubuntu:24.04 nvidia-smi -L`.
2. **Cross-container/host DDS** — `deploy.sh smoke` greps the listener for
   `I heard`.
3. **Headless GPU render (EGL)** — fastest check, no Clearpath:
   ```bash
   ./scripts/deploy.sh render                      # starts the smoke world
   ./scripts/deploy.sh shell sim                   # in another shell
   gz topic -e -n1 -t /smoke/camera                # non-empty image → ogre2/EGL works
   ```
4. **Husky sim sensors** — with `up compute` running, confirm the robot's
   lidar/camera/imu topics are live (namespaced `a200_0000`):
   ```bash
   ./scripts/deploy.sh shell fusion
   ros2 topic list | grep -E 'lidar3d|oakd|imu'
   ```

## Resource caps & hardening

Every service runs with `cpus`, `mem_limit` (= `memswap_limit`, so no swap),
`pids_limit`, `shm_size`, `no-new-privileges` and `cap_drop: NET_RAW` — the same
hardening idiom as the LTSpice deployment. Caps come from `.env`.

## Slow machines / low RTF

**RTF (real-time factor)** = sim time ÷ wall time. RTF 1.0 = real speed;
RTF 0.04 = 1 sim-second takes 25 wall-seconds. Weak machines and dense
[WildSeed worlds](wildseed-worlds.md) both push it down. Read it any time:

```bash
./scripts/deploy.sh rtf        # measured RTF + tier hint (sim must be up ~1 min)
./scripts/deploy.sh logs husky | grep rtf_probe    # auto-logged after bring-up
```

**The sim-seconds contract.** Everything *inside* the ROS graph runs on sim
time and scales gracefully. The demos and smoke gates (`m3-smoke`,
`gps_denied_demo.py`, `m3_vio_demo.py`, the N1 demo) define **all durations in
SIM seconds** and self-report the measured RTF: at RTF 0.1 a "40 s" demo takes
~400 wall-seconds but drives the identical path and records the identical
physics. They abort with a clear message if RTF < `SIM_RTF_FLOOR` (`.env`,
default 0.02 — below that the sim isn't meaningfully interactive).

**What does NOT scale by itself** is the wall-clock *control plane*: the
ros2_control spawner handshake, service calls, bring-up backstops. Those
budgets multiply by **`SLOW_SIM_FACTOR`** (`.env`, default 1):

| Symptom | Fix |
|---|---|
| `[rtf_probe] WARN: RTF < 0.1` at bring-up | raise `SLOW_SIM_FACTOR` to 2–5, `deploy.sh restart` |
| `Failed to activate controller …` / watchdog "recovering" loops | same — the spawner's wall-clock handshake starves ([wildseed-worlds.md](wildseed-worlds.md) RTF wall) |
| demos abort at the RTF floor | lighten the world (density — see below) or free CPUs |

**Tuning ladder** (measured evidence: `scripts/bench_rtf.sh`, results in
[wildseed-worlds.md](wildseed-worlds.md)):
1. **World density** — include count is the dense-world bottleneck
   (physics-step-bound): regenerate with `wildseed scenario --density-scale 0.5`.
2. **Physics step** — `scripts/tune_world_bundle.sh <bundle> --step 0.002`
   (prepare's default for new bundles); forest RTF scaled ~linearly with step.
3. **CPUs** — `deploy.sh set --cpus N && deploy.sh restart` (gz is CPU-hungry).
4. Shadows/sky off (`tune_world_bundle.sh --no-shadows --no-sky`) measured as
   **no-ops** on RTX 2070-class GPUs — try them only on much weaker GPUs.

## Datasets (later milestones)

`DATASETS_DIR` (default `../datasets`, gitignored) is mounted read-write into
`fusion` at `/datasets`. It is the hard-capped, self-pruning 60 GB area from
PLAN §9.1; the `fetch_seq.sh` / `run_seq.sh` / `prune.sh` tooling lands in
Milestone 4b.

## Verified configurations

| Host | Role | Verified | Notes |
|---|---|---|---|
| Desktop server + laptop | `compute` + `gui` | ✅ | headless sim on the server, RViz on the laptop over DDS (workflow B) |
| Laptop (Intel+NVIDIA hybrid, `DISPLAY=:1`) | `all` | ✅ | **whole stack on one box** (workflow A): check/build/smoke/render/diag_sim/viz all green; RViz hardware GL 4.6. See PLAN §19. |

> Laptop-only is the default — use `scripts/deploy.sh`. `scripts/remote.sh`
> (workflow B) needs a separate GPU server and refuses to run (pointing back here)
> if none is reachable.

## Known issues (build-validated; /clock + odom + movement verified on the server; laptop-only ROLE=all verified)

- **Controller stack not loading (no joint_states / odom / movement) — RESOLVED.**
  Root-caused by the scientific method (hypothesis → controlled experiment) to a
  chain of **FOUR independent bugs**, all now fixed; controller_manager loads and
  `/a200_0000/platform/joint_states` (≈20 Hz) + `/a200_0000/platform/odom` flow:
  1. **Real-hardware description in sim.** Clearpath's `a200.urdf.xacro` defaults
     `is_sim=false`, so the spawn used `clearpath_hardware_interfaces/A200Hardware`
     (real CAN hardware) instead of `gz_ros2_control/GazeboSimSystem`, and omitted
     the `gz_ros2_control` system plugin. **Fix:** flip the `is_sim` default to
     `true` in the platform xacros (Dockerfile.sim sed — sim-only image).
  2. **diagnostic_updater ABI skew.** The osrf base image shipped
     `diagnostic_updater 4.2.6`; Clearpath's freshly-pulled packages were built
     against `4.2.7` (the `Updater` ctor gained a param). Missing symbol →
     `twist_mux`, `joy_linux` AND `controller_manager` aborted. **Fix:**
     `apt-get upgrade` to align to the current snapshot (Dockerfile.sim).
  3. **gz can't find the plugin.** We start `gz sim -s` directly (headless), so
     `ros_gz_sim`'s plugin-path env isn't applied → `GZ_SIM_SYSTEM_PLUGIN_PATH`
     empty → gz never loads `libgz_ros2_control-system.so`. **Fix:** set
     `GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib` (compose `husky` env).
  4. **Pipeline world meshes unresolved → degraded world → model plugins skipped.**
     `model://pipeline/*` / `model://accessories/*` live under
     `clearpath_gz/meshes/`, not on `GZ_SIM_RESOURCE_PATH` (`…/share`), so gz threw
     "Failed to load a world" and didn't load the inserted model's plugins. **Fix:**
     prepend `…/clearpath_gz/meshes` to `GZ_SIM_RESOURCE_PATH` (compose `husky`
     command). Confirmed: "Failed to load a world" count → 0, controllers load.
- **Wheel odometry (§16.11) — now flowing.** `diff_drive_controller`
  (`platform_velocity_controller`, publish_rate 50, `odom_frame_id: odom`,
  `enable_odom_tf: False`) publishes `/a200_0000/platform/odom` once the controllers
  load. The toggleable `ego_localizer` input.
- **`joy_linux_node` symbol-lookup crash** was the same diagnostic_updater skew (#2)
  and is fixed too; keyboard teleop (`deploy.sh teleop`) remains the intended path.
- **ROS `/clock` → odom + movement — RESOLVED.** Symptom was
  `ros2 topic info /clock` → **0 publishers**, every `use_sim_time:=true` node
  ("No clock received") blocked, and the robot wouldn't drive. On the broken
  baseline (a 54‑min‑old container) the **`clock_bridge` node was not in
  `ros2 node list`** — dead/never‑started; a **clean restart** with a live bridge
  fixed it. Two controlled experiments pinned down what *didn't* matter: (1) both
  the namespaced bridge (x 0 → 4.49 m) and the plain `/clock` bridge (x 1.27 →
  5.76 m) drive odom — bridge **form** is irrelevant; (2) bringing up a **second**
  gz server (render‑smoke) against a healthy Husky left `/clock` at 1 pub and odom
  still moving (x 1.43 → 3.70 m) — so the first‑draft "multiple gz servers" theory
  is **disproven**. We still simplified the bridge to Clearpath's canonical
  `'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'` (no remap / no `use_sim_time`).
  Now `/clock` has 1 publisher ~324 Hz, `odom` ~19 Hz. `husky_sim.launch.py` also
  now drives gz via `ros_gz_sim/gz_sim.launch.py` (folds in #3 plugin‑path / #5
  resource‑path); that launch has **no** clock bridge, so the explicit `/clock`
  bridge stays. Full postmortem in [`sim-debugging-notes.md`](sim-debugging-notes.md) #7.
  ⚠️ When `/clock` shows 0 publishers, **check `ros2 node list` for `clock_bridge`**
  and restart cleanly — don't chase server count or bridge topic.
