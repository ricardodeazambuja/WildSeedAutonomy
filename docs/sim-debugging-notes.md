# Bringing up the Clearpath Husky headless on Jazzy + Harmonic — what we learned

A postmortem + reproduction guide for getting the Clearpath Husky (Ouster OS1 +
OAK‑D + Microstrain IMU) running **headless, in Docker, on ROS 2 Jazzy + Gazebo
Harmonic**, in the off‑road `pipeline` world. Getting sensors up was easy; getting
the **controllers + odometry + movement** up took root‑causing a cascade of seven
independent bugs. Everything here was found by the
scientific method: *hypothesis → controlled experiment → observe → conclude.*

Reproduce with `scripts/diag_sim.sh` (full health check) and
`scripts/expt_gz_ros2_control.sh` (the isolated controller experiment).

## State today
- ✅ Headless GPU sim: gz server‑only via EGL, no GUI to crash (see headless-gui.md).
- ✅ Sensors: `lidar3d_0/points` (~10 Hz), camera, `imu_0/data`, `/tf`.
- ✅ `controller_manager` + `joint_states` (~19 Hz) come up **reliably** (event‑driven launch).
- ✅ **`/clock` bridged & advancing** (~324 Hz), **odom** (~19 Hz), **movement** confirmed
  (teleop forward → odom x grows ~`linear.x × t`). #7 resolved — see below.

## The bugs, root causes, and fixes
| # | Symptom | Root cause | Fix | Where |
|---|---|---|---|---|
| 1 | spawned URDF had `clearpath_hardware_interfaces/A200Hardware` (real HW), no gz plugin | Clearpath's `a200.urdf.xacro` defaults `is_sim=false`; its generators never pass `is_sim:=true` (Jazzy/Harmonic rough edge) | flip the `is_sim` default to `true` in the platform xacros | `Dockerfile.sim` (sed) |
| 2 | `twist_mux`, `joy_linux`, `controller_manager` abort: *undefined symbol `diagnostic_updater::Updater::Updater(...)`* | osrf base image ships `diagnostic_updater 4.2.6`; Clearpath's freshly‑pulled pkgs built against `4.2.7` (ctor gained a param) | `apt-get upgrade` to align the snapshot | `Dockerfile.sim` |
| 3 | gz never loads `gz_ros2_control`; spawners time out (no controller_manager) | starting `gz sim -s` directly doesn't apply `ros_gz_sim`'s env → `GZ_SIM_SYSTEM_PLUGIN_PATH` empty → `libgz_ros2_control-system.so` not found | drive gz via **`ros_gz_sim/gz_sim.launch.py`**, which sets `GZ_SIM_SYSTEM_PLUGIN_PATH` from `LD_LIBRARY_PATH` + package exports (folds in the old explicit `=/opt/ros/jazzy/lib`) | `husky_sim.launch.py` |
| 4 | gz `Configure()` aborts: *"Error opening YAML file … control.yaml"* (in isolation only) | the xacro's `gazebo_controllers` default points to a **missing** package file; the real flow uses the generated `/clearpath/platform/config/control.yaml` | (no fix needed in the real flow — Clearpath sets the generated path explicitly) | — |
| 5 | model plugins silently not loaded; controllers absent | pipeline world meshes (`model://pipeline/*`, `model://accessories/*`) live under `clearpath_gz/meshes/`, not on `GZ_SIM_RESOURCE_PATH` → *"Failed to load a world"* → inserted‑model plugins skipped | set `GZ_SIM_RESOURCE_PATH` = worlds + meshes + all sourced package shares (mirrors `clearpath_gz`'s own `gz_sim.launch.py`) | `husky_sim.launch.py` |
| 6 | controllers came up only *intermittently* | fragile `gz & sleep N; robot_spawn` — the robot spawned before the heavy world finished loading | event‑driven launch: **wait on a condition** (`gz service -l` shows `/world/pipeline/create`) before spawning | `husky_sim.launch.py` |
| **7** | no odom / no movement; ROS `/clock` has 0 publishers | the gz→ROS `clock_bridge` node was **dead/absent** from `ros2 node list` on a 54‑min‑old container (exact trigger not fully isolated — multi‑gz was suspected but **disproven**, see below) → ROS `/clock` 0 publishers → every `use_sim_time:=true` node (diff_drive `controller_manager`, `ekf_node`) blocked on *"No clock received"* | **clean restart** with a live `clock_bridge` (a single fresh bringup). Bridge also simplified to Clearpath's canonical **plain `/clock`, `[gz.msgs.Clock`** form | `husky_sim.launch.py` |
| **8** | OAK‑D color image flips to a **solid uniform frame** = the scene's clear/background colour (`std=0`, whole frame) at **diagonal robot yaws** (≈45/135/225/315°); cardinals render fine. Was **yellow** with the procedural `<sky>`, **blue** (`<background>`) after we stripped `<sky>` | **TWO independent causes, both now fixed.** (A) ogre2's headless EGL fell back to **llvmpipe software rendering** (the NVIDIA EGL vendor ICD was missing), whose tiny texture budget stalled the camera's colour pass. (B) the camera was mounted **below the Husky A200 top deck** (z=0.20 < ~0.245 m): at diagonal robot yaws the deck enters the camera frustum and trips a gz/ogre2 cull → whole frame = background. (All earlier guesses — `<sky>`, terrain‑mesh frustum culling, world‑direction, rgbd vs plain, lidar render‑thread contention, "old gz" — **disproven**; see "#8 in detail".) | **(A)** register `/usr/share/glvnd/egl_vendor.d/10_nvidia.json` → renders on the RTX 2070 (`GL_RENDERER = NVIDIA…`). **(B)** raise the camera to **z=0.30** (≥0.25 clears the deck; sharp threshold — 0.20 blanks, 0.25 clean). Verified: pipeline + lidar, 0/8 headings blank, std 84–90. The `<sky>` strip + `image_guard.py` are now both redundant (kept, harmless). | `Dockerfile.sim` (EGL ICD), `robot.yaml` (camera z); root cause in **#8 in detail** |

### #7 in detail — isolating cause from coincidence
The original "recommended next step" was to drive gz via **`ros_gz_sim/gz_sim.launch.py`**,
on the theory that it provides the `/clock` bridge. **Reading the source corrected that:**
`ros_gz_sim/gz_sim.launch.py` only sets `GZ_SIM_SYSTEM_PLUGIN_PATH` / `GZ_SIM_RESOURCE_PATH`
— it has **no** clock bridge. The clock bridge lives in *Clearpath's* wrapper
`clearpath_gz/launch/gz_sim.launch.py`, which bridges **plain `/clock`** with
`'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'`. So the refactor is worth doing for
the #3/#5 env cleanup, but it does **not** by itself fix #7 — the clock bridge must
still be added explicitly, and a single gz server is what actually unblocks `/clock`.

**Symptom on the broken baseline.** A 54‑min‑old Husky container: `gz topic -l`
showed `/clock`, `/world/pipeline/clock` **and `/world/smoke/clock`** (three gz
servers — Husky, render‑smoke, a leftover `iso.sh` experiment — sharing the
`network_mode: host` gz‑transport), `gz topic -e -t /clock` was **silent**, ROS
`/clock` had **0 publishers**, and crucially the **`clock_bridge` node was not in
`ros2 node list`** — it had died or never started. A clean restart of the Husky
service (with the extras stopped) brought everything back.

**Two controlled experiments, because the restart changed several things at once.**

*1 — does the bridge form matter?* Single server, `diag_sim.sh` run twice, only the
bridge form changed:

| bridge form | `/clock` pub | odom movement (teleop fwd) |
|---|---|---|
| plain `/clock` `[gz.msgs.Clock` | 1 (~324 Hz) | ✅ x 1.27 → 5.76 m / 6 s |
| namespaced `/world/pipeline/clock`→`/clock` | 1 (~326 Hz) | ✅ x 0.00 → 4.49 m / 6 s |

→ **Both work.** The bridge form was never the blocker. We keep plain `/clock` only
because it is Clearpath's canonical form and simpler (no remap / no `use_sim_time`).

*2 — does a second gz server break `/clock`?* (The first‑draft root cause.) From a
**known‑good single Husky** (`/clock` 1 pub, odom live), bring up the render‑smoke
sim as a real 2nd gz server (`gz sim -s -r … smoke.sdf`) and re‑measure:

| gz servers | plain gz `/clock` | ROS `/clock` pub | odom under teleop |
|---|---|---|---|
| 1 (Husky) | alive | 1 (~326 Hz) | ✅ |
| 2 (+ smoke) | **still alive** | **still 1 (~326 Hz)** | ✅ x 1.43 → 3.70 m / 5 s |

→ **A second gz server did NOT break `/clock` or odom.** So "multiple gz servers"
is **not** the cause — that first‑draft conclusion was a coincidence of the
restart (server‑count *and* a fresh container with a live bridge both changed). The
reproducible remedy is a **clean bringup with `clock_bridge` actually running**;
why plain gz `/clock` was *silent* in the original 3‑server baseline was never
isolated (best guess: the dead bridge + stale state, not server count). When
`/clock` shows 0 publishers, **first check `ros2 node list` for `clock_bridge`** and
restart — don't chase the server count or the bridge topic.

**The refactor (done).** `husky_sim.launch.py` now drives gz through
`ros_gz_sim/gz_sim.launch.py` (`gz_args: '-s -r -v3 --headless-rendering <world>'`),
which sets the plugin path (subsuming the explicit #3 set) and appends model paths;
we keep an explicit Clearpath‑style `GZ_SIM_RESOURCE_PATH` (#5) and the explicit
`/clock` bridge (ros_gz_sim provides none). The #6 condition‑wait is preserved by
polling `gz service -l` for `/world/<world>/create` (it doesn't need to hook gz's
process start). Re‑verified end‑to‑end with `scripts/diag_sim.sh`: controller_manager
up ~6 s, sensors + `/clock` (1 pub, ~326 Hz) + odom (~19 Hz), teleop forward grows
odom `x` 0 → ~4.53 m / 6 s. No regression from the raw‑`gz` version.

### #8 in detail — two stacked causes, and the journey that found them
A long, scientific debug (instrumented in `scripts/{yaw_logger,multicam_logger,
rotate_full,step_capture,capture_fine}.py`; data in `results/`) peeled back **two
independent bugs**. The lesson: a single symptom ("solid‑colour camera") hid two
unrelated causes, and several confident intermediate hypotheses were **wrong**.

**Cause A — ogre2 was rendering in software (llvmpipe), not on the GPU.**
The smoking gun was in `~/.gz/rendering/ogre2.log`: `GL_RENDERER = llvmpipe` and
`Texture memory budget exceeded. Stalling GPU.` The NVIDIA driver libs are injected
into the container (`libEGL_nvidia.so` present, `NVIDIA_DRIVER_CAPABILITIES=all`), but
the **glvnd EGL *vendor ICD* JSON was missing** — only `50_mesa.json` existed, no
`10_nvidia.json` — so the EGL loader picked Mesa → llvmpipe. The camera's texture‑heavy
colour pass blew llvmpipe's tiny CPU‑RAM budget and shipped the clear colour; the
gpu_lidar's lighter depth pass squeaked under budget (→ "lidar never blanks", which
misled us toward a render‑*contention* story). **Fix:** write `10_nvidia.json`
(`Dockerfile.sim`) → `GL_RENDERER = NVIDIA GeForce RTX 2070`, texture‑budget stalls
gone. Note: `NVIDIA_DRIVER_CAPABILITIES=all` is necessary but **not sufficient** — the
vendor ICD file must also exist, or headless EGL silently falls back to software.

**Cause B — the camera was mounted below the chassis deck.** Even on the GPU a
clean **whole‑frame blank remained at diagonal robot yaws** (45/135/225/315°), `std=0`
= the `<background>` colour. The isolation chain:
- **Bare camera, no Husky** (`results/bare_*.sdf`): a camera spun through 0–90° in 10°
  steps **never blanks** (std 52–59). So it is **not** the camera, not gz core.
- **Husky present** → blanks return at the diagonals. So the **robot** is required.
- **Raise the camera**: at z=1.0 m the blank vanishes (0/8). Bracketing found a **sharp
  threshold between z=0.20 (blanks) and z=0.25 (clean)** — exactly the **A200 top‑deck
  height (~0.245 m)**. A camera below the deck is partly buried in the chassis; at a
  diagonal yaw the deck's geometry enters the (now world‑diagonal) frustum and trips a
  gz/ogre2 cull that blanks the whole render. **Fix:** mount the camera at **z=0.30**
  (forward‑facing, above the deck). Verified on `pipeline` with the lidar back: **0/8
  headings blank, std 84–90, NVIDIA renderer.**

**Hypotheses that were WRONG (kept as a caution):** procedural `<sky>` (only set the
blank *colour*); per‑object frustum culling of the offset terrain mesh; world‑axis /
"diagonal resonance"; rgbd_camera vs plain camera; lidar↔camera single‑render‑thread
contention; "old Gazebo" (we're on current Harmonic 8.11.0). The 4‑camera test that
looked like "render contention" was **confounded by Cause A** (on llvmpipe, cameras
facing texture‑heavy directions starved while others rendered) — it dissolved once on
the GPU. **Moral:** confirm the render backend (`GL_RENDERER`) *first*, and isolate with
the **simplest possible scene** (bare sensor, no robot) before theorising.

**Now‑redundant leftovers (kept, harmless):** the `<sky>` strip (`Dockerfile.sim`) and
`image_guard.py` / the `camera_guard` service. With both root causes fixed there are no
blank frames to guard; OpenVINS can read the raw `…/color/image` instead of
`…/image_guarded` if you drop the guard.

## Debugging method + gotchas (the meta‑learnings)
- **Scientific method beat guessing.** Each fix above came from a falsifiable
  hypothesis + a one‑variable controlled experiment. The big unlock was isolating
  `gz_ros2_control` by spawning the known‑good `is_sim:=true` URDF into an **empty**
  world (`scripts/expt_gz_ros2_control.sh`) — that's where the params‑file abort and
  later the plugin‑path/world dependencies became visible.
- **`ros2 topic echo` truncates long fields by default.** This cost real time: a
  full 22 KB `robot_description` read back as "150 bytes" and looked empty. Use
  `--full-length`. For scalars use `--field x` (or `--csv`).
- **`ros2 control …` needs `ros2controlcli`** installed — absent here, so we checked
  controllers via `ros2 node list` / `ros2 service list | grep controller_manager`.
- **gz "Failed to load a world" can be non‑fatal** (it recovers and sensors work)
  yet still **break model‑plugin loading** — don't dismiss it.
- **When ROS `/clock` has 0 publishers** (nodes log "No clock received", no odom):
  **check `ros2 node list` for `clock_bridge` first.** In #7 the bridge node had
  died on a long‑running container; a clean restart fixed it. Don't chase the bridge
  topic form or the gz server count — a controlled test showed neither breaks
  `/clock` (the namespaced bridge works with one server, and a 2nd gz server
  coexists fine). Keep the bridge on the GZ→ROS direction (`[gz.msgs.Clock`) anyway,
  so it only *subscribes* on the gz side.
- **`network_mode: host` ⇒ all gz servers share one gz‑transport partition**, so
  they see each other's topics (`/world/<name>/clock`, …). This was *suspected* in #7
  but a 2nd healthy gz server did **not** break the Husky's `/clock`/odom. If you run
  many sims and hit cross‑talk, isolate each with `GZ_PARTITION`.
- **Long sim startups need generous timeouts** on every wrapper (ssh, the agent's
  Bash tool default is 2 min) — and prefer **condition waits over `sleep`** (#6).
- **Use `use_sim_time:=true` everywhere that consumes sim data**, and make sure
  `/clock` is actually bridged and advancing — a `use_sim_time` node with no
  `/clock` blocks silently with only a warning.
- **Stale `header.stamp` ⇒ commands silently dropped.** A `use_sim_time` controller
  (diff_drive) rejects commands whose stamp is too old. `ros2 topic pub` always
  stamps **0**, so it only drives the robot right after a fresh bring-up (sim time
  ≈ 0) and is ignored later — looks like *random* "won't move". Drive from a node
  that stamps `clock.now()` (`scripts/n1_drive.py`, or `teleop_twist_keyboard
  stamped:=true`). Cost a long detour in N1.
- **Rogue ROS nodes survive a `kill`** (a classic ROS trap). A backgrounded
  `ros2 topic pub`/`run` often outlives a shell `kill`, keeps publishing, and
  silently drives the robot or holds a latch (a left-on `e_stop` latches → blocks
  ALL teleop). `pkill -9 -f <pattern>`, **verify `pgrep -fc` == 0**, add cleanup
  `trap`s. `pgrep`/`docker ps` whenever behaviour is inexplicable.
- **Bring the sim up FRESH, don't `restart`.** `docker compose restart husky`
  re-runs the launch against stale gz-transport state and leaves a half-working sim
  (controllers listed but no movement). `down` → `up` instead. Node start order &
  timing matter — sequence with launch **event handlers / condition-waits**, not
  `sleep` (#6).

## Reproduce on another machine
```bash
# host needs: Docker + NVIDIA Container Toolkit (./scripts/deploy.sh check)
./scripts/deploy.sh init && ./scripts/deploy.sh build      # builds with fixes 1,2 baked in
./scripts/diag_sim.sh                                       # full health check (fixes 3-6 in the launch)
./scripts/expt_gz_ros2_control.sh                           # isolated controller experiment (empty world)
./scripts/expt_gz_ros2_control.sh /opt/ros/jazzy/share/clearpath_gz/worlds/pipeline.sdf  # vs pipeline
```
