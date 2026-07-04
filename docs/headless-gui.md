# Headless GUI: seeing Gazebo & RViz on a machine with no monitor

The server is headless; the laptop has a screen. We want **one image set** to
work for both "server + laptop" and "laptop-only", with no Windows-style browser
desktop (both clients are Linux). The chosen approach renders on the **local
GPU** and never streams pixels for the normal workflow.

## The split: render data on the GPU box, render *pixels* on your screen

```
        SERVER (headless, RTX A2000)          LAPTOP (where you sit, RTX 2070)
        ┌───────────────────────────┐         ┌────────────────────────────┐
        │ gz sim -s  (EGL, no window)│  DDS    │ rviz2     (local GPU GL)   │
        │ fusion / ROS nodes         │◄──────► │ gz sim -g (local GPU GL)   │
        │   publishes /topics,       │  gz-tp  │   subscribes & renders     │
        │   gz-transport scene       │         │                            │
        └───────────────────────────┘         └────────────────────────────┘
```

- **Gazebo** runs **headless** on the GPU box: `gz sim -s -r --headless-rendering`.
  EGL renders camera/lidar **sensor data** offscreen — no X, no window. The
  server already has `libEGL_nvidia`.
- **RViz** is a ROS 2 node: run it where you sit; it subscribes over DDS and
  renders on **your** GPU. This is the documented ROS 2 best practice and avoids
  the well-known "RViz over VNC/X-forwarding fails on OpenGL" problem.
- **Gazebo GUI** (`gz sim -g`) is a separate process from the server and talks
  **gz-transport**; run it where you sit to interact (play/pause/step, click,
  spawn) with local-GPU rendering.

`deploy.sh up gui` runs `rviz` + `gzgui`, mounts `/tmp/.X11-unix`, sets
`DISPLAY`, and runs `xhost +local:` so the containers can reach your X server.

## The two one-command workflows

```bash
./scripts/deploy.sh viz       # (A) sim LOCAL + RViz + Gazebo GUI — all on this box
./scripts/remote.sh viz       # (B) sim on the SERVER + RViz on the laptop (over DDS)
```
- **(A) all-local** brings up `husky` + `rviz` + `gzgui` (skips fusion). Full Gazebo
  GUI available. Drive with `deploy.sh teleop`, stop with `deploy.sh down`.
- **(B) server + laptop** runs `deploy.sh up compute` on the server over SSH and a
  **local RViz** that sees the server's topics over DDS. Drive with
  `remote.sh teleop`, stop with `remote.sh viz-stop`. **RViz only** — the cross-host
  Gazebo GUI needs extra gz-transport config (below), so for the live Gazebo window
  use (A). Server host/dir come from `SENSING_SERVER` / `SENSING_SERVER_DIR` (`.env`).

RViz loads `config/husky.rviz` (Ouster cloud, odom arrows, TF, OAK-D image, fixed
frame `odom`).

### Local-GPU GL: the `/dev/dri` + NVIDIA-graphics gotcha (verified)
The `rviz`/`gzgui` services need **hardware OpenGL**, which the compute-only GPU
reservation does NOT provide — symptom: `MESA: error: Failed to query drm device`
and a black/software 3D view. Fix baked into both GUI services:
`NVIDIA_DRIVER_CAPABILITIES=all` (graphics GL libs, not just compute) **and** a
`/dev/dri` device mount (the DRM render node — on hybrid laptops X runs on the iGPU).
After this, RViz logs `OpenGl version: 4.6`. The headless `husky` sim needs none of
this — its sensors render via EGL offscreen with compute caps only.

## The Clearpath `husky` sim, headless

`clearpath_gz simulation.launch.py` forwards only `world`/`rviz` and starts the
**combined** Gazebo (server+GUI) with no headless flag. On a headless box the GUI
can't create an OpenGL context and **its death takes the whole `gazebo` process
down** (verified: `[QT] Failed to create OpenGL context` → `process has died,
exit code -2`). `QT_QPA_PLATFORM=offscreen` does **not** save it.

**Implemented fix (verified on the server):** don't use the combined launch. The
`husky` service starts gz **server-only** with EGL offscreen rendering and then
runs *only* `robot_spawn.launch.py` (spawn + bridges, which does not start gz):

```bash
gz sim -s -r --headless-rendering .../clearpath_gz/worlds/pipeline.sdf &
ros2 launch clearpath_gz robot_spawn.launch.py world:=pipeline setup_path:=/clearpath/ use_sim_time:=true
```

Two gotchas handled in the `husky` service:
- **Writable `setup_path`.** The Clearpath generators *write* the generated
  description tree into `setup_path`, so `robot.yaml` is mounted read-only at
  `/clearpath-src` and copied into a writable `/clearpath` at startup (keeps
  generated files out of the repo).
- Sensors render on the GPU via EGL; you **interact from the laptop** (`ROLE=gui`
  → `rviz` / `gz sim -g`) over DDS / gz-transport — the same Option-1 path.

Confirmed live: `/a200_0000/sensors/lidar3d_0/points`, camera, `imu_0/data`,
`/clock`, `/tf`. (Known benign noise: `joy_linux_node` dies on a symbol-lookup —
a Clearpath teleop dep ABI skew; irrelevant headless.)

Fallbacks if ever needed: run on a machine **with** a display (`ROLE=all`), or a
virtual display (`xvfb-run`) wrapping the command.

### RViz sees nothing — the namespaced-TF gotcha

Symptom: RViz shows **"Fixed Frame [odom] does not exist"** and the lidar errors
**"could not transform [lidar3d_0_sensor_link] to [odom]"** — only stray frames
(e.g. an external VIO node's) appear, no robot.

Cause: the Clearpath robot publishes its **entire** TF tree on the **namespaced**
topics `/a200_0000/tf` and `/a200_0000/tf_static`, not the global `/tf[_static]`.
That tree is complete and live — `odom → base_link` (from Clearpath's own
`robot_localization` **`ekf_node`** at ~13 Hz; the `diff_drive_controller` keeps
`enable_odom_tf:false` so the two don't fight) plus `base_link → {lidar, camera,
imu, gps}` static — it's just on a topic RViz isn't subscribed to. RViz defaults to
the global `/tf[_static]`, finds nothing, and can't resolve any frame.

Fix (in the compose `rviz` service, so `deploy.sh rviz` / `viz` and `remote.sh` all
get it): **remap RViz's TF topics into the namespace**:

```bash
rviz2 -d …/husky.rviz --ros-args -r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static
```

`husky.rviz` Fixed Frame stays `odom` (it exists once remapped). Verify headlessly
without a screen:

```bash
ros2 run tf2_ros tf2_echo odom lidar3d_0_sensor_link \
  --ros-args -r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static   # -> a transform, not "does not exist"
```

Note: an external node (e.g. OpenVINS, M3) may publish its *own* small TF island
(`imu → cam0`) to the **global** `/tf`; that's why RViz on the global `/tf` showed
only those frames. The remap points RViz at the robot tree instead. The lesson: in a
namespaced ROS 2 robot, **every TF consumer (RViz, tf2_echo, fusion nodes) must use
the namespaced `/tf[_static]`**, or it silently sees an empty/partial tree.

## Single host vs. two hosts

**Laptop-only (`ROLE=all`):** everything is localhost; DDS multicast and
gz-transport "just work". Nothing extra to set.

**Server + laptop** (`remote.sh viz`): both must share the LAN.
- DDS: same `ROS_DOMAIN_ID` on both; with host networking, FastDDS multicast
  discovers across the LAN. **Verified** on this setup: a laptop container on
  `ROS_DOMAIN_ID=42` sees the server's 39 `/a200_0000/*` topics and `platform/odom`
  flows at ~19 Hz — so `remote.sh viz` (server sim + local RViz) works out of the
  box. If multicast is blocked, switch to CycloneDDS unicast — see below.
- Gazebo GUI cross-host: on the laptop set `GZ_IP=<laptop-LAN-IP>` (already a
  knob in `.env`) and the same `GZ_PARTITION` as the server, then `gz sim -g`.
  ⚠️ Cross-host `gz -g` is supported but can be finicky (a known empty-window
  case unless the world has a camera, and `GZ_SIM_RESOURCE_PATH` must point at
  the same worlds/meshes — we mount the repo into both, so that part is handled).

### CycloneDDS unicast fallback (multicast blocked)
In `.env` on **both** machines:
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds.xml
```
Then uncomment the server + laptop peers in `docker/cyclonedds.xml`.

## When you genuinely need a full interactive desktop *on the server*

Family 1 above covers normal use. For the rare case you must run the whole
interactive GUI server-side and just view/click it remotely, stream pixels from
a GPU virtual display. Ranked for a Linux→Linux, NVENC-capable setup:

| Option | Why | Client |
|---|---|---|
| **Amazon/NICE DCV** | turnkey GPU remote-viz, adaptive H.264/265, free | native Linux + browser |
| **Sunshine + Moonlight** | hardware NVENC → lowest latency (A2000 has NVENC) | native Linux (Moonlight) |
| **NoMachine** | NX, GPU-accelerated, easy setup | native Linux |
| **VirtualGL + TurboVNC** | HPC classic; ready ROS/Gazebo Docker recipes; robust on slow links | TurboVNC / web |

These all need a GPU virtual display on the headless host; DCV and Sunshine
provide their own more readily than bare VirtualGL+Xorg. This path is
intentionally **not** wired into the default compose — it's the documented
escape hatch, not the daily driver.
