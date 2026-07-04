# M3 — Visual-Inertial Odometry (VIO) on the sim

The visual frontend of the sensing spine, validated **in sim first** (PLAN §12 M3). OpenVINS estimates ego-motion from the OAK-D camera + IMU; the
`ego_localizer` EKF fuses that as a relative-motion source; we score it against the
Gazebo ground-truth pose. The EuRoC/real-dataset comparison is split into M3b.

## Refresher: where VIO sits in the stack

- **VO / VIO / SLAM.** *Visual Odometry* tracks image features frame-to-frame to
  estimate incremental motion — it **drifts** and has no map. *Visual-Inertial
  Odometry* fuses an IMU to make scale observable (monocular VO can't recover metric
  scale on its own) and to survive motion blur / low-texture gaps. *SLAM* adds a map
  + loop closure to cancel drift — at the cost of **retroactive jumps** when a loop
  closes, which complicate any downstream filter. We want odometry, not SLAM (§3.2).
- **Filter vs optimization VIO.** OpenVINS is an **MSCKF** (Multi-State Constraint
  Kalman Filter): a *filter* that keeps a sliding window of recent camera poses in
  the state and marginalizes old ones — bounded, real-time, no loop closure. The
  optimization camp (VINS-Fusion, ORB-SLAM3) re-solves a bundle-adjustment window;
  more accurate, heavier, and they do loop-close. We pick the filter because its
  output is **clean to fuse** (no jumps) — exactly the property `ego_localizer` needs.
- **Loosely vs tightly coupled.** *Tightly* coupled = raw features + IMU in one
  estimator (that's what OpenVINS does **internally**). At the **system** level we
  fuse *loosely*: OpenVINS is a black box emitting a pose; `ego_localizer` consumes
  that pose, **not** its raw IMU. The rule (§3.2) is **don't double-count the IMU** —
  the spine IMU feeds the EKF directly; the camera-IMU lives inside OpenVINS only.

## The robot's camera (what we actually have in sim)

| Property | Value | Implication for VIO |
|---|---|---|
| Streams | **2** RGB — `.../camera_0/color/image` (LEFT) + `.../camera_1/color/image` (RIGHT), `rgb8` | **stereo** VIO; 0.075 m baseline = OAK-D Lite |
| Resolution | **320×240** each | low; fewer sub-pixel features, but enough (gate below) |
| Rate | **~6.3 Hz** sim-time (two rgbd cams) | low for VIO; OK because the robot drives slowly |
| `camera_info` | **absent** | intrinsics supplied **manually** in the OpenVINS config |
| IMU | `.../sensors/imu_0/data` (Microstrain) | the camera-IMU pair OpenVINS needs |

**Stereo, not mono**: monocular VIO is scale-degenerate on a
smooth planar UGV (scale comes only from IMU accel excitation, which a constant-velocity
Husky barely provides — mono wouldn't even initialise here). The stereo baseline makes
scale directly observable. The missing `camera_info` means we compute `fx,fy,cx,cy` from
the sim camera's resolution + horizontal FOV and hard-code them per cam in the estimator
config. (Sim cams are the ideal-pinhole COLOR pair — proxies for the real OAK-D Lite's
mono global-shutter L/R VIO cameras; in an ideal sim that distinction doesn't bite.)

## The texture gate (PLAN §14 / §17.4 wall) — PASS

A KLT-tracking VIO is only as good as the corners it can hold. Before building any
fusion we verified the world is trackable with `scripts/check_sim_texture.py` (grabs
one frame, counts Shi-Tomasi + FAST corners — the same kind OpenVINS tracks):

```
image      : 320x240  encoding bgr8
Shi-Tomasi :  264 strong corners (KLT-trackable)
FAST       :  193 corners
Laplacian  : var=230.3
VERDICT    : PASS  (PASS>=150  MARGINAL>=50  else FAIL)
```

`results/sim_camera_features.png` shows the corners spread across the textured
ground and horizon (the off-road `pipeline` world) — good parallax for translation
and scale, not clustered on one patch. A few land on clouds (distant, no parallax);
OpenVINS naturally down-weights those. **Conclusion: the world has enough texture;
M3 is not blocked by the §14 starvation risk.** Re-run the gate if the world changes.

## Pipeline (what runs where)

```
husky sim ─┬─ /…/camera_0/color/image ─► camera_guard ─ /…/image_guarded ─┐
           └─ /…/imu_0/data ──────────────────────────────────────────────┴─► OpenVINS ─ /odomimu ─► ego_localizer ─► /ego_localizer/odom
                                          (drops blank frames)               (MSCKF VIO)   (VIO pose)   (EKF: VIO Δ + IMU yaw-rate)   │
   gz model pose ─ /world/pipeline/dynamic_pose/info ─(ros_gz bridge)──────────────────────► ground truth ───────────────────────────┴─► eval_tools ATE/RPE
```

- **camera_guard** (`sensing_bringup/scripts/image_guard.py`, in the `vio` profile)
  drops near-uniform camera frames before OpenVINS sees them. **⚠️ Now REDUNDANT** —
  the "solid-colour frame at certain yaw angles" was root-caused (sim-debugging-notes #8)
  to two real bugs, both since fixed: (1) ogre2 rendering in **software/llvmpipe** (missing
  NVIDIA EGL ICD), and (2) the camera mounted **below the chassis deck** (z=0.20 → now
  z=0.30). With those fixed the camera renders cleanly at every heading (0/8 blank), so
  there are no blank frames left to drop. (It is **not** a "frustum-culling artefact" — that
  was a disproven hypothesis.) The guard is kept as a harmless safety net; OpenVINS can
  read the raw `…/color/image` directly if you drop it.

- **OpenVINS** runs in its own image (`sensing-node/openvins:local`) as
  `run_subscribe_msckf`, config in `config/openvins/`. **Stereo** (`use_stereo: true`,
  `max_cameras: 2`) on the OAK-D Lite L/R pair. Output topic is `/odomimu`.
- **ego_localizer** consumes `/odomimu` as a *relative* body-frame increment
  (`visual_delta_update`) + the IMU yaw-rate. VIO alone — **no wheel odom, no GNSS**
  (`gps_enabled: false`; GNSS fusion is M5's keystone) — so the chart reflects the
  visual frontend only. See `config/ego_localizer_visual.yaml`.
- **Ground truth** is the gz model pose, bridged from `/world/pipeline/dynamic_pose/info`
  (a `Pose_V`); the first transform is the robot model root. ATE/RPE use Umeyama
  alignment, so OpenVINS' arbitrary frame and our origin seed don't matter.

## How to run

With the sim up (`./scripts/deploy.sh up compute`), in three steps:

```bash
# 1. OpenVINS (its own container, same DDS domain)
docker run -d --name sensing-openvins --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$PWD/ros2_ws/src/sensing_bringup/config/openvins:/config:ro" \
  sensing-node/openvins:local \
  bash -lc "source /ov_ws/install/setup.bash && ros2 run ov_msckf run_subscribe_msckf \
            --ros-args -p config_path:=/config/estimator_config.yaml"

# 2. ground-truth bridge (gz model pose -> ROS TF)
docker run -d --name sensing-gtbridge --network host --ipc host -e GZ_PARTITION=sensing \
  -e ROS_DOMAIN_ID=42 sensing-node/sim:local bash -lc "source /opt/ros/jazzy/setup.bash && \
  ros2 run ros_gz_bridge parameter_bridge \
  '/world/pipeline/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'"

# 3. ego_localizer (visual config) + the drive/record + the chart — in the fusion container
deploy.sh shell fusion   # then, inside:
PYTHONPATH=/ros2_ws/src/fusion_core:/ros2_ws/src/ego_localizer:$PYTHONPATH \
  python3 /ros2_ws/src/ego_localizer/ego_localizer/node.py --ros-args \
  --params-file /ros2_ws/src/ego_localizer/config/ego_localizer_visual.yaml &
python3 scripts/m3_vio_demo.py /results/m3 0.5 0.1 45          # jerk-start + drive + record
PYTHONPATH=/ros2_ws/src/eval_tools python3 -m eval_tools.evaluate \
  --gt /results/m3_gt.csv --est ego_localizer:/results/m3_ego.csv \
  --est openvins_raw:/results/m3_ov.csv --out /results/m3_vio.png
```

`m3_vio_demo.py` is self-contained: it **jerk-starts** the robot (two sharp forward
jabs) to fire OpenVINS' static init — a smooth diff-drive ramp never trips the accel
threshold — then drives the recorded curve at 50 Hz (publishing slower lets twist_mux
time the command out, so the robot stutters and mono/stereo VIO starves on no motion).

All the demo's durations — the jerk jabs, the warmup, the `secs` drive window, the
CSV `t` column — are **SIM seconds**: on a slow sim (low RTF) the run stretches in
wall time but the driven path, accel transients and recorded physics are identical.
The script measures and prints the RTF at start and aborts below `SIM_RTF_FLOOR`
(see [operations.md](operations.md) "Slow machines / low RTF"). Same contract in
`m3_smoke.py` and `gps_denied_demo.py`.

## Smoke gate — `./scripts/deploy.sh m3-smoke`

A fail-fast regression check (the "texture/feature gate" of PLAN §14/§17.4, extended to
the whole stereo path). Brings up the sim + vio stack and asserts, with PASS/FAIL +
exit code: **(1)** both cameras render a *non-blank* frame (catches the EGL/llvmpipe +
sub-deck-mount render bug #8), **(2)** each yields enough Shi-Tomasi corners for KLT to
hold lock (≥80; a starved front-end is silently worthless), **(3)** the stereo pair is
time-synced (a guarded/raw topic mix would desync it), and **(4)** OpenVINS publishes
`/odomimu` after a jerk-start (stereo init fired). Run it after any change to the camera
mount, world, or OpenVINS config — green means the pipeline is alive *and tracking*, not
just "the nodes launched." Script: `ros2_ws/src/sensing_bringup/scripts/m3_smoke.py`.

## Result (sim, laptop-only) — STEREO

`results/m3_vio.png` — over a **20.5 m curved drive** vs gz ground truth, GNSS off:

| trajectory | ATE rmse | RPE rmse |
|---|---|---|
| raw stereo OpenVINS | **0.069 m** | 0.0042 m |
| fused ego_localizer (VIO + IMU yaw-rate) | **0.077 m** | 0.0091 m |

Both hug ground truth; the fused output is ~the same as its raw VIO input (a well-tuned
fusion never makes its best input materially worse). ATE 0.069 m on 20.5 m is ~0.3% of
path. The alignment is **rigid SE(3)** Umeyama (`with_scale=False`) — no scale fudge — so
0.069 m is a genuine *metric* number, which is the whole point of going stereo.

> **Why stereo (changed from mono).** Monocular VIO is **scale-degenerate on a
> smooth planar UGV**: absolute scale comes only from IMU accel excitation, and a Husky
> cruising at constant velocity on flat ground barely excites the accelerometer. Mono
> OpenVINS wouldn't even *initialise* here (static init needs an accel jerk the ramp never
> makes; dynamic init needs rotation-disparity the straight start never builds), and when
> forced it diverged to km. The fix is **geometric, not parametric**: the OAK-D Lite's
> stereo baseline (0.075 m) makes depth/scale directly observable — no IMU-excitation
> dependence. Two `luxonis_oakd` instances (camera_0=LEFT, camera_1=RIGHT) in `robot.yaml`;
> `use_stereo: true`, `max_cameras: 2`; cam0/cam1 share `R_CtoI` and differ only in
> `p_CinI.y` (±0.0375), with the kalibr y-sep equal to the mount y-sep. (Sim uses the
> ideal-pinhole COLOR cams as proxies for the real OAK-D Lite's mono global-shutter L/R
> VIO pair.) The old mono ATE 0.148 is retired: it was measured on a *blanking* camera —
> dead-reckoning, so VIO was never actually exercised.

## The lesson that bit us: texture is a *runtime* property, not a one-frame gate

The static texture gate passes on the start frame, but a VIO needs texture **for the
whole run**. On the first attempt the robot drove ~60 s on a curve, wandered until the
camera faced featureless terrain/sky (raw frame went to a uniform gray, std = 0), and
with zero tracked features OpenVINS had **no visual constraint** — orientation stayed
fine (gyro) while position ran away to >7000 m on pure IMU integration. The tell-tale
divergence signature: *stable q_GtoI, exploding p_IinG*. The fix was a fresh start at
the textured spot + a short, gentle, near-straight drive that keeps the scene in view.
Takeaway: when a filter-VIO diverges, check the **tracker image** (`/trackhist`) and
the **raw frame's std** before suspecting the filter — a blank frame starves it.

> **Correction:** the `std=0` "featureless terrain/sky" frames above were
> **not** genuine low texture — they were the **camera-render bug** (software/llvmpipe +
> camera below the chassis deck), since fixed (sim-debugging-notes #8). So "texture is a
> runtime property" was a *misread* of this particular symptom: the camera was blanking for
> a render reason, not because the scene was untextured. The general takeaway still holds
> (a blank frame starves a VIO, so check the raw-frame std), but with the render fixed the
> camera no longer blanks at any heading, so the short/gentle workaround drive is no longer
> needed — re-run M3 over a normal route.

