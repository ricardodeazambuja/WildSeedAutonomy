# Project Plan — Sensing Node: Sensor‑Agnostic Edge Fusion + GPS‑Denied Localization

> A reproducible, Dockerized ROS 2 + Gazebo project that doubles as a public tutorial.
> One low‑cost "node" that (a) knows where **it** is by fusing **GNSS/INS + a pluggable odometry source
> (lidar‑inertial *and* visual‑inertial) + IMU**, and degrades gracefully when GPS drops, and
> (b) tracks external objects — first in simulation on a laptop, later on a Jetson, with the **same
> application code** (arch‑matched GPU base — see §13) and a measured latency/power delta.
>
> Design north star: mirror the architecture autonomy companies actually field — a **classical
> estimation + planning spine** (factor graph / EKF) with **pluggable, swappable odometry frontends** —
> rather than clone any single SLAM repo.

**Status:** lives in [`status-and-testing.md`](status-and-testing.md) (the single per-session-updated home — milestone table, verification log, next steps). This document is the *timeless* design/rationale (v2.1 — sensor‑agnostic spine + nav modes). **All version/size/stack facts web‑verified** (sources + confidence inline; re‑check before you start since releases move).

---

## 0. TL;DR decisions

| Decision | Choice | Why (one line) |
|---|---|---|
| **Architecture** | **Sensor‑agnostic fusion core + pluggable odometry frontends** | Matches what's fielded (Shield AI fuses "IMUs, GNSS, visual odometry, radar"; RACER runs lidar‑visual‑inertial). The *fusion*, not any one frontend, is the focus. |
| **Odometry frontends** | **Lidar‑inertial (primary) + Visual‑inertial (secondary)**, both feeding one fusion core | "In the wild" credibility (lidar) + exercises your OAK‑D (visual). You chose **both**. |
| ROS 2 distro | **Jazzy Jalisco** (LTS, EOL May 2029) | Latest *fully usable* distro: mature binaries + clean Jetson path. |
| Simulator | **Gazebo Harmonic** (LTS, EOL May 2029) | Official **binary** pairing with Jazzy (verified in `ros_gz` table). |
| Why not newest (Lyrical + Jetty)? | v2 branch only | Lyrical ~1 month old, immature binaries, needs Ubuntu 26.04 which **no JetPack ships** → breaks Jetson. |
| Fusion backend | **Hand‑rolled EKF/UKF (showcase)** + **GTSAM** factor‑graph + **`robot_localization`** as baselines | All three apt‑installable on Jazzy (GTSAM 4.2.0 verified). A/B comparison = credibility. |
| Your OAK‑D Lite | **Stereo‑depth + RGB + object‑detection sensor; NOT the sole inertial source** | Its BMI270 IMU is the weakest‑for‑VIO in the OAK family — design around it (§8). |
| Host OS impact | **None** | Ubuntu 22.04 host; Docker carries Ubuntu 24.04 inside the Jazzy image. |
| **Platform** | **Ground UGV — Clearpath Husky (sim)** | **Not aerial.** The goal is a simple, GPS‑available ground UGV; the aerial datasets (§9) are kept only as *optional, harder‑than‑KITTI* stress tests, **not** the spine. |
| Start mode | **Recorded datasets first**, Gazebo second | Estimator doesn't need a rendering sim to start; datasets give free ground truth + error curves day one. |
| GPU verdict | **RTX 2070 Mobile (8 GB) is enough** | Lidar odometry + OpenVINS are largely CPU; Gazebo + small YOLO fit 8 GB. Caveats §10. |
| Disk verdict | **Bounded, not cumulative** | Docker ~15 GB + **hard‑capped 60 GB `datasets/`** that streams seq‑by‑seq and auto‑prunes (§9.1). Datasets pass through, never pile up; steady‑state <25 GB, peaks ~60 GB only during a Boreas seq. **Never pull a full set** (TartanAir 57 TB) — `fetch_seq.sh` uses selective downloaders. 174 GB free → never close to tight. |
| **Resilience** | **Every layer has a ranked fallback ladder + a dataset‑replay floor** | No single abandoned/unbuildable package can stall the project — see **§3.3** pivot matrix. The whole stack can drop Jazzy→Humble pre‑validated. |
| **Control / navigation** | **Three operator modes — remote‑control · semi‑auto (waypoints + obstacle avoid) · full‑auto (goal → plan)** | The "planning" half of the spine, layered on the fusion via **Nav2 + `twist_mux`**, not a rewrite — see **§18**. |

---

## 1. Grounded in what autonomy companies actually field (2025–2026)

This was researched with web sources + GitHub liveness checks. **Confidence tags:** [F]=primary/official source · [I]=inference from evidence. Internal stacks are mostly proprietary, so company‑specific items are evidence‑based, not insider fact.

**The dominant fielded pattern is consistent across independent sources:** a **classical estimation + planning spine, with learned perception layered on top, on Jetson Orin/Thor edge compute.** Not end‑to‑end neural nets; not a SLAM black box.

- **State estimation is domain‑segmented but always classical‑core** [F]:
  - Off‑road ground → **lidar‑(visual‑)inertial odometry** (CMU/JPL DARPA RACER run "Super Odometry," a LiDAR‑visual‑inertial fusion).
  - Air / GPS‑denied → **visual‑inertial** (Shield AI: V‑BAT navigates "using a combination of visual and inertial sensors… irrespective of GPS").
  - Warehouse → **cuVSLAM** stereo‑visual‑inertial (NVIDIA Isaac Perceptor; named customers KION/ArcBest).
  - Quadruped → **graph localization** (Boston Dynamics GraphNav, fully documented).
- **The backend is a factor graph / EKF** [F]: Anduril's state‑estimation work names "Ceres, GTSAM" + "SLAM"; Shield AI's stack names "EKF, UKF, particle filters." **This is the layer that defines a serious fusion stack — and it's exactly what this project's hand‑rolled filter + GTSAM comparison targets.**
- **ROS 2 + Nav2 is genuinely production** in warehouse/inspection ("trusted by 100+ companies"), shipping **MPPI** + **BehaviorTree.CPP**. **Off‑road autonomy leans proprietary** (Anduril's drone stack uses behavior trees but *not* ROS) [F/I].
- **GPS‑denied architecture** [F] = tactical **GNSS/INS** (VectorNav / SBG / OxTS / Septentrio) in an EKF, drift‑reset by lidar/visual odometry + place recognition, with anti‑jam/anti‑spoof on the GNSS input. OxTS literally sells "LiDAR Boost… compensate for missing or erroneous GNSS data in real time." **This project's exact thesis.**
- **Learned‑perception trend is real but specific** [F]: learned *traversability* → classical MPPI planner is fielded (DARPA RACER demoed with the US Army's 36th Engineer Brigade, Oct 2025). **VLMs/foundation models are real in the *toolchain* (synthetic data) but not yet a fielded on‑robot perception front‑end** — that part is still hype.

**Key honest caveat** [F]: the famous open‑source SLAM packages (FAST‑LIO2, Point‑LIO, OpenVINS, VINS‑Fusion) are alive in **academia** but show **no confirmed evidence of being shipped by an off‑road autonomy company** — they build proprietary using the same *techniques*. **So the goal here is to demonstrate the technique + fusion architecture, not to clone a repo.** That's why the spine here is sensor‑agnostic and the filter is hand‑rolled.

> On **VINS**: VINS‑Mono (2018) / VINS‑Fusion (stereo + IMU + **GPS** fusion) were *the* reference VIO and are the conceptual ancestor of this project's GPS‑aided story — but both are now in maintenance (last push 2024). Today you'd reach for **OpenVINS** (active, Nov 2025) on the visual side or a **lidar‑inertial** frontend on the ground side. Cited as lineage, not used as a dependency.

---

## 2. What this project demonstrates (capabilities)

The headline is **not** "I called `detect()` and drew boxes." The substance is in the middle layer:

- **Multi‑sensor fusion** — a hand‑rolled EKF/UKF (and a GTSAM factor‑graph variant) over heterogeneous, asynchronous sensors: IMU (100–250 Hz), **lidar‑inertial odometry** (~10 Hz), **visual‑inertial odometry** (~10–20 Hz), GNSS (1–10 Hz), with **out‑of‑order / time‑sync handling** and **online sensor health switching** (drop a degraded frontend, keep going).
- **Sensor‑agnostic design** — a clean odometry‑adapter interface so lidar‑inertial *or* visual‑inertial (or both) plug into the same fusion core. This *is* the architecture real companies describe.
- **Graceful degradation** — explicit GPS‑dropout + a measured **drift → reacquisition** story; bonus: frontend‑dropout (lidar fails in open field / vision fails in whiteout).
- **Calibration** — explicit sensor extrinsics + time offset, not magic numbers.
- **Explainable output** — covariance/confidence ellipses rendered live (interpretable uncertainty).
- **Edge on COTS** — the *same application code* (arch‑matched container, §13) on a Jetson with a measured **latency + power** delta. One chart tells the whole "cheap hardware, real capability" story.

Deliverable pattern: **public repo + one‑page writeup + a ~60 s clip.**
Money charts:
1. **GPS‑dropout drift** — error growing during outage, snapping back on reacquisition (run for *both* frontends → a comparison).
2. **Frontend comparison** — lidar‑inertial vs visual‑inertial accuracy/robustness on the same trajectory.
3. **Tracking accuracy** — MOT metrics or position RMSE on tracked objects.
4. **Edge delta** — end‑to‑end latency + CPU/GPU/power, laptop vs Jetson.

---

## 3. Verified technology stack

### 3.1 Core (fact‑checked)

| Component | Version | Verification |
|---|---|---|
| **ROS 2** | **Jazzy Jalisco** (LTS, May 2024, EOL May 2029, Tier‑1 Ubuntu 24.04 Noble) | REP 2000. |
| **Gazebo** | **Harmonic** (= Gazebo Sim 8, LTS, Sep 2023, EOL May 2029) | gazebosim.org releases. |
| **ROS↔Gazebo** | **`ros_gz`** Jazzy branch, **binary** from packages.ros.org | `ros_gz/README.md` table: `Jazzy | Harmonic | … | packages.ros.org`. |
| **RMW (DDS)** | default `rmw_fastrtps_cpp`; **`rmw_cyclonedds_cpp`** fallback | §7.3. |

> **Release landscape** (so the tutorial can say "as of mid‑2026"):
> ROS 2: Humble (2022 LTS) · Jazzy (2024 LTS) · Kilted (2025, **EOL Nov 2026** → avoid) · Lyrical (May 2026 LTS, too fresh).
> Gazebo: Fortress (LTS) · Harmonic (LTS) · Ionic (**EOL Dec 2026** → avoid) · Jetty (Sep 2025 LTS, newest).
> **Sweet spot = Jazzy + Harmonic** (both LTS to 2029, binary‑paired, native Jetson route via JetPack 7.2).

### 3.2 ROS 2 packages (apt `ros-jazzy-<name>` unless noted) — all Jazzy‑release‑verified

**Fusion backends (the showcase + baselines)**

| Package | Role | Status |
|---|---|---|
| *(your code)* `fusion_core` | **Hand‑rolled EKF/UKF** — the showcase, zero ROS deps, pytest‑tested | — |
| **`gtsam`** | Factor‑graph backend (iSAM2) — the "production‑grade" comparison | **apt binary, 4.2.0‑4 verified** |
| **`robot_localization`** | Simple EKF/UKF (`ekf_node`,`ukf_node`) + `navsat_transform_node` (GPS→local) | **apt binary, 3.8.3 verified** |

**Odometry frontends (pluggable — this is the "both" architecture)**

| Package | Frontend type | Status / note |
|---|---|---|
| **`kiss_icp`** (PRBonn) | **LiDAR‑*only* odometry** (point‑to‑point ICP + const‑velocity, ~no params) — *showcase‑preferred lidar frontend* (see note) | source/pip (`pip install kiss-icp`; ROS 2 wrapper in‑repo, easy build); not in rosdistro. 2.2k★, active |
| **`rko_lio`** (PRBonn) | **LiDAR‑*inertial* odometry** — robust apt alternative (fuses IMU internally) | **apt binary, 0.3.0 verified**, active (Jun 2026) |
| **`mola_lidar_odometry`** (MOLA) | LiDAR odometry alt (modular, well‑maintained) | **apt binary, 2.2.1 verified**, active (Jun 2026) |
| `kinematic-icp` (PRBonn) | KISS‑style lidar odometry with a **wheeled‑robot kinematic** motion prior | source; use if the sim platform is a ground vehicle |
| **OpenVINS** | **Visual‑inertial *odometry*** (MSCKF, no loop closure → clean to fuse) — *primary visual frontend* | source; native ROS 2, emits odometry. **⚠️ Build `master` ONLY** — Ubuntu 24.04's Ceres 2.2 broke older tags; the fix (PR #520) is on `master`, **not** any `develop_vX`/release tag (§17.1). |
| **Basalt** | Stereo‑inertial VIO — active alternative | source; active (Mar 2026); ROS integration more DIY |
| Kimera‑VIO | Stereo‑VIO + mesh — alternative | source; active‑ish (Mar 2025); community ROS 2 |
| ORB‑SLAM3 / VINS‑Fusion / ‑Mono | Famous full‑SLAM / VIO — **reference & comparison, NOT the build‑critical frontend** | no *official* ROS 2 (community wrappers last touched 2024); Pangolin/OpenCV build‑hell on 24.04; loop closure ≠ clean odometry. VINS‑Fusion's built‑in GPS fusion = the ancestor of this project's story |
| GLIM / Point‑LIO / FAST‑LIO2 | Heavier LiDAR(‑visual)‑inertial alternatives | source build; optional upgrades |

> **Why lidar‑*only* (KISS‑ICP) is preferred over lidar‑*inertial* (`rko_lio`) for *this* project:** the point is to show off **your** fusion. A lidar‑inertial package fuses the IMU *internally* and hands you an already‑fused pose — so the interesting fusion happens inside someone else's code, and using the IMU again in your filter **double‑counts** it (correlated measurements). KISS‑ICP gives a lidar‑only pose, so **your `fusion_core` does the real work** fusing {lidar odometry + raw IMU + GNSS}. Bonus: KISS‑ICP's known **degeneracy** in featureless scenes (open Arctic flats, tunnels) becomes a *demonstration* — show it drift, then show your IMU‑aided fusion hold (money chart). Keep `rko_lio` (apt) as the robust drop‑in when you don't want to own degeneracy handling.

> **Decision (confirmed): KISS‑ICP primary, `rko_lio` apt fallback.** Added reasoning on the IMU/deskew worry: KISS‑ICP needs **no IMU even for deskewing** — it motion‑compensates each scan with a **constant‑velocity model** estimated from its own prior poses (verified against the KISS‑ICP paper). That's good enough because a 10–20 Hz sweep spans 0.05–0.1 s, over which velocity barely deviates — so the IMU is genuinely **free for `fusion_core`** (high‑rate prediction between updates, GPS‑dropout survival, degeneracy backup) with **no double‑counting**. At Husky speeds on mild–moderate terrain this costs nothing in odometry quality. **Sim caveat (§17.2):** Gazebo `gpu_lidar` has no per‑point time, so run KISS‑ICP **deskew‑off** (or synthesize per‑point time) in the Gazebo phase — small error at moderate speed; real datasets (Ouster time+ring / Livox per‑point time) deskew properly. **Where we expect KISS‑ICP to break — and how we provoke & measure each failure in sim against Gazebo ground truth — is catalogued in `kiss-icp-failure-modes.md`** (geometric degeneracy is the headline → the M4 chart). `rko_lio` (LIO, apt) is the drop‑in if a run's motion proves too aggressive to own.

> **On the visual side (ORB‑SLAM3 / VINS vs OpenVINS):** there is **no robust visual‑*only* analog of KISS‑ICP** — mono/stereo VO drifts on low texture / blur / scale, which is *why* tight‑coupled VIO exists. So the visual frontend is a **VIO black box**; the rule to avoid IMU **double‑counting** is: fuse the VIO's *pose output* + GNSS at the top level, and **don't re‑feed its IMU**. Choose the black box on **ROS 2 build‑ability + output type + maintenance**, not fame: ORB‑SLAM3 & VINS are *full SLAM with loop closure* (retroactive trajectory jumps complicate downstream fusion), have **no official ROS 2** (community wrappers, last 2024) and Pangolin/OpenCV build‑hell on Ubuntu 24.04 → use them as **reference/comparison**. **OpenVINS** is maintained, native‑ROS 2, and emits *odometry* (no loop‑closure surprises) → primary. **Basalt / Kimera** are fine active alternatives if OpenVINS gives trouble. *(These differ in license — several VIO packages are GPL‑3.0 — but per current scope that is **not** a selection factor; revisit only if any of this code heads toward a product.)*

**Perception, sensors, plumbing**

| Package | Role | Status |
|---|---|---|
| `ros_gz` (`_sim`,`_bridge`,`_image`) | Gazebo↔ROS bridge | apt binary 1.0.23 |
| **`depthai-ros`** | OAK‑D driver (stereo/depth/RGB/IMU topics) — *real device* | **apt binary: v2 `depthai-ros` 2.12.2 *and* v3 `depthai_ros_v3` 3.2.1 verified** |
| `vision_msgs` | Detection/tracking message types | apt 4.1.1 |
| `message_filters` | `ApproximateTimeSynchronizer` | apt 4.11.17 |
| `imu_tools` | complementary/Madgwick filters (for the 6‑axis OAK IMU) | apt 2.1.5 |
| `perception_pcl`, `octomap` | point‑cloud / mapping utilities | apt 2.6.4 / 1.10.0 |
| `tf2_ros`, `image_transport`, `image_pipeline` | frames, image plumbing, rectification | apt |
| `rviz2`, `rosbag2` | viz + record/replay (mcap default in Jazzy) | apt |
| `rtabmap_ros` | **RGB‑D mapping / OAK‑D fallback** (NOT the spine — it's not tight VIO) | apt 0.22.1; alive, has OAK integration |

**Real‑device VIO (OAK‑D, later):** **Spectacular AI SDK** — native OAK‑D VIO, **explicitly supports "OAK‑D Lite (with an IMU)"**. Free for non‑commercial; ROS 2 example targets **Humble** (expect Jazzy porting glue). Binary SDK, not apt.

**Python (pip, in‑container):** `numpy`, `scipy` (Hungarian via `linear_sum_assignment`), `opencv-python`, `ultralytics` (YOLO), `matplotlib`, `evo` (ATE/RPE), `rosbags` (Ternaris — convert ROS1 bags ↔ rosbag2 without a ROS install).

> **GPU/PyTorch pin (host driver 535 / CUDA 12.2):** use **`cu121`** PyTorch wheels. `cu124`+ can demand driver ≥ 550 and fail on 535. Ultralytics runs on CPU too (fallback).

### 3.3 Decision points & pivot options — no single point of failure

Built from experience: software gets abandoned, or won't build on *your* box, and you're forced to pivot mid-project. So **every critical layer here has a ranked fallback ladder**, and the whole thing rests on a floor that can't be taken away.

**THE FLOOR (the thing nothing can wall you on):** because the source layer is decoupled (§5), the always‑works path at *every* layer is **replay a recorded dataset and use its provided data.** KITTI ships continuous GPS/INS ground‑truth poses **and** object labels; Boreas ships GNSS/INS poses **and** 3D boxes. So even if every simulator, every odometry package, and every detector refuses to build on your machine, you can still demonstrate the **fusion filter**, the **GPS‑dropout experiment**, and **object tracking** from a bag + a CSV of poses/labels. The project survives the loss of literally everything else below.

**Verified apt‑availability (rosdistro)** — `✓` = apt binary on that distro, `src` = source build, `–` = absent:

| Pivot ladder | A — Primary | B — Safe apt fallback (portable) | C — Source upgrade (more capable, more fragile) | Eject / floor | Pivot trigger |
|---|---|---|---|---|---|
| **ROS 2 distro** | **Jazzy** ✓ (LTS, Noble) | **Humble** ✓ — *most mature; 22.04‑native; ALL key pkgs below exist on it (only `fuse` missing); Gazebo pairing = Fortress* | Kilted ✓ (newer, EOL Nov 2026) | — | a key pkg won't build on Jazzy, or you want JetPack 6.x‑native → **move the whole stack to Humble** |
| **Simulator** | **Gazebo Harmonic** ✓ | **Webots** (`webots_ros2` ✓ on Humble/Jazzy/Kilted) — *simpler rendering, far fewer Docker‑GL headaches* | Gazebo Fortress ✓ (older, rock‑stable, Humble‑paired) | **No sim — dataset replay only** | ogre2/GL won't render in Docker after §7.2 fixes → Webots; all sim fails → dataset‑only (lose closed‑loop, keep everything else) |
| **Sim container base** | osrf/ros `desktop-full` ✓ | `ros:jazzy-ros-base` + `apt install ros-jazzy-ros-gz` | — | Gazebo on **host** (skip the container for sim) | image won't pull / variant shifts |
| **Lidar odometry frontend** | **KISS‑ICP** (src/pip, lidar‑*only* → your filter does the IMU+GNSS fusion; the better showcase) | **`rko_lio`** ✓ / **`mola_lidar_odometry`** ✓ (all distros, lidar‑*inertial*, robust, apt) | FAST‑LIO2 / GLIM / Point‑LIO (src); `kinematic-icp` (wheeled) | `slam_toolbox`/`cartographer_ros` ✓ → or **dataset poses** | KISS‑ICP degenerates in featureless scenes or won't build → `rko_lio` apt → dataset |
| **Visual‑inertial frontend** | **OpenVINS** (src, ROS 2, *odometry*) | **`rtabmap_ros`** ✓ (all distros — visual/RGB‑D odometry; can't‑fail apt visual path) | **Basalt** / Kimera (active VIO); ORB‑SLAM3 / VINS (ref/compare only) | dataset‑provided VO, or Spectacular AI on the real OAK | OpenVINS won't build → **rtabmap apt** → Basalt |
| **Fusion backend** | **Hand‑rolled EKF/UKF** (pure Python, zero external deps — *cannot* be deprecated) | **`robot_localization`** ✓ + **`gtsam`** ✓ (all distros) | `fuse` ✓ (Jazzy/Kilted only); FilterPy (pip) | hand‑rolled (it's the floor *and* the showcase) | a baseline lib breaks → drop it; the showcase never depends on it |
| **Object detector** | **Ultralytics YOLO** (pip) | torchvision detectors (pip, fewer deps) → CPU inference | MMDetection / YOLOX (src) | **dataset‑provided 2D/3D labels** (KITTI/Boreas ship boxes — zero inference) | torch/GPU pain → CPU → dataset labels |
| **OAK‑D software** (real device, later) | **`depthai_ros_v3`** ✓ | **`depthai-ros`** v2 ✓ | Spectacular AI SDK (VIO, binary) | rtabmap + depthai (apt) | v3 API churn → v2; need VIO → Spectacular AI |
| **RMW / DDS** | FastDDS (default) | **CycloneDDS** ✓ (`rmw_cyclonedds_cpp`) | — | — | cross‑container discovery flaky → Cyclone |
| **Edge ML (Jetson)** | TensorRT | **ONNX Runtime** (portable, version‑tolerant) | — | plain PyTorch on device | TensorRT engine/version pain → ONNX Runtime |
| **Bag conversion** | `rosbags` (pure Python) | native KITTI converters | — | `kitti2bag`→`rosbags-convert` | one converter chokes on a sequence → try the next |
| **Datasets** | MARS‑LVIG + NTU VIRAL (aerial real) | TartanAir V2 (sim aerial+snow) + EuRoC (baseline) | Boreas/CADC (real snow); KITTI (baseline) | any one with poses (+GNSS for the dropout test) | a host goes down / gated download |

**How to use this:** pick column A and go. The moment A walls you, drop to **B (apt, portable)** — that keeps you on binaries that exist across distros, so you don't also trigger a distro pivot. C is only for when you *want* more capability and can afford build fragility. The eject column is the "ship something anyway" path. The two design choices that make this work are deliberate: the **fusion core is hand‑rolled** (no external lib can deprecate your showcase) and the **source layer is decoupled** (dataset replay is always available).

**The big pivot, pre‑validated:** if Jazzy/Harmonic itself becomes the wall, the entire stack drops to **Humble + Fortress** with almost no redesign — `rko_lio`, `mola`, `slam_toolbox`, `cartographer`, `rtabmap`, `robot_localization`, `gtsam`, `depthai‑ros`, `webots_ros2` are **all apt on Humble too** (verified). You'd lose `fuse` (a non‑critical baseline) and swap Harmonic→Fortress in the bridge. Keep the code distro‑agnostic (standard ROS 2 APIs, no Jazzy‑only features) and this pivot stays cheap.

---

## 4. Hardware reality check (this machine, measured)

| Resource | Measured | Verdict |
|---|---|---|
| OS | Ubuntu 22.04.5 (Jammy) | OK — everything in Docker (Jazzy = Noble inside). |
| CPU | i7‑8750H, 6c/12t | OK — and it matters more now: lidar odometry + OpenVINS are **CPU‑bound**, not GPU. 6 cores is enough for one frontend at a time; running lidar+visual+tracker+sim simultaneously will be tight → stagger or replay. |
| RAM | 31 GiB (~22 free) | Comfortable. |
| **GPU** | **RTX 2070 Mobile, 8 GB** (Turing CC 7.5); + Intel UHD 630 (Optimus) | **Enough** — Gazebo render + small YOLO; lidar/VIO barely touch it. §10. |
| Driver/CUDA | 535.309.01 / CUDA 12.2 | Pin PyTorch `cu121`. |
| Disk | 467 GB, **146 GB free** | Fine (budget §9). |
| Docker | 29.6.0, **`nvidia` runtime configured** | Ready (one gotcha §7.2). |

---

## 5. Architecture

Decoupled at ROS 2 topic boundaries: **source** (dataset/sim/live), **odometry frontends** (swappable), **fusion core** (the showcase), **output**.

```
 SOURCE LAYER            ODOMETRY FRONTENDS            FUSION CORE                 OUTPUT
 (swappable)             (pluggable, swappable)        (the showcase)              (explainable)

 rosbag2 replay ─┐    ┌─ lidar odometry ──┐
  MARS‑LVIG      │    │  (KISS‑ICP,       │ /odom_lidar ─┐
  NTU VIRAL      ├──► │   lidar‑only)     │              │
  TartanAir/EuRoC│    └───────────────────┘              ├─► ┌────────────────────┐
                 │    ┌─ visual‑inertial ─┐ /odom_visual │   │  ego_localizer     │ /pose+cov ─► RViz
 Gazebo Harmonic ├──► │  (OpenVINS)        │             ┤   │  EKF/UKF  ⟷  GTSAM  │   + evo logs
  (lidar+IMU+    │    └───────────────────┘              │   │  + GNSS + dropout  │
   GNSS+OAK‑D)   │    /imu  /gnss(navsat) ───────────────┘   │  + sensor‑health   │
 live OAK‑D ─────┘                                           │    switching       │
  (real, later)                                              └────────────────────┘
                      camera ─► YOLO detector ─► /detections ─► ┌──────────────────┐
                                                                │ object_tracker   │ /tracks+ellipses ─► RViz
                                                                │ EKF + gating +   │                   + MOT metrics
                                                                │ Hungarian        │
                                                  shared fusion_core library ◄──────┘
```

**Two ROS modules, one shared filter library.** `ego_localizer` and `object_tracker` both wrap `fusion_core` (different state vectors / measurement models). **Don't merge them** — the separation is the architectural point.

- **Odometry adapter interface:** each frontend publishes a standard `nav_msgs/Odometry` (+ covariance) on its own topic. `ego_localizer` subscribes to whichever are configured. Adding a frontend = a launch‑file/param change, **not** a code change. This is the "sensor‑agnostic" claim made concrete.
- `ego_localizer` state = pose+velocity(+IMU bias). Measurements: IMU (predict); lidar‑ and/or visual‑odometry (relative); **wheel odometry (relative, *toggleable* — easy add/remove via config; §16.11)**; GNSS/NavSat (absolute, *droppable*, conditioned by the `gps_conditioner`; §11). EKF first; UKF + GTSAM as comparison branches.
- `object_tracker`: per‑track position+velocity; detections from YOLO; Mahalanobis gating + Hungarian; track lifecycle (birth/confirm/coast/die).

**Frames (TF):** `map → odom → base_link → {imu, lidar, oak_*, gnss}`. GNSS gives `map→odom`; odometry gives `odom→base_link` (standard ROS layering; matches `robot_localization`).

### 5.1 Simulated sensors (Gazebo Harmonic)
| Sensor | Gazebo system | Notes |
|---|---|---|
| 3D LiDAR | `gpu_lidar` | feeds the lidar frontend. ⚠️ gz cloud has xyz/intensity/**ring** but **no per‑point time** — synthesize it or run deskew‑off (§17.2). Needs Sensors system + GPU render. |
| Vehicle IMU | `imu` | the "good" IMU for the spine (~200 Hz, low bias) |
| **Magnetometer** | `magnetometer` (→`MagneticField`) | **heading source** so `navsat_transform` can orient GPS — without it a 6‑axis IMU leaves the GPS track rotating/drifting (§17.4) |
| GNSS | `navsat` | absolute pose; the *droppable* input. ⚠️ **requires `<spherical_coordinates>` + the NavSat system plugin in the world** or it emits nothing (§17.2) |
| **OAK‑D‑Lite‑equivalent bundle** | 2× `camera` (mono, 640×480, **75 mm baseline**) + 1× `camera` (RGB) + `depth_camera` + `imu` (**6‑axis, ≤250 Hz**) | mimics your real device — see §8. ⚠️ set camera `frame_id` to a REP‑103 **optical** frame (bridge `override_frame_id`) |

IMU, magnetometer and NavSat are **CPU‑side, non‑rendering**; only cameras + lidar hit the GPU.

---

## 6. Repository / package layout

```
sensing-node/                          # public repo
├── README.md                          # tutorial + money charts
├── docker/
│   ├── Dockerfile.sim                 # amd64: osrf/ros:jazzy-desktop-full (+ explicit ros-gz)
│   ├── Dockerfile.fusion              # multi-arch app: base ros:jazzy-perception
│   ├── compose.yaml                   # services: sim, lidar_odom, visual_odom, fusion, tracker, rviz, bag
│   └── entrypoints/
├── ros2_ws/src/
│   ├── sensing_bringup/               # launch, params, RViz cfg, SDF worlds (lidar+imu+navsat+oak)
│   ├── fusion_core/                   # shared filter LIBRARY (EKF/UKF + GTSAM variant), no ROS deps, pytest
│   ├── ego_localizer/                 # ROS node wrapping fusion_core (subscribes configured odom frontends)
│   ├── odometry_adapters/             # thin wrappers normalizing rko_lio / OpenVINS → std Odometry+cov
│   ├── object_tracker/               # detections -> multi-object EKF
│   ├── perception_yolo/               # ultralytics detector -> vision_msgs
│   ├── dataset_publishers/            # stream-publish adapters: native format -> live ROS2 topics (§9.1)
│   ├── oak_sim/                       # OAK-D-Lite-equivalent xacro + Gazebo sensor plugins
│   └── eval_tools/                    # evo wrappers, GPS-mask injector, chart generators
├── real_device/                       # Spectacular AI / depthai-ros configs for the physical OAK-D (later)
├── datasets/  (gitignored)            # HARD-CAPPED 60 GB, auto-pruned (§9.1)
├── results/                           # (committed) traj.tum + metrics.csv + chart.png per seq — the deliverables
├── scripts/                           # fetch_seq.sh (capped) · run_seq.sh · prune.sh (§9.1)
└── docs/                              # writeup, diagrams, benchmark CSVs
```

`fusion_core` has **zero ROS deps** → unit‑testable, reusable, the 10‑minute read that proves your filter skills.

---

## 7. Docker design (gotchas that would otherwise sink a weekend)

### 7.1 Images
| Image | Base | Arch | Purpose |
|---|---|---|---|
| **sim** | `osrf/ros:jazzy-desktop-full` (Gazebo Harmonic + RViz) | **amd64 only** (verified) | simulator + GUI on the laptop |
| **fusion** | `ros:jazzy-perception` (**multi‑arch amd64+arm64**, verified) + app | amd64 now, arm64 later | the portable node → Jetson |

`desktop-full` really includes **modern** Gazebo (not EOL Classic): verified via **REP 2001** (Jazzy `simulation` variant = `ros_gz_*`, pulled in by `desktop_full`). **Robustness:** in `Dockerfile.sim` still `apt-get install -y ros-jazzy-ros-gz` and check `gz sim --versions` at build.

### 7.2 GPU in Docker — the #1 gotcha (bold this in the tutorial)
NVIDIA Container Toolkit defaults `NVIDIA_DRIVER_CAPABILITIES=compute,utility` → **no OpenGL/GLX** → Gazebo `ogre2` dies with *"Unable to create the rendering window."*
```yaml
environment:
  - NVIDIA_DRIVER_CAPABILITIES=all        # or at least graphics,compute,utility
  - NVIDIA_VISIBLE_DEVICES=all
deploy: { resources: { reservations: { devices: [{ driver: nvidia, count: all, capabilities: [gpu] }] } } }
# GUI: xhost +local:docker ; mount /tmp/.X11-unix ; pass DISPLAY
```
**Optimus laptop:** force NVIDIA (`prime-select nvidia` or `__NV_PRIME_RENDER_OFFLOAD=1`,`__GLX_VENDOR_LIBRARY_NAME=nvidia`). The `libEGL warning: DRI2…` line is harmless.
**Headless (CI/no‑X):** `gz sim -s -r --headless-rendering world.sdf` (EGL offscreen) for sensor data.

### 7.3 DDS across containers — the #2 gotcha
- **Simplest:** `network_mode: host` on every ROS service + shared `ROS_DOMAIN_ID`.
- **Robust fallback:** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + Cyclone XML unicast peers (no multicast).

### 7.4 First thing to build
`compose up` → 1 rosbag → pass‑through node → RViz. Prove containers/GPU/DDS/bag replay **before** any filter math. (Milestone 1.)

---

## 8. The OAK‑D Lite — candid reality + sim‑vs‑real (so future‑you doesn't get stuck)

> This is the *design summary*. The full field guide — measured IMU characteristics,
> camera↔IMU sync, the depthai 3.7.1 mono-camera crash investigation, version guide,
> troubleshooting playbook — is [`oak-d-lite-guide.md`](oak-d-lite-guide.md).

Your unit is confirmed to have an IMU; research says that IMU is almost certainly a **Bosch BMI270**.

**Facts [F]:**
- **BMI270 = 6‑axis (accel+gyro), ≤250 Hz, no magnetometer, no on‑chip fusion.** (Confirm on the unit: `device.getConnectedIMU()` → `"BMI270"`.) It's the **weakest‑for‑VIO** IMU in the OAK family; OAK‑D Pro/S2 use a 9‑axis BNO086.
- **No hardware camera↔IMU sync.** Same device clock, but a documented **~10 ms residual offset** (depthai‑core issue #599). The usual mitigation "run the IMU faster" is itself **capped at 250 Hz** on this chip.
- **Spectacular AI SDK explicitly supports "OAK‑D Lite (with an IMU)"** for VIO — so real‑device VIO is achievable, just Lite‑grade. Its ROS 2 example targets **Humble** (Jazzy = porting glue).
- **depthai‑ros**: Jazzy binaries exist for **both** v2 (`depthai-ros` 2.12.2) and **v3** (`depthai_ros_v3` 3.2.1). The in‑wrapper VSLAM was disabled in the v2 line — **don't depend on the wrapper's built‑in SLAM**; use it for raw stereo/depth/RGB/IMU topics.

**Design conclusion [I]:** **Do not make the BMI270 your sole inertial source for tight VIO.** Use the OAK‑D Lite as a **stereo‑depth + RGB + object‑detection** sensor; get ego‑motion from the lidar‑inertial spine and/or Spectacular AI, fused with GNSS/INS. This is *more* realistic (matches multi‑sensor fielded stacks), not a compromise.

**OAK‑D Lite specs for the sim model [F]:** 2× OV7251 mono **640×480 global‑shutter** (FOV ~73° H), 1× IMX214 **13 MP RGB rolling‑shutter** (~69° H), **75 mm baseline** (the hardware page's "75 cm" is a unit typo; depthai xacro uses 0.075 m), depth ~0.2 m–12 m.

**Sim‑vs‑real difference to bake in [F/I]:** depthai ships an OAK‑D‑Lite **URDF + mesh but no Gazebo sensor plugins** — you compose them (§5.1). On the **real** device, depth is computed on the OAK's VPU (SGBM → holes, quantization, min‑distance dead zone); in **sim**, Gazebo's `depth_camera` gives near‑perfect ground‑truth depth. **Your pipeline will look better in sim than on hardware** unless you (a) **inject depth noise/dropout**, or (b) compute disparity in sim from the two mono images instead of using Gazebo's clean depth, and (c) **inject an IMU timestamp offset** to mimic the ~10 ms.

---

## 9. Datasets

> **Decision: UGV/ground‑first, NOT aerial.** The goal is a simple ground UGV (Husky in sim), so the spine is the **Gazebo Husky sim + recognizable ground baselines (KITTI / EuRoC) + a ground GPS‑denial set** (KITTI OXTS, or Boreas automotive GNSS/INS). The aerial sets below were picked only because they're *harder than KITTI* — they're retained as **optional stress tests**, not the project's thesis. The earlier "aerial‑first / real aerial‑Arctic data doesn't exist" framing is **dropped**.

Tiered by role. "Mask GPS → measure drift" needs **continuous GNSS‑derived** truth — on the ground that's **KITTI's 10 Hz OXTS** or the **automotive‑winter** sets; the aerial‑GNSS sets are the optional harder version.

| Tier / dataset | Platform · year | Sensors / GT | Role here | GPS to mask? | License · format |
|---|---|---|---|---|---|
| **Sim‑first dev — TartanAir V2** ⭐ | **SIM aerial** (UE+AirSim) · ~2024 | stereo RGB + perfect depth + sim IMU + lidar; **snow/fog/night/seasons**; sim‑perfect pose | **the aerial+snow dev set** — "more realistic than a hand‑built Gazebo world"; tune odometry/perception in conditions that don't exist in real aerial data | No (sim pose) | CC BY 4.0 · `pip install tartanair`, folders+poses (no rosbag → write a publisher) |
| **Aerial GPS‑denied keystone — MARS‑LVIG** ⭐ | **UAV** (DJI M300 RTK, 80–130 m) · 2024 | **Livox Avia lidar (per‑point time, `line` idx)** + **mono** 2448×2048 GS cam + BMI088 **@200 Hz**; **real continuous RTK‑GNSS GT @5 Hz** | **the genuine drone GPS‑denial drift experiment** (only set that's aerial + real GNSS + per‑point lidar time) | **Yes — real RTK** | CC BY‑NC‑SA 4.0 · ROS1 bag (needs Livox driver) → convert |
| **Aerial lidar companion — NTU VIRAL** | **UAV** (DJI M600) · 2022 | **2× Ouster OS1‑16 (native t+ring)** + stereo GS + VN100 **@385 Hz** + UWB; Leica total‑station GT (no GNSS) | stereo + **ring‑based deskew** coverage MARS‑LVIG's mono/Livox lacks; clean first real‑lidar | No (total station) | CC BY‑NC‑SA 4.0 · ROS1‑only → convert |
| **Real cold proxy — Boreas / Boreas‑RT / CADC** | **automotive** · 2022–26 | lidar (per‑point t+ring) + cam + **GNSS/INS** (cm); **real falling snow** | snow‑robustness on **real** data (aerial‑snow is sim‑only); on‑theme Canadian winter; 3D labels (Boreas) | **Yes** | Boreas CC BY 4.0 (no bags→`pyboreas`); CADC CC BY‑NC |
| **Baselines — EuRoC / KITTI** | MAV indoor / car · 2016 / 2013 | EuRoC stereo+IMU@200 Hz; KITTI stereo+velodyne+10 Hz OXTS | **report numbers everyone recognizes** + EuRoC = lowest‑friction VIO bring‑up (OpenVINS ships `config/euroc_mav/`). **NOT build targets** | EuRoC No / KITTI yes (10 Hz IMU, no per‑point time) | EuRoC CC BY 3.0 / KITTI CC BY‑NC‑SA |

**~~The narrative this unlocks (put it in the writeup)~~ [SUPERSEDED — kept for reference only]:** the old aerial framing argued that real *aerial + Arctic + lidar + GNSS* data **does not exist** — every real winter‑lidar set is a car — so the honest path for cold‑weather *aerial* perception is sim‑first (TartanAir + Gazebo), validated against real aerial GNSS (MARS‑LVIG) and real snow (Boreas/CADC) separately. **This is no longer the thesis** (the project is ground‑UGV‑first; see the §9 decision note and §16.9). The aerial sets remain only as optional harder‑than‑KITTI stress tests.

**On KITTI/EuRoC (still defensible as baselines):** 2025–26 papers still report KITTI odometry + EuRoC VIO tables — they're recognizable yardsticks, just not the frontier. Show their numbers; build on the modern aerial sets. KITTI's specific limits (10 Hz synced IMU, no per‑point lidar time/ring — verified in KISS‑ICP's loaders) are *why* it's a baseline, not a build target.

**Custom snow scenes (beyond Gazebo):** **Cosys‑AirSim** — the *maintained* AirSim successor (Microsoft `AirSim` is **dead since Jul 2022**; don't use it); UE5, ROS 2, GPU lidar — or **NVIDIA Isaac Sim** (RTX lidar, drone‑capable). Optional, only if Gazebo + TartanAir aren't enough.

**Ingestion (see §9.1 — stream‑publish by default, don't archive):** publisher adapters read native formats live, so TartanAir / Boreas / MARS‑LVIG / KITTI need **no rosbag2 archive at all**. Only the small reused sets (EuRoC, NTU VIRAL) are *optionally* cached as rosbag2 via `rosbags` (pure‑Python; output to a **folder**; hand‑fix `metadata.yaml` QoS if `bag play` errors). MARS‑LVIG's adapter handles the Livox `CustomMsg` and emits standard `PointCloud2`.

**Minimum to start (~5 GB):** 1 EuRoC seq (VIO bring‑up) + a few **TartanAir** aerial seqs (sim dev). Add **MARS‑LVIG** (real aerial GPS‑denial), **NTU VIRAL** (ring‑based lidar), **Boreas/CADC** (real snow) as each milestone needs them.

⚠️ **Caveats:** MARS‑LVIG is **monocular** (mono‑VIO) and uses Livox **`line`** not conventional `ring` (deskew needs Livox handling) — NTU VIRAL covers the stereo+ring gap. TartanAir GT is **sim‑perfect, never a real‑GNSS drift benchmark**. MARS‑LVIG/NTU VIRAL aggregate sizes unverified; TartanAir V1 ~4 TB total → **download select environments only**. Both aerial sets are **CC BY‑NC‑SA** (non‑commercial — fine for this project; not a selection blocker per current scope).

### 9.1 Disk‑frugal data strategy — streaming, capped at 60 GB, auto‑prune

**Decision (confirmed):** datasets live on the internal disk inside a **hard‑capped, self‑pruning `datasets/` dir (ceiling 60 GB)**. Principle: **data passes through, it does not accumulate.** This is what keeps the project from creeping toward filling the disk regardless of how many sequences you touch over time.

1. **Stream‑publish, don't archive.** A thin ROS 2 **publisher adapter** per dataset reads the *native* format (EuRoC folders, KITTI bins, `pyboreas`, TartanAir folders, MARS‑LVIG ROS1 bag) and publishes live ROS 2 topics. This **eliminates the ROS1‑bag + rosbag2 double‑copy** (you never store a converted archive), and it *is* the swappable "source layer" the architecture already wants (§5). For MARS‑LVIG the adapter reads the Livox stream and emits standard `PointCloud2` directly — no intermediate file. Trade‑off: re‑reads the source each run (fine — disk beats CPU here). **Exception:** the small, heavily‑reused sets (EuRoC ~1–2 GB, NTU VIRAL ~2 GB) *may* be cached as rosbag2 for fast replay — they fit the cap trivially.
2. **One sequence at a time: fetch → run → keep results → delete raw.** Deliverables are trajectories (TUM/CSV, KB), metrics, charts (MB) — those persist in `results/`; the multi‑GB inputs don't.
3. **Tooling enforces the cap (it can't run away):**
   - `scripts/fetch_seq.sh <dataset> <seq>` — **refuses if `datasets/` would exceed 60 GB**; uses the *selective* downloaders (TartanAir `modality=['image','lidar','imu']`; Boreas per‑sensor `lidar/`+`applanix/`; MARS‑LVIG per‑file).
   - `scripts/run_seq.sh <dataset> <seq>` — publisher adapter → pipeline → writes `results/<dataset>/<seq>/{traj.tum, metrics.csv, chart.png}`.
   - `scripts/prune.sh` — deletes raw seqs that already have results, keeping `datasets/` under cap (auto‑called at the end of `run_seq.sh`).
4. **Results‑only git.** `.gitignore` covers `datasets/` + any raw/converted bags; commit only `results/` + configs + code → repo stays <100 MB.

**Footprint in practice:** Docker (~15 GB) + small working set (EuRoC + NTU VIRAL + a few TartanAir envs ≈ 8–10 GB) + at most **one** big seq transient (Boreas ~50 GB) → peaks near the 60 GB dataset cap *only* while a Boreas seq is resident, then `prune.sh` drops it back to <10 GB. You never approach the 174 GB free.

---

## 10. Disk + GPU budgets

**Disk — verified sizes (byte‑exact where noted). You have 146 GB free → the project fits *if you never pull a full set*.**
| Item | Smallest usable | Working set | Full (NEVER pull) |
|---|---|---|---|
| **Docker images** | base `ros:jazzy-perception` **3.46 GB** (measured) | sim ~4 + fusion ~10 + cache → **~12–15 GB** | — |
| EuRoC (VIO bring‑up) | ~1 GB/seq | ~18 GB (all) | 25 GB |
| NTU VIRAL (aerial lidar) | 2.0 GB/seq | ~12 GB (3) | 35 GB |
| TartanAir V2 (sim, **`modality=['image','lidar','imu']`**) | ~0.5–0.9 GB/env | ~40 GB (6 aerial envs) | **57 TB** ⚠️ |
| MARS‑LVIG (GPS‑denied keystone) | ~10–15 GB/seq ⚠️*est, unpublished* | ~50–75 GB (few) | ~0.5 TB |
| Boreas (real snow, **lidar+GPS only**) | **~50 GB/seq** (the budget‑buster) | 100–150 GB (2–3) | 4.4 TB |
| CADC (real snow, alt) | 0.65 GB/drive | ~4 GB (3) | 96 GB labeled |
| KITTI (baseline) | 0.46 GB/drive | — | 85 GB velodyne |
| Build artifacts + result bags + charts | — | +5–10 GB | — |

**Disk‑frugal strategy (see §9.1) makes the footprint bounded, not cumulative:** datasets stream through a **hard‑capped 60 GB `datasets/`** dir, one sequence at a time, auto‑pruned after results are saved. So steady‑state is **Docker (~15 GB) + a small working set (~8–10 GB) + ≤1 big transient seq**, peaking near the 60 GB cap only while a Boreas seq is resident, then back to <10 GB. With **174 GB free** this never gets close to tight.

**Two silent disk‑doublers — both now designed out:** (1) the ROS1→rosbag2 double‑copy is gone because adapters **stream‑publish** (no archive, §9.1); (2) TartanAir's default pulls *all* modalities (→57 TB) — `fetch_seq.sh` always passes the modality filter. Gitignore `datasets/`; commit only `results/`.

**"Will my GPU be enough?" — Yes.**
- Gazebo Harmonic (moderate world, camera+lidar+imu+navsat): light for an 8 GB Turing.
- **Lidar‑inertial (`rko_lio`) and OpenVINS are CPU‑bound** — they barely touch the GPU. The real constraint is your **6 CPU cores**, not VRAM: don't run lidar+visual+tracker+sim all live at once — **stagger them or replay from bag**.
- YOLO: `yolov8n/s`, ~1–2 GB VRAM.
- **Caveat:** don't co‑locate heavy GPU loads (big world render + `yolov8x`); use small models or run tracking on recorded data. No training on 8 GB — inference only (on‑message for an "edge" project).

---

## 11. GPS as a tunable input — the `gps_conditioner` (honest)
Gazebo has **no built‑in GPS‑dropout toggle** (verified), and we want more than on/off — we want to **vary GPS to test scenarios** (reduce rate, add noise/bias, cut over a window). So GPS is treated as a **knob, not a fixed input**: a dedicated **`gps_conditioner` node** sits between the GPS source and `navsat_transform`/the global filter and republishes a conditioned `NavSatFix`. It works **identically for the Gazebo NavSat and for dataset GPS** (the §5 source‑decoupling), so the rest of the stack is unaware.

> **Knobs (scoped):** update‑rate decimation (10→1→0.2 Hz) · hard cut + timed **dropout windows** `[t_start,t_end]` · covariance inflation / Gaussian noise — the MVP set; slow **bias/drift** and **denial regions** (cut inside a polygon) are optional. Two rules make it an *instrument*, not a hack: **runtime‑controllable** (ROS 2 params + a service/topic to trigger a dropout live) and **scenario = a file** (a small YAML schedule, e.g. `0–30s @10 Hz, 30–60s @1 Hz, 60–90s off`) so every run is reproducible and chartable — the scenario file *is* the experiment. (Optional stretch: a Gazebo system plugin toggling NavSat `SetActive` for a "physical" denial.)

**Do it as a dual‑EKF (the documented jump‑free pattern):** a **local** EKF fuses only continuous data (odometry + IMU) and publishes `odom→base_link`; a **global** EKF additionally fuses GNSS and publishes `map→odom`. On dropout the local EKF keeps the control‑frame pose smooth; on **reacquisition the jump is absorbed into `map→odom`** (which REP‑105 *allows* to jump), so there's no discontinuity in `odom→base_link`. Mahalanobis‑gate the first re‑fix. **Heading caveat (ties to §8):** a 6‑axis IMU (OAK BMI270, or a basic sim IMU) has **no absolute heading**, so `navsat_transform` can't orient the GPS track — it rotates/drifts. Fix in sim by adding a **magnetometer** (gz Magnetometer→`MagneticField`) or a 9‑axis IMU for the spine; or initialize yaw from **GPS course‑over‑ground** once moving. See §17.4.

---

## 12. Phased milestones (each ends in a commit + something to show)

| # | Milestone | Output | Effort |
|---|---|---|---|
| **1** | **Thin slice**: `compose up` → 1 rosbag → pass‑through → RViz. Proves containers/GPU/DDS/bag. | clip | 1 wk |
| **2** ✅ | **`fusion_core` + pytest**: EKF predict/update + covariance, no ROS. **Done** — `ros2_ws/src/fusion_core/` (generic EKF + CV models, Joseph form, NIS/Mahalanobis). | green tests (**14 passed**, see §19.1) | 1 wk |
| **3** ✅ | **Visual frontend — sim‑first (DONE)**: **OpenVINS** (**stereo**, master `69488123`) runs **live on the Husky sim** (OAK‑D Lite L/R pair + Microstrain IMU, no download), feeding `ego_localizer` (`visual_delta_update`). Over a 20.5 m curved drive vs gz ground truth: **raw stereo VIO ATE 0.069 m / RPE 0.004 m**, **fused ego_localizer ATE 0.077 m / RPE 0.009 m** (rigid‑SE(3) — genuine metric scale; `results/m3_vio.png`, §19.1). **Went stereo because mono is scale‑degenerate on a smooth planar UGV** (no IMU accel excitation → mono won't even initialise; diverged to km). Cleared the §17.1 Ceres‑2.2 + `.h→.hpp` build walls and the camera‑render bug (#8). EuRoC/Vicon comparison split to **M3b**. | sim ATE/RPE plot ✅ | 1–2 wk |
| **3b** | **Visual frontend on a real dataset (deferred from M3)**: `ego_localizer` + **OpenVINS** on **EuRoC**; compare vs `robot_localization` + **Vicon** truth — the recognizable VIO numbers. Dataset/source‑build heavy (EuRoC download + rosbag2); split out so M3 stays laptop‑closable in sim. | ATE/RPE plot (real data) | 1 wk |
| **4** ✅ | **Lidar frontend — sim‑first (DONE)**: **KISS‑ICP** (source build, pinned) live on the Husky's own Ouster, feeding `ego_localizer` through a **relative hook of the identical shape as the visual one** (`lidar_delta_update` — **zero `fusion_core` changes**: the sensor‑agnostic proof). A/B vs the M3 stereo VIO in the *same drive* + a **four‑world terrain sweep** (`results/m4_terrain_sweep.png`): VIO superb wherever texture exists (ATE 0.045–0.097 m) but **diverges on alpine terrain** (measured mid‑route texture starvation) exactly where lidar keeps working — the degeneracy/complementarity chart (#2) delivered in sim. Measured walls + the slow‑UGV under‑report bias: [`m4-lio.md`](m4-lio.md). **The real‑lidar tier (NTU VIRAL: native ring → deskew ON, ROS1→rosbag2 via `rosbags`) is deferred to the M4b/dataset step below** — split out so M4 stayed laptop‑closable, mirroring M3/M3b. | sim A/B + terrain‑sweep chart ✅ | 1–2 wk |
| **4b** | **`dataset_publishers/` stream adapters + capped fetch/run/prune tooling** (§9.1): native‑format → live ROS2 topics for NTU VIRAL, MARS‑LVIG (handles Livox `CustomMsg`→`PointCloud2`), TartanAir, Boreas — **no rosbag2 archive**; `fetch_seq.sh` enforces the 60 GB cap. Unlocks the aerial tiers disk‑frugally. | seq streaming live + cap enforced | ~1 wk |
| **5** | **GPS‑denied keystone** on **MARS‑LVIG** (real aerial **RTK‑GNSS** — the one set that supports this on a drone) for **both** frontends + the dropout gate; **dual‑EKF** so the reacquisition jump lands in `map→odom`, not the control frame (§11). | drift→reacquire chart (#1) | 1 wk |
| **6** ✅ | **GTSAM variant** of the fusion core; A/B vs hand‑rolled EKF. **Done** — `fusion_core/factor_graph.py` (`PlanarFactorGraph`: ISAM2 pose graph behind the exact `PlanarPoseEstimator` interface; relative hooks land as native `BetweenFactorPose2`). A/B on identical streams: **accuracy parity, EKF 3.5× cheaper** on GNSS‑heavy streams (`results/m6_ab.md`) — confirming the §3.2 choice of the hand‑rolled EKF as the live spine. | accuracy/timing table ✅ | 1 wk |
| **7** | **`object_tracker`** on KITTI (cars/peds) or Boreas (3D labels): YOLO → multi‑obj EKF + gating + Hungarian + ellipses. | tracking chart (#3) | 1–2 wk |
| **7b** | **Arctic/snow robustness chapter**: run the pipeline on **TartanAir V2 snow** (sim aerial) + **Boreas/CADC** (real automotive snow — write the `pyboreas`→rosbag2 exporter here). Shows graceful degradation in the conditions off‑road autonomy cares about; encodes the "aerial‑snow only exists in sim" thesis. | snow‑vs‑clear degradation chart | 1–2 wk |
| **8** | **OAK‑D‑equivalent in sim**: compose the sensor bundle (§5.1, §8); run the visual frontend on *its* stereo+IMU; inject depth noise + IMU offset. | sim clip | 1–2 wk |
| **9** | **Gazebo closed‑loop (optional)**: full SDF world, live dropout demo. | sim video | 1–2 wk |
| **10** | **Jetson delta** (when HW available): same `fusion` container on Jetson; latency+power vs laptop. | edge chart (#4) | HW‑gated |
| **11** | **Real OAK‑D (optional, when ready)**: Spectacular AI VIO on the physical Lite; compare to sim. | bonus clip | HW‑gated |
| **12** | **Tutorial writeup**: README + charts + 60 s clip; clone → `compose up`. | the deliverable | 1 wk |

Milestones 1–7 are a complete, defensible piece on the laptop. 8–12 are upgrades. **Don't build all frontends at once** — get one working end‑to‑end (M3), *then* prove the architecture by adding the second (M4).

**Navigation milestones N1–N3 (the three operator control modes) are specified in §18.** They ride on M3/M4/M5/M8 (they consume the fused pose and the Husky sim) rather than forking the roadmap.

**Stressor‑axis milestones S1–S5 ride on the sim the same way.** WildSeed ≥ `c472085` (scenario format 4 — same seed with dials unset still reproduces the old worlds byte‑identically) added seeded stressor dials (`--texture`, `--photometric`, `--weather`, `rig --calib`, `--biome-file`; [`wildseed-worlds.md`](wildseed-worlds.md)). Each isolates **one variable at fixed geometry**, upgrading the M4 terrain sweep from biome‑as‑texture‑proxy to controlled A/B — all laptop‑closable, reusing the existing harness (`prepare_wildseed_world.sh` + `provenance.json` → `deploy.sh world` → `m3-smoke` → `m4_lio_eval.sh` → `plot_m4_sweep.py`) and its measured rules (chart **RPE not ATE** — slow‑UGV bias; FRESH frontend start after steady RTF > 0.4; texture is a *route* property — probe mid‑route corners):

| # | Milestone | Output | Effort |
|---|---|---|---|
| **S1** ✅ | **Texture A/B at fixed geometry (DONE 2026‑07‑09)**: seeds {42, 101, 107} × `--texture {0.0, 1.0}`, 6 bundles, `m4_lio_eval.sh` each, mechanism logged along every drive (`s1_corner_log.py`: corners + KLT fwd‑bwd survival). **Result: a controlled negative** — the dial alone does not degrade stereo VIO while corridor scatter fills the forward view (VIO RPE 0.009–0.022 m on both variants; corners ~174/frame and KLT survival 0.91–0.96 everywhere; LIO flat). Sharpens M4: alpine divergence needed *route‑wide* starvation. Route landmark *density* (S5) or the photometric axis (S2) is the lever that should bite. Walls hit + encoded: hardlink‑bundle corruption (bundler now real‑copies), vio_lio corridor blocked by multi‑metre `rock_moss_set` collisions (`clear_drive_lane.py` + upstream keep‑out flagged), OpenVINS load‑transient init wedge (eval restarts both frontends at the RTF gate). | `results/s1_texture_ab.png` ✅ | 1 session |
| **S2** | **Photometric + weather axes**: render‑gate ONE fog + ONE sunglare bundle FIRST (`GL_RENDERER`, camera frame via `peek_cam.py`, RTF via `bench_rtf.sh` — particle emitters are unproven in our ogre2/EGL path; WildSeed's own renderer crashed on weather pre‑`7708155`); if particles fail/starve RTF, restrict to fog + sunglare (scene‑level) and document. Then `--photometric {0, 0.5, 1.0}` at fixed seed; optional 2×3 texture×photometric grid. *Honesty note:* gz fog attenuates cameras but NOT `gpu_lidar` — sensor‑stress asymmetry by construction; the real‑snow tier (M7b) carries the physical claim. | second complementarity chart | 1 session |
| **S3** | **Hypothesis‑driven sweep harness**: `scripts/wildseed_axis_sweep.sh` — WildSeed experiment‑spec YAML (hypothesis + dial distributions) → `wildseed experiment --count N` → bundle each world immediately → `m4_lio_eval.sh` per bundle → merge WildSeed's report card with our ATE/RPE table (extend `plot_m4_sweep.py`); `provenance.json` ties every row to (WildSeed hash, spec, seed). Build after S1 proves the loop by hand; re‑run S2's grid through it. | the harness itself (test instrument, not demo) | 1 session |
| **S4** | **Calibration‑robustness**: adapt WildSeed's perturbed‑SDF + truth‑JSON pattern to the Husky — seeded perturbation of OAK‑D mount extrinsics (mm / tenths of °) + IMU noise via the **Clearpath generator patch path** (generator overrides xacro defaults; a xacro‑level sed is a silent no‑op — M4 lesson), true values exported to JSON. Matrix: perturbed sim × OpenVINS {fed nominals, online extrinsic calib ON} → fused ATE/RPE. Schedule after S1/S2. | estimator error vs miscalibration dial, with/without online calib — the robustness figure | own milestone |
| **S5** | *(optional)* custom biome via `--biome-file` (explicit‑select only): route‑texture YAML steering landmark density along the corridor — only if S1's binary texture lever is too coarse; else fold into the S1 writeup. | — | if needed |

---

## 13. Jetson phase — portability caveats
**Sim on the x86 laptop, inference on the Jetson** (don't run Gazebo on Orin — arm64 `ogre2` rendering is problematic). Connect over ROS 2 DDS.
1. **CPU arch:** `docker buildx --platform linux/amd64,linux/arm64`; `ros:jazzy-perception` is already multi‑arch.
2. **GPU/CUDA:** an amd64 CUDA image won't run on Jetson — use `--runtime nvidia` + an **L4T base matching the JetPack** (`nvcr.io/nvidia/l4t-jetpack:rXX`), picked via `${TARGETARCH}`.
3. **TensorRT engines are NOT portable** (laptop Turing CC 7.5 ≠ Orin Ampere CC 8.7) — **ship ONNX, build the `.engine` on the Jetson**.
4. **JetPack picks the distro:** JP 6.2.2 = Ubuntu 22.04 → ROS 2 **Humble**; JP 7.2 = Ubuntu 24.04 → ROS 2 **Jazzy** (matches this project — preferred if your Orin is on 7.2).
> No Jetson yet? M1–9 don't need one. Keep the `fusion` app layer arch‑agnostic so arm64 is a base‑image swap.

---

## 14. Risk register / backup plans

> These are *tactical* backups for specific failures. For **structural** "this package is dead / won't build" pivots, use the **§3.3 pivot matrix**. For **documented "obvious in hindsight" walls** (build flags, sim‑time, NavSat config, TF/GPS conventions) with their smoke tests, use the **§17 pre‑flight checklist**.

| Risk | Likelihood | Backup |
|---|---|---|
| Gazebo `ogre2` can't create window in Docker | High (default) | `NVIDIA_DRIVER_CAPABILITIES=all`; else `--headless-rendering`; last resort `LIBGL_ALWAYS_SOFTWARE=1` or Gazebo on host |
| Cross‑container DDS flaky | Medium | host networking + `ROS_DOMAIN_ID`; else CycloneDDS unicast |
| Optimus uses Intel GPU | Medium | `prime-select nvidia` / PRIME offload env |
| PyTorch GPU wheel won't load (driver 535) | Medium | `cu121` wheels; or YOLO on CPU; or upgrade driver |
| **6 cores saturated** (lidar+visual+tracker+sim live) | **Med–High** | stagger nodes; replay from bag; run one frontend at a time for eval |
| 8 GB VRAM tight | Low–Med | `yolov8n/s`, modest world; tracking on recorded data |
| **OpenVINS / GLIM source build pain** | Medium | prefer **apt** `rko_lio`/`mola_lidar_odometry` for lidar; OpenVINS is the only source‑build on the critical path — pin a known‑good commit |
| **Sim too low‑texture → OpenVINS starves (M3 sim‑first)** | **Med–High** | VIO needs trackable features; flat/untextured ground or repetitive sky → too few tracks → front‑end diverges and *all* downstream fusion is garbage. **Gate M3 on a feature‑count check** before trusting results; use a textured world (`pipeline` rugged terrain), add ground/wall texture or props, or fall back to M3b (EuRoC, real imagery) for the eval numbers |
| **OAK‑D BMI270 weak / sync offset** | Certain | don't make it the sole inertial source; use as stereo‑depth+detection; Spectacular AI for VIO if needed (§8) |
| **Spectacular AI is Humble, not Jazzy** | Medium | port the node, or run its SDK standalone publishing pose into ROS 2; it's a binary talking to the OAK |
| **depthai‑ros built‑in VSLAM disabled** | Certain | use depthai‑ros only for raw topics; do SLAM/VIO yourself or via Spectacular AI |
| **Sim depth too clean** (flatters pipeline) | Certain | inject depth noise/dropout + IMU time offset (§8) |
| Dataset ingestion friction | Low–Med | **stream‑publish adapters** (§9.1), no rosbag2 archive; small sets optionally cached; OpenVINS ships EuRoC rosbag2s |
| **MARS‑LVIG Livox `CustomMsg`** (non‑standard msg) | Low–Med | the `dataset_publishers/` adapter reads Livox → emits standard `PointCloud2`; or start lidar on **NTU VIRAL** (Ouster, native ring) |
| **`datasets/` fills the disk over many sequences** | **Designed out** | hard 60 GB cap in `fetch_seq.sh` + `prune.sh` auto‑delete after results saved (§9.1); stream‑publish kills the double‑copy |
| **Aerial sets gated / sizes unverified / TartanAir 57 TB** | Medium | `fetch_seq.sh` uses **selective** downloaders (modality/per‑sensor/per‑file); one seq at a time |
| `microsoft/AirSim` is **dead** (since Jul 2022) | Avoided | use **Cosys‑AirSim** (maintained) or Isaac Sim if you need custom snow scenes |
| Gazebo no GPS‑dropout toggle | Certain | gate in the fusion node (§11) |
| Jetson `.engine` won't load | Certain if mishandled | ship ONNX, build on‑device |
| Newest stack (Lyrical/Jetty) instability | Avoided | Jazzy+Harmonic; Lyrical = v2 branch |
| Disk fills (lidar data) | Low | gitignore datasets; start ~5 GB; 146 GB free |

---

## 15. Tutorial‑ization notes
- **Reproducibility is the product** — first command `git clone … && docker compose up sim` → visible result. Each milestone = tagged commit + section.
- **The "add a second odometry frontend with no code change" moment (M4) is the tutorial's wow** — it teaches the sensor‑agnostic pattern that real stacks use.
- **Explain the gotchas you hit** (`NVIDIA_DRIVER_CAPABILITIES`, DDS, engine non‑portability, OAK sim‑vs‑real) — that's what makes a tutorial worth bookmarking.
- **Pin versions** (image digests, apt versions, OpenVINS commit) so it still works in a year.
- **Keep it vendor‑neutral** — "perception for low‑cost, intermittently‑connected platforms," not a pitch.
- **Charts + clip** beat 3,000 words.

---

## 16. Assumptions & open decisions
1. **Spine = sensor‑agnostic, lidar + visual‑inertial frontends** — your call, confirmed. (Lidar = KISS‑ICP, lidar‑*only*; visual = OpenVINS VIO — see item 2.) Build order: one frontend working first (M3), then prove the architecture by adding the second (M4).
2. **Lidar frontend = KISS‑ICP** (lidar‑*only*, so your filter owns the IMU+GNSS fusion — the better showcase; §3.2 note); `rko_lio`/`mola` (apt, lidar‑*inertial*) as the robust fallback. **Visual frontend = OpenVINS** (maintained, native‑ROS 2, emits odometry; Basalt/Kimera as active alternatives if it gives trouble). ORB‑SLAM3 / VINS = reference/comparison only — *technical* reasons, not license: no official ROS 2, build‑hell on 24.04, loop‑closure ≠ clean odometry. *(License is not a selection factor at this stage.)* Confirm you're OK *not* hand‑writing the odometry frontend — the **fusion** is the showcase, and a lidar‑only lidar frontend maximizes how much fusion lives in your code.
3. **OAK‑D Lite = stereo‑depth + detection sensor, not sole VIO IMU** (§8). Real‑device VIO via Spectacular AI is an optional later chapter.
4. **Distro = Jazzy + Harmonic** (Lyrical = v2 branch only).
5. **Start on datasets**, Gazebo at M8–9; OAK‑D‑equivalent sim at M8.
6. **Jetson** HW‑gated; nothing else depends on it.
7. **Tracking eval** — start with position RMSE / ID‑switches; full MOTA/HOTA optional.
8. **Pivot philosophy (§3.3)** — none of the package picks above are commitments; each is column A of a ladder. If any walls you, drop to the apt fallback (B) before changing anything else. Write code against **standard ROS 2 APIs only** (no Jazzy‑only features) so the Jazzy→Humble escape hatch stays cheap. Two anchors are deliberately un‑deprecatable: the **hand‑rolled fusion core** (no external lib) and the **dataset‑replay floor** (no sim/driver needed).
9. **Datasets are ground/UGV‑first (§9), confirmed — not aerial.** Spine = Husky Gazebo sim + **KITTI / EuRoC** baselines + a **ground GPS‑denial set** (KITTI OXTS or Boreas automotive GNSS/INS). The aerial sets (TartanAir, MARS‑LVIG, NTU VIRAL) are kept **only as optional, harder‑than‑KITTI stress tests**; the "aerial‑Arctic" thesis is dropped. *(They were originally chosen because they're harder than KITTI — that's their only remaining role.)*
10. **Disk‑frugal data strategy (§9.1), confirmed:** stream‑publish (no archive) + one‑seq‑at‑a‑time + **hard 60 GB cap on `datasets/`** + auto‑prune + results‑only git. Datasets pass through, never accumulate — the disk can't fill up over time. Implies the `dataset_publishers/` adapter work in M4b (replaces bag conversion).
11. **Wheel odometry = a pluggable, toggleable input**. The Husky publishes wheel odometry natively; `ego_localizer` treats it as one more *configurable* measurement source — a launch/param flag to **add or remove** it, exactly like the lidar/visual odometry adapters (§5). This lets us A/B *with vs without* wheel odom (it bounds along‑track drift when KISS‑ICP degenerates or GPS drops — see `kiss-icp-failure-modes.md`), and to mimic platforms that lack wheel encoders. `kinematic-icp` (§3.2) is the related wheeled‑prior frontend option.

---

## 17. Pre‑flight checklist — documented walls & smoke tests

The "obvious only after you hit them" walls, found by reading the actual source/docs. Each has a one‑line **smoke test** — run it the moment you wire that piece, *before* building the next layer on top. This section is the answer to "don't let me hit a wall that was obvious in the docs."

### 17.1 Build / install
| Wall | Mitigation | Smoke test |
|---|---|---|
| **OpenVINS won't compile on Ubuntu 24.04** — Ceres 2.2 removed `LocalParameterization` | **`git clone -b master`** (fix PR #520, 2025‑11‑30). A `develop_vX`/release tag **will** fail to build. Fallback: run OpenVINS in a `ros:humble` container, bridge `/odom` over DDS; or use `rtabmap_odom` (apt) | `colcon build --packages-select ov_msckf` succeeds; on EuRoC bag `/ov_msckf/odomimu` publishes |
| KISS‑ICP & OpenVINS are **source‑only** (no apt) | colcon build the in‑repo `ros/` pkg; pin a known‑good commit. `pip install kiss-icp` gives only the offline CLI, **not** the ROS node | `ros2 pkg list \| grep -E 'kiss_icp\|ov_msckf'` |
| `rko_lio` needs **IMU↔LiDAR extrinsics** if not in TF | set `extrinsic_imu2base_*` + `extrinsic_lidar2base_*`, or provide via TF | node logs "initialized"; `/odom` advances |
| **pip↔apt Python clash — the real day‑one wall.** ✅*Verified in the real container:* `ros:jazzy-perception` ships **numpy 1.26.4 + OpenCV 4.6.0**, and `cv_bridge` is built against them. Installing `ultralytics`/`torch` via pip can drag in **numpy 2.x** + a pip `opencv-python` that shadow the apt ones → `cv_bridge`/`rclpy` ABI break | **pin `numpy<2`**; do **not** pip‑install `opencv-python` (use the apt cv2 4.6); or isolate YOLO in its own venv (`--system-site-packages`) / process. Pin torch `cu121` | in the *built* image: `python3 -c "import cv2,numpy,rclpy,torch,ultralytics; print(numpy.__version__)"` → `1.26.x`, no error |
| **PEP 668 on Ubuntu 24.04** — `pip install` into system Python **refuses** ("externally‑managed‑environment"). ✅*Verified present* | use a venv (`python3 -m venv --system-site-packages`) or deliberate `pip install --break-system-packages` in the Dockerfile | the Dockerfile's `pip` step succeeds in `docker build` |
| apt set co‑install | ✅*Verified:* all 6 apt pkgs + `ros_gz` resolve together cleanly in `ros:jazzy-perception` (dry‑run exit 0, no conflicts) — base image now pulled locally | `apt-get install -s <set>` exits 0, no `E:`/held |

### 17.2 Gazebo Harmonic + ros_gz
| Wall | Mitigation | Smoke test |
|---|---|---|
| **NavSat emits nothing** without `<spherical_coordinates>` **and** the NavSat *system plugin* in the world | add both (SDF below) | `ros2 topic echo --once /navsat` → lat/lon ≈ origin (not 0/nan); gz console has no "Spherical coordinates not set" |
| gz `gpu_lidar` cloud has **no per‑point time** (only xyz/intensity/ring) | synthesize per‑point time from azimuth/column+update_rate via the **`gz_lidar_timestamp` node** (`ros2_ws/src/gz_lidar_timestamp`), or run KISS‑ICP deskew‑off | `ros2 topic echo --once /lidar/points` field list = `xyz,intensity,ring`, no `t` |
| gpu_lidar/cameras need the **Sensors system + GPU render** (fail headless/CPU) | load `gz-sim-sensors-system`; GL caps in Docker (§7.2); or EGL `--headless-rendering` | the points/image topic is non‑empty |
| **`/clock` + sim time** — TF & `message_filters` **silently stall** if missing | bridge `/clock` gz→ROS; `use_sim_time:=true` on **every** node; `ros2 bag play --clock` | `ros2 param get <node> use_sim_time`=true on **all**; `/clock` advances; `tf2_echo` resolves w/o "extrapolation" |
| Camera `frame_id` is **body (x‑fwd), not REP‑103 optical (z‑fwd)** → stereo/depth misalign | bridge `override_frame_id:=*_optical_frame` + static TF body→optical | object appears upright/centered in RViz Image+cloud overlay |
| `depth_camera` = **two** bridged topics (image + points), no single mapping | bridge `.../image`, `.../depth_image`, `.../points` separately | all three present & non‑empty |
| **Ignition→Gazebo naming churn** breaks copy‑pasted old tutorials | use `gz` / `GZ_SIM_RESOURCE_PATH` / `gz-sim-*-system` / `gz.msgs.*` (not `ign*`) | `gz sim --versions` works; world loads, models found |

**NavSat world SDF (the exact block):**
```xml
<plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>45.4</latitude_deg> <longitude_deg>-75.7</longitude_deg> <elevation>70</elevation>
</spherical_coordinates>
```

### 17.3 Datasets
| Wall | Mitigation | Smoke test |
|---|---|---|
| **KITTI starves tight LIO/VIO** (synced IMU 10 Hz; lidar no per‑point time/ring) | build on **EuRoC** (200 Hz IMU) + **Boreas** (native time+ring, 200 Hz INS); KITTI only illustrates the limit (§9) | `ros2 bag info` → check IMU topic `hz`; if ~10 Hz it's the synced KITTI trap |
| `rosbags` convert: **dest must be a folder**; `metadata.yaml` QoS can be ill‑formed | output to a folder; hand‑fix QoS if `bag play` errors; register custom msgs first | `ros2 bag info` lists expected topics/types |
| Jazzy bag default = **mcap** (Humble = sqlite3); sqlite3 corrupts on `kill -9` | prefer mcap (self‑describing, Foxglove‑playable); `ros2 bag reindex` if metadata lost | `ros2 bag play` runs; Foxglove opens mcap w/o your msg ws |
| `rtabmap` stereo_odometry needs **rectified** `image_rect` + synced + `camera_info` | feed rectified topics; explicit remaps | rtabmap `/odom` advances (silent = sync never fired) |

### 17.4 Fusion / TF / GPS (the "silently wrong" ones)

> Distilled from the sourced deep-dive [`gps-fusion-conventions.md`](gps-fusion-conventions.md)
> (navsat_transform inputs, REP-103/105 conventions, IMU message requirements,
> the top-5 "silently wrong" causes + fixes). Update *there*; keep only the table here.

| Wall | Mitigation | Smoke test |
|---|---|---|
| **6‑axis IMU has no absolute heading** → `navsat_transform` GPS track rotates/drifts | sim a **magnetometer** or 9‑axis for the spine; or init yaw from **GPS course‑over‑ground** (`use_odometry_yaw` w/ earth‑referenced odom). (Same BMI270 limit as §8) | rotate 90°/closed loop: yaw returns, no drift at rest |
| `yaw_offset` default **changed in robot_localization 2.2.1** — now assumes IMU 0 = **east**; a north‑zero IMU needs `pi/2` | set `yaw_offset:=1.5707963`; set `magnetic_declination_radians` | `/odometry/gps` heading matches a known driven direction |
| **TF authority**: exactly ONE publisher of `map→odom` (global) and ONE of `odom→base_link` (a frontend) | dual‑EKF (§11); a frontend must NOT publish `map→odom`; set madgwick `publish_tf:=false` | `ros2 run tf2_tools view_frames` → single‑parent tree, no fighting |
| **GPS reacquisition jump** | dual‑EKF absorbs it into `map→odom`; Mahalanobis‑gate the first re‑fix | on dropout/reacquire, `odom→base_link` stays smooth |
| **QoS mismatch**: best‑effort sensor pub + reliable sub = **no connection, no error** | match reliability (sensor_data profile) | `ros2 topic info -v <topic>` sub count >0; `ros2 topic hz` non‑zero |
| IMU block "absent" = `covariance[0] == -1`; EKF needs `*_config` flags, **not** covariance inflation, to ignore a field | set `imuN_config` 15‑bool; don't fake huge covariance | `print_diagnostics:=true` shows fields used |

**Per‑milestone rule:** end every milestone by running its relevant smoke tests above — a green smoke test is the milestone's real "done," not "the node launched."

### Source note (verification provenance)
Versions/tags/pairings web‑verified; Docker tags + `ros_gz` pairing + `simulation` variant (REP 2001) directly re‑checked. **Jazzy apt releases confirmed via `rosdistro`:** `gtsam` 4.2.0, `robot_localization` 3.8.3, `rko_lio` 0.3.0, `mola_lidar_odometry` 2.2.1, `ros_gz` 1.0.23, `depthai-ros` 2.12.2 + `depthai_ros_v3` 3.2.1, `vision_msgs` 4.1.1, `message_filters` 4.11.17, `imu_tools` 2.1.5, `perception_pcl` 2.6.4, `octomap` 1.10.0, `rtabmap_ros` 0.22.1. **Source‑build (not in rosdistro):** OpenVINS, GLIM, Point‑LIO, KISS‑ICP (ROS 2), Spectacular AI SDK (binary). **OAK‑D facts** (BMI270, no HW sync ~10 ms, Spectacular AI Lite support, specs) tied to primary Luxonis/SpectacularAI/depthai‑core sources. **Fielded‑stack claims (§1)** are evidence‑based from company pages / DARPA / NVIDIA + GitHub liveness — internal stacks are proprietary, so treat company specifics as well‑sourced inference, not insider fact. Re‑confirm flagged items (EuRoC license/size, 4Seasons gating, Boreas per‑seq sizes, current JetPack, your unit's IMU via `getConnectedIMU()`) at build time. **§17 pre‑flight walls** were read from primary source: ros_gz `jazzy` bridge type table + gz‑sim8/gz‑sensors8 source (NavSat spherical_coords, gpu_lidar fields, `/clock`); OpenVINS issue #385 / PR #520 (Ceres 2.2); KISS‑ICP KITTI loaders (per‑point time); pyboreas `pointcloud.py` (Boreas time+ring); robot_localization `jazzy-devel` docs + REP‑103/105 (navsat heading, TF authority, `yaw_offset` 2.2.1 change).

---

## 18. Navigation & control modes — the planning half of the spine (v2.1 scope add)

> The north star (§0/§9) is a "classical **estimation + planning** spine," but §2/§5/§12 build only the *estimation* half. This section adds the **planning/control** half: one UGV under three operator modes — **remote‑controlled**, **semi‑autonomous** (operator sets waypoints, robot stops at / drives around obstacles), and **fully autonomous** (operator gives a destination, robot plans and drives there). It is layered **on top of** the fusion spine, not a rewrite — and it is the consumer that *justifies* the GPS‑denied design (REP‑105 keeps `odom→base_link` smooth precisely so a controller can track it through a GPS dropout; see §11 and the checklist's "GPS jumps are unfit for navigation").

**Stack (ROS 2 Jazzy; apt unless noted — re‑verify at build):**
- **Nav2** (`navigation2` / `nav2_bringup`) — global planner (NavFn/Smac), local controller (**MPPI**, the fielded choice §1), costmaps, **BehaviorTree.CPP** navigator, recoveries, `waypoint_follower`.
- **`twist_mux`** — priority arbitration of `cmd_vel` across input sources (teleop > assisted > autonomous > idle).
- **`teleop_twist_joy` / `teleop_twist_keyboard`** — manual input.
- **`slam_toolbox`** (✓ apt, all distros) — live map for full‑auto in an unmapped world; or a prebuilt costmap.
- **Clearpath `clearpath_nav2_demos`** — Husky‑specific Nav2 bringup (a Jazzy nav2 tutorial exists), so this is *integration*, not from‑scratch.

**Architecture (extends §5 — same topics, one new layer):**
```
 FUSION SPINE (§5)                          NAVIGATION LAYER (new)
 ego_localizer ─► /pose (+cov),             costmaps ◄─ Ouster OS1 + OAK‑D depth (§5.1 robot.yaml)
   TF map→odom→base_link  ────────────────► ├─ global planner   → path to goal
                                            ├─ local controller (MPPI) → follow + avoid
                                            ├─ BT navigator + recoveries
                                            └─ waypoint_follower
  joy/keyboard ─► teleop ──┐
  waypoint_follower ───────┼─► twist_mux ─(priority)─► /cmd_vel ─► Husky diff‑drive
  BT navigator ────────────┘        ▲
                            mode_manager (RC | semi | full) sets the active source; e‑stop/deadman always wins
```

**The three modes ↔ Nav2 mechanism:**

| Operator mode | Operator does | Robot does | Mechanism |
|---|---|---|---|
| **1. Remote control** | drives with joystick/keys | nothing autonomous | teleop → `twist_mux` → `cmd_vel` |
| **2. Semi‑autonomous** | drops a sequence of **waypoints** | drives waypoint→waypoint, **stops for / routes around** obstacles | `waypoint_follower` + local costmap + MPPI + recoveries |
| **3. Fully autonomous** | sets one **final destination** | plans a route and follows it, replanning around what it finds | full Nav2 BT navigator: **global** plan + **local** control, on a (SLAM or prebuilt) map |

A small **`mode_manager`** (state machine, or a BehaviorTree to stay on‑thesis with the fielded pattern §1) owns the RC↔semi↔full switch and sets `twist_mux` priorities.

**Frames / IO contract:** consumes `map→odom→base_link` + `/pose` from `ego_localizer`; emits `nav_msgs/Path` (planned) and `/cmd_vel`; costmaps subscribe the Husky's `lidar3d` points + OAK‑D depth. **No new estimator** — Nav2 trusts the spine's fused pose (this is the whole point of building the spine first).

**Honest off‑road caveat [F/I]:** Nav2's costmaps are **2D, warehouse/road heritage**. On the `pipeline` world's hills a 2D‑costmap first cut works; genuine off‑road **traversability** (slope, roughness, negative obstacles) is a **stretch chapter** — it is exactly the "learned traversability → classical MPPI" frontier flagged in §1. Document it as future work; don't promise it in the first pass. *(How the 2D costmap is built/updated from the Ouster + OAK‑D, and the ground‑segmentation problem: **deeper dive in `costmap-deep-dive.md`**.)*

**Pivot / floor (consistent with §3.3):** Nav2 is apt and battle‑tested; the `mode_manager` is a few‑hundred‑line state machine you own (un‑deprecatable, like `fusion_core`). If a planner/controller misbehaves on terrain, **mode 1 (teleop) is the floor** — always demonstrable — and modes 2/3 degrade (MPPI→DWB; full‑auto→waypoint) before being dropped.

**Added milestones (extend §12; HW‑independent, all in sim):**

| # | Milestone | Output | Effort |
|---|---|---|---|
| **N1** ✅ | **Teleop**: keyboard → `twist_mux` (`joy_teleop/cmd_vel`, prio 10) → diff_drive on the Husky in `pipeline`; e‑stop via the `twist_mux` `e_stop` lock. **Done** — `scripts/demo_n1_teleop.sh` (PASS), `nav-n1-teleop.md`. | odom‑trajectory plot (`img/n1_trajectory.png`) | ~0.5 wk |
| **N2** | **Semi‑auto MVP** (the headline nav slice): Nav2 **GPS waypoint following** (`FollowGPSWaypoints` + dual‑EKF/`navsat_transform`), **map‑less rolling global costmap** (no SLAM), local costmap from Ouster + OAK‑D depth, **RPP** controller (MPPI as upgrade). Operator drops setpoints; robot plans straight legs and **global‑replans around obstacles** found in between. Off‑road scope is deliberate: **fix obstacle‑vs‑ground discrimination** (gravity‑aware height band using the IMU), **defer** traversability + negative obstacles (`costmap-deep-dive.md`). | waypoint‑+‑avoid clip | 1–2 wk |
| **N3** | **Full‑auto**: destination → global plan → MPPI follow, on the **fused** pose, with `slam_toolbox` (or prebuilt) map. **Bonus money chart:** GPS dropout *during* a navigation run — pose holds, the robot keeps planning (ties modes 2/3 back to chart #1, §2). | autonomous‑traverse + GPS‑denied‑nav chart | 2 wk |

**Build order:** N1 after the Husky sim runs (M8); N2 once a frontend feeds `ego_localizer` (M3/M4); N3 after the GPS‑dropout keystone (M5). Navigation rides on milestones already planned — it does not fork the roadmap.

**N1 deadman scope:** for keyboard/SSH the N1 safety primitive is
the latching `twist_mux` `e_stop` lock (held `true` engages, `false` releases) plus the
`joy` input's 0.5 s timeout (publisher stops → robot stops). A true *hold‑to‑drive*
joystick deadman is **deferred** to a joystick teleop mode (needs `teleop_twist_joy`).
See `nav-n1-teleop.md`, incl. the use_sim_time **timestamp** gotcha (`ros2 topic pub`
stamps 0 → diff_drive drops stale cmds; drive via a node that stamps `now()`).

**N1 follow‑ups — all RESOLVED (verified on the laptop):**
- ✅ **Teleop "drives forever" — the wall‑stamp side effect.** `teleop_twist_keyboard`
  (the `teleop` service, no `use_sim_time`) stamped **wall time**, far in the future vs sim
  time, so diff_drive's `cmd_vel_timeout` check (`sim_now − stamp > timeout`) was always
  false → the command **never expired** → one keypress latched motion until the robot hit a
  wall. **Fix (done):** added `-p use_sim_time:=true` to the teleop service command
  (`docker/compose.yaml`) so it stamps sim‑time `now()`. **Verified:** (a) a `use_sim_time`
  node in a *separate* container on host‑net DDS receives the bridged `/clock` (sim time
  ~51 s, not 0); (b) after sim‑stamped commands stop, the 0.5 s `cmd_vel_timeout` fires and
  diff_drive ramps the robot to a full stop (vx 0.50→0.39→0.18→0.03→**0.00** over ~3 s,
  bounded by the 1.0 m/s² accel limit). Caveat: the halt is a ramp‑down (~2–3 s), not an
  instant stop — that's the diff_drive accel profile, not the stamp; for an immediate stop
  use the teleop `k`/space zero key or the e‑stop.
- ✅ **RViz odom arrow trail.** `config/husky.rviz` Odometry display had `Keep: 100`
  (accumulating trail of pose arrows). **Fix (done):** set `Keep: 1` — shows only the
  current pose arrow.
- ✅ **"Duplicate `clock_bridge`" — NOT a duplicate (premise was stale).** Runtime check:
  `ros2 node list` shows exactly **one** `/clock_bridge` (ours, from `husky_sim.launch.py`)
  and `/clock` has **Publisher count: 1**. `robot_spawn.launch.py` adds **no** clock bridge
  (the Clearpath clock bridge lives only in `clearpath_gz/gz_sim.launch.py`, which our launch
  deliberately does **not** use — we drive gz via `ros_gz_sim/gz_sim.launch.py`, see
  sim‑debugging #7). So our explicit bridge is the **sole, required** `/clock` publisher —
  removing it would break `/clock`. No change needed; the second bridge the note feared
  never existed in the current launch path.

## 19. Status & verification — moved

Current milestone status, the per-milestone **verification log** (evidence), and the
next-session roadmap live in **[`status-and-testing.md`](status-and-testing.md)** —
the single per-session-updated home. This PLAN stays timeless design/rationale;
references elsewhere to "PLAN §19 / §19.1 / §19.2" resolve there.
