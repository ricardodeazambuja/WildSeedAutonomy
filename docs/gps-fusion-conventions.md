# GPS + Odometry + IMU Fusion in ROS 2 — Documented Conventions & Gotchas

> This is the **sourced deep-dive behind PLAN §17.4** ([`PLAN.md`](PLAN.md)) — the
> distilled wall/mitigation/smoke-test table lives there; update conventions and
> source quotes *here*. The 6-axis-IMU-heading wall this document predicted was hit
> and fixed in sim (course-aided heading) — see the verification log in
> [`status-and-testing.md`](status-and-testing.md).

Scope: hand-rolled EKF + `robot_localization` baseline, fusing lidar/visual odometry + a **6-axis IMU (no magnetometer/heading)** + GNSS. The conventions and parameter behaviors below are quoted from primary sources (URLs given). A few items — the `sensor_msgs/Imu` covariance[0]=-1 "field absent" convention, per-node `use_sim_time`, `message_filters` ApproximateTime behavior, and the `ros2 ...` smoke-test commands — are standard ROS 2 message-definition / tooling practice rather than lines in those specific docs; they are flagged where they appear.

Primary sources used (all fetched verbatim from canonical repos / docs, ROS 2 `jazzy`):
- `navsat_transform_node.rst` — https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/doc/navsat_transform_node.rst
- `integrating_gps.rst` — https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/doc/integrating_gps.rst
- `preparing_sensor_data.rst` — https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/doc/preparing_sensor_data.rst
- `state_estimation_nodes.rst` — https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/doc/state_estimation_nodes.rst
- Rendered docs: https://docs.ros.org/en/jazzy/p/robot_localization/
- REP-103 (Units & Coordinate Conventions) — https://www.ros.org/reps/rep-0103.html (source: https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
- REP-105 (Coordinate Frames for Mobile Platforms) — https://www.ros.org/reps/rep-0105.html (source: https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
- `imu_filter_madgwick` (config + ROS node) — https://github.com/CCNYRoboticsLab/imu_tools/tree/jazzy/imu_filter_madgwick
- ROS 2 QoS concepts — https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html (source: https://github.com/ros2/ros2_documentation/blob/rolling/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

---

## 1. `navsat_transform_node` — exact inputs and the heading gotcha

### Required inputs (all three)
`navsat_transform_node` subscribes to exactly three topics (source: `navsat_transform_node.rst` "Subscribed Topics"):

| Topic | Type | What it must contain |
|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | "A `sensor_msgs/Imu` message **with orientation data**" — an absolute, **earth-referenced heading** |
| `odometry/filtered` | `nav_msgs/Odometry` | The robot's current pose (normally the EKF output). "needed in the event that your first GPS reading comes after your robot has attained some non-zero pose." |
| `gps/fix` | `sensor_msgs/NavSatFix` | Raw lat/lon (optionally altitude) |

`integrating_gps.rst` ("Required Inputs") states it plainly: *"navsat_transform_node requires three sources of information: the robot's current pose estimate in its world frame, an **earth-referenced heading**, and a geographic coordinate expressed as a latitude/longitude pair."* The default-mode message list explicitly requires *"A sensor_msgs/Imu message with an **absolute (earth-referenced) heading**."*

Output: `odometry/gps` (`nav_msgs/Odometry`) — GPS transformed into the robot's world frame, ready to fuse as an `odomN`/`poseN` input. It does NOT publish `map->odom` itself; it produces a fusable measurement, and (optionally) the `utm`->world TF.

### (a) Gotcha: a 6-axis IMU has NO absolute heading — this is the core problem
navsat_transform needs an **earth-referenced yaw** to rotate GPS into the map frame. A 6-axis IMU (accel + gyro, no magnetometer) can recover absolute **roll/pitch** from gravity, but its **yaw is gyro-integrated** → only relative, drifting, with an arbitrary zero. Feeding that into navsat_transform means the `utm`->`map` rotation `θ` is wrong/drifting, so the **fused GPS track comes out rotated and/or smeared** relative to the true world frame. The math is explicit in `integrating_gps.rst` ("Details"): `θ = yaw_imu + ω + offset_yaw`, where `yaw_imu` MUST be the absolute heading.

Documented ways to supply an absolute heading without a magnetometer:
- **Dual-antenna RTK GNSS** → true heading published as IMU/odometry orientation.
- **`use_odometry_yaw: true`** (navsat param): *"navsat_transform_node will not get its heading from the IMU data, but from the input odometry message. Users should take care to only set this to true if your odometry message has orientation data specified in an **earth-referenced frame**, e.g., as produced by a magnetometer."* (Does not manufacture heading — the odometry must already be earth-referenced.)
- **Heading from GPS motion** (course-over-ground) once moving — but that is undefined at rest and is not what navsat_transform synthesizes for you.
- A magnetometer feeding an orientation filter (see §3) — but a mag is exactly what a 6-axis OAK IMU lacks.

> Trap (flagged for the writeup): running `imu_filter_madgwick`/`imu_complementary_filter` on a 6-axis IMU **without a magnetometer does NOT solve this** — it gives absolute roll/pitch but only relative (gyro-integrated) yaw. See §3.

### (b) `magnetic_declination_radians` + `yaw_offset` (ENU vs the IMU's NED-style zero)
- `magnetic_declination_radians`: *"Enter the magnetic declination for your location... This parameter is needed if your IMU provides its orientation with respect to the magnetic north."* Find it at https://www.ngdc.noaa.gov/geomag-web (convert to radians). It corrects magnetic north → true north.
- `yaw_offset`: *"Your IMU should read 0 for yaw when facing **east**. If it doesn't, enter the offset here (`desired_value = offset + sensor_raw_value`). For example, if your IMU reports 0 when facing **north, as most of them do**, this parameter would be `pi/2` (~1.5707963)."*
- **Version note (do not get this wrong):** *"This parameter changed in version 2.2.1. Previously, navsat_transform_node assumed that IMUs read 0 when facing north."* Since 2.2.1 the assumed zero is **east**. So on modern ROS 2 (`jazzy`), a north-zero IMU needs `yaw_offset = pi/2`. Confirm against your installed version.
- Why east: REP-103 ENU defines yaw=0 along +X = **east**, increasing **counter-clockwise** — *"this differs from a traditional compass bearing, which is zero when pointing north and increments clockwise."*

### (c) datum / first-fix initialization
- **Default mode (`wait_for_datum: false`)**: the world-frame origin is set from the **first GPS fix + first IMU heading + current odometry pose**. The `utm->map` transform is computed once and reused.
- **`wait_for_datum: true`**: node waits for a `datum` param `[lat, lon, heading_rad, world_frame, base_link_frame]` (e.g. `[55.944904, -3.186693, 0.0, map, base_link]`) or the `set_datum` service. *"the robot assumes that your robot's world frame origin is at the specified latitude and longitude and with a heading of 0 (east)."*
- Other params: `zero_altitude` (force Z=0 for 2D), `publish_filtered_gps` (`/gps/filtered` back-projection for sanity-checking), `broadcast_utm_transform` / `broadcast_utm_transform_as_parent_frame`, `delay`, `frequency`, `transform_timeout`.

### Failure modes when (a)/(b)/(c) are wrong
- **Wrong `yaw_offset` (north-zero IMU, offset left at 0):** entire GPS track **rotated ~90°** about the start point — "the path comes out sideways."
- **Missing/wrong declination:** track rotated by the declination angle (small but systematic; grows the error far from origin).
- **No absolute heading (6-axis only):** rotation `θ` is wrong/drifting → GPS fusion drifts and rotates; the estimate fights itself.
- **`odomN_differential` left true on the GPS input:** *"The GPS is an absolute position sensor, and enabling differential integration defeats the purpose of using it."* (`integrating_gps.rst`) — set it **false**.
- **Bad datum / first fix taken before heading is valid:** the one-time `utm->map` transform bakes in a wrong rotation for the whole session.

---

## 2. REP-103 / REP-105 — conventions that silently break things

### Axes & handedness (REP-103)
- **All frames right-handed** (right-hand rule).
- **Body frame:** **x forward, y left, z up.**
- **Short-range world frame:** **ENU** — **X east, Y north, Z up.** (Choose a nearby origin to avoid float32 precision loss.)
- **Yaw:** zero pointing **east**, increases **counter-clockwise** — explicitly *not* a compass bearing (north/clockwise). "Hardware drivers should make the appropriate transformations before publishing standard ROS messages."
- **NED** is allowed only as a suffixed secondary frame (`*_ned`: X north, Y east, Z down) — `robot_localization` does **not** accept NED IMU data (see §3).

### Camera optical frame — the classic TF mistake (REP-103 "Suffix Frames")
A camera has TWO frames:
- Body/link frame: x-forward, y-left, z-up (REP-103 body convention).
- **`*_optical_frame`: z forward, x right, y down.**

If you publish visual-odometry poses (or point clouds) in the optical frame but feed them to the EKF as if they were body frame, **every axis is permuted** → the trajectory is rotated/mirrored and fusion diverges. Always insert the static `base_link -> camera_link -> camera_optical_frame` chain and make sure VO output is expressed in (or transformed to) a body-convention frame before fusion.

### Frame ordering & authority (REP-105 "Relationship between Frames" / "Frame Authorities")
- Tree: **`earth -> map -> odom -> base_link`.** *"The map frame is the parent of odom, and odom is the parent of base_link. Although intuition would say that both map and odom should be attached to base_link, this is not allowed because **each frame can only have one parent**."*
- **Who publishes what:**
  - `odom -> base_link`: *"computed and broadcast by one of the odometry sources"* (your local/continuous EKF, wheel/visual/IMU odom).
  - `map -> base_link`: computed by the localization component, **but it does NOT broadcast `map->base_link`**. *"Instead, it first receives the transform from odom to base_link, and uses this information to broadcast the transform from map to odom."* So the GPS/global EKF publishes **`map -> odom`**, never `map -> base_link`.
- **Therefore a local odometry frontend must NOT also publish `map->odom`** if a GPS/global node owns it — two publishers of the same edge = TF fight, jumping tree, broken lookups. `robot_localization` makes this explicit: when `world_frame == map_frame`, *"Make sure something else is generating the odom->base_link transform... that instance should not fuse the global data."* (`state_estimation_nodes.rst`)
- **Continuity contract:** `odom` is **continuous but drifts** (good for control/local planning); `map` is **drift-free but jumps** (good for global reference, bad for local sensing). This is why GPS goes in the map-frame EKF, not the odom-frame one.
- **Float precision:** centimeter accuracy in the `odom` frame holds out to ~**83 km** from origin before float32 degrades; reset the odom origin on very long runs.

---

## 3. IMU message requirements — orientation needed or not?

`sensor_msgs/Imu` has three blocks: `orientation` (quaternion), `angular_velocity`, `linear_acceleration`, each with a covariance. A field is "absent" when its covariance[0] is set to **-1**.

Who needs what:
- **EKF / UKF (`ekf_node` / `ukf_node`) — orientation NOT required.** You select which IMU fields to fuse via `imuN_config` (the 15-bool `[X,Y,Z,roll,pitch,yaw, vX,vY,vZ, vroll,vpitch,vyaw, aX,aY,aZ]`). With a 6-axis IMU you fuse **angular velocity (`vroll,vpitch,vyaw`)** and optionally **linear acceleration**, and leave the absolute roll/pitch/yaw bits **false**. The EKF runs fine on a 6-axis IMU.
  - Caveat: `imuN_remove_gravitational_acceleration` *"assumes that the IMU... is also producing an absolute orientation. The orientation data is required to correctly remove gravitational acceleration."* So if you fuse linear acceleration **and** want gravity removed, you need orientation — otherwise don't fuse raw accel, or handle gravity yourself.
- **`navsat_transform_node` — orientation REQUIRED** (absolute earth-referenced yaw). This is the node a 6-axis IMU cannot satisfy on its own (see §1a).

### Synthesizing orientation: `imu_filter_madgwick` and the magnetometer trap
`imu_filter_madgwick` (CCNYRoboticsLab/imu_tools) turns a 6-axis (or 9-axis) IMU into a `sensor_msgs/Imu` with a filled orientation quaternion. Confirmed from its `config/imu_filter.yaml` and `src/imu_filter_ros.cpp`:
- Default `world_frame: "enu"` (matches robot_localization), with `ned`/`nwu` options.
- `use_mag` (default `true`): subscribes to `/imu/mag` (`sensor_msgs/MagneticField`) and applies `yaw_offset_total = yaw_offset - declination`, producing an **absolute, earth-referenced** heading.
- **With `use_mag: false` (6-axis case): the filter computes orientation from accel+gyro only. Gravity fixes absolute roll/pitch, but yaw is gyro-integrated → RELATIVE heading with an arbitrary zero, drifting over time.** It is NOT earth-referenced.

**Consequence:** Madgwick/complementary on a 6-axis IMU **does not give navsat_transform the absolute heading it needs.** It gives the EKF good roll/pitch (useful), but the absolute-heading problem is unsolved. You still need a magnetometer, a dual-antenna RTK heading, or `use_odometry_yaw` fed by an earth-referenced source. Don't let "just add Madgwick" imply GPS fusion will work.

**TF-authority footgun:** the shipped `imu_filter.yaml` has **`publish_tf: true`** with `fixed_frame: "odom"`. Run as an orientation source for the EKF, the filter then broadcasts `odom -> <imu frame_id>`, creating a **second parent path** that fights the EKF's own `odom -> base_link` authority (the §2 "who publishes each frame" mistake). **Set `publish_tf: false`** when using Madgwick purely as a sensor input to the EKF.

### Covariance & sign rules (`preparing_sensor_data.rst`)
- Do **not** inflate covariance to 1e3 to "ignore" a variable — set the `*_config` bool **false** instead. Inflated covariance is "unnecessary and even detrimental."
- A fused variable with **0 variance** gets an epsilon (1e-6) added — set covariances properly.
- Signs must follow REP-103: turn CCW → yaw increases; drive forward → +X. Wrong signs are a listed "common error."
- Acceleration sanity: right-side-up flat IMU reads **+9.81 on Z**; rolled +90° (left up) → +9.81 on Y; pitched +90° (nose down) → −9.81 on X.

---

## 4. Time / sync

- **`use_sim_time` must be identical across ALL nodes.** If the EKF has `use_sim_time:=true` (replaying a bag / sim clock on `/clock`) but a driver or navsat_transform has it false (wall clock), their timestamps live on different timelines → message_filters/TF lookups reject everything or the filter stalls. Mixed sim/real time is a silent, total-failure mode. Set it the same everywhere (launch-wide `use_sim_time` arg). Note ROS 2 has no global `/use_sim_time` param — it is **per node**, so it is easy to miss one.
- `robot_localization` exposes **`reset_on_time_jump`**: *"If true and `ros::Time::isSimTime()` is true, the filter will reset... when a jump back in time is detected"* — set true for looping bags.
- **ApproximateTime / `message_filters` tolerance:** when you sync streams (e.g. stereo VO, or IMU+image), the slop must cover real inter-sensor latency. Too tight → sync starves (no synchronized callbacks, looks "hung"); too loose → you pair mismatched samples and inject motion error. Match `queue_size`/slop to the slowest stream's period. (`robot_localization` itself does NOT ApproximateTime-sync inputs — it timestamps and queues each independently; `*_queue_size` must be large enough when sensor rate >> filter `frequency`.)
- Make sure **every message has a valid, monotonic `header.stamp`** in the chosen timeline. Zero or wall-clock stamps under sim time are a classic stall.

---

## 5. GPS-denied behavior and re-fusing on reacquisition without a jump

This is exactly why `robot_localization` documents the **dual-EKF** pattern (`integrating_gps.rst` "Notes on Fusing GPS Data"):

1. **Local EKF** — `world_frame == odom_frame`. Fuses ONLY continuous data (wheel/visual/lidar odom + IMU). Publishes **`odom -> base_link`**. "Execute local path plans and motions in this frame."
2. **Global EKF** — `world_frame == map_frame`. Fuses everything **including** `odometry/gps`. Publishes **`map -> odom`**.

Rationale (quoted): *"using a position estimate that includes GPS data will likely be unfit for use by navigation modules, owing to the fact that GPS data is subject to discrete discontinuities ('jumps')."*

- **When GPS drops:** the **local EKF keeps producing `odom->base_link` uninterrupted** from odom+IMU — control and local planning are unaffected (odom is continuous by REP-105 contract). The global EKF coasts on the same continuous inputs; its `map->odom` correction simply stops being updated by GPS and the estimate drifts at odom's drift rate. **Nothing breaks; you just lose global drift-correction.**
- **On GPS reacquisition:** the new GPS measurement re-enters the **global** EKF only. Because the jump is absorbed into the **`map->odom`** transform (not `odom->base_link`), the `base_link` pose used for control stays continuous — the discontinuity lives entirely in the map frame, which is *allowed* to jump (REP-105). This is the documented way to re-fuse GPS "without a jump" in the control/odom frame.
- Keep `gps` input `_differential: false` (it's an absolute sensor). Mahalanobis `*_rejection_threshold` params can gate a wild first re-fix so a single bad fix doesn't yank the map.

---

# Deliverable A — Checklist of conventions to honor

- [ ] **Axes:** body = x-fwd/y-left/z-up; world = ENU (X-east/Y-north/Z-up); right-handed. (REP-103)
- [ ] **Yaw zero = east, CCW positive** — not compass north/clockwise. Drivers must convert. (REP-103)
- [ ] **No NED into robot_localization.** Convert IMU to ENU first. (`preparing_sensor_data.rst`)
- [ ] **Camera optical frame** (`*_optical_frame`: z-fwd/x-right/y-down) is distinct from camera body frame — publish the static TF chain; express VO in body convention before fusing. (REP-103)
- [ ] **TF tree** `earth->map->odom->base_link`, each frame one parent. (REP-105)
- [ ] **`odom->base_link` published by exactly one local odometry source.** Disable any duplicate (e.g. robot driver's own odom TF). (REP-105, `preparing_sensor_data.rst`)
- [ ] **`map->odom` published by the global/GPS node — never `map->base_link`.** Frontend must NOT also publish `map->odom`. (REP-105)
- [ ] **navsat_transform inputs:** `imu/data` (absolute heading), `odometry/filtered`, `gps/fix`. (`navsat_transform_node.rst`)
- [ ] **`yaw_offset = pi/2`** for a north-zero IMU on RL ≥ 2.2.1 (east-zero assumption); verify version. (`navsat_transform_node.rst`)
- [ ] **`magnetic_declination_radians`** set from NOAA, in radians. (`integrating_gps.rst`)
- [ ] **GPS input `_differential: false`** (absolute sensor). (`integrating_gps.rst`)
- [ ] **Dual-EKF:** local (odom) for continuous data + control; global (map) for everything incl. GPS. (`integrating_gps.rst`)
- [ ] **IMU covariances real**, never inflated to ignore; use `*_config` bools to drop variables; fix signs; +9.81 on Z right-side-up. (`preparing_sensor_data.rst`)
- [ ] **`use_sim_time` identical on every node**; valid monotonic `header.stamp` everywhere; `reset_on_time_jump` for looping bags.
- [ ] **QoS:** EKF/navsat subscriptions must be reliability-compatible with sensor publishers (see Top-5 #1).
- [ ] **`imu_filter_madgwick` as sensor source: set `publish_tf: false`** so it doesn't broadcast `odom->imu` and fight the EKF's `odom->base_link` authority.

# Deliverable B — Top 5 "GPS fusion silently wrong" causes + fixes + smoke test

**1. QoS reliability mismatch — sensor publisher (best-effort) vs EKF/navsat subscriber (reliable) → connection never forms, zero error.**
- Source: ROS 2 QoS table — *Best effort publisher + Reliable subscription = NOT compatible*. https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- Fix: make the subscriber best-effort (or the publisher reliable) so they match. Many drivers (IMU/GPS/camera) use the sensor-data profile (best-effort, small depth).
- Smoke test: `ros2 topic info -v /imu/data` and `/gps/fix` — confirm publisher count, subscriber count, and matching Reliability. If `Subscription count: 0` on the publisher side or the EKF never updates, it's QoS. Cross-check with `ros2 topic hz /odometry/filtered` (no output = not fusing).

**2. Wrong `yaw_offset` (north-zero IMU left at 0) → entire GPS trajectory rotated ~90°.**
- Source: `navsat_transform_node.rst` (`yaw_offset`, version-2.2.1 east-zero change).
- Fix: set `yaw_offset: 1.5707963` for a north-zero IMU on RL ≥ 2.2.1; verify against installed version.
- Smoke test (discriminating): drive a straight line in a known compass direction and compare the direction of `/odometry/gps` against real-world movement (`ros2 topic echo /odometry/gps`, or plot in RViz). A consistent ~90° (or declination-sized) rotation of the whole track ⇒ yaw_offset/declination. Note: with a 6-axis IMU `/imu/data` carries no orientation, so test against whatever produces the absolute heading (Madgwick+mag / RTK / `use_odometry_yaw` source), not `/imu/data` directly.

**3. 6-axis IMU has no absolute heading (and Madgwick-without-mag doesn't fix it) → navsat rotation drifts/wrong.**
- Source: `integrating_gps.rst` (earth-referenced heading required); `imu_filter_madgwick` `use_mag` (mag needed for absolute yaw).
- Fix: supply absolute heading via magnetometer-backed filter, dual-antenna RTK, or `use_odometry_yaw:true` with an earth-referenced odom source. Do NOT rely on `use_mag:false` Madgwick for heading.
- Smoke test: rotate the robot 90° in place; `ros2 topic echo` the IMU/odom orientation. If yaw drifts when stationary or doesn't return to the same value after a closed loop, the heading is relative, not absolute — navsat will be wrong.

**4. Two nodes publishing the same TF edge / frontend publishing `map->odom` (or robot driver also publishing `odom->base_link`) → jumping, inconsistent tree.**
- Source: REP-105 Frame Authorities; `state_estimation_nodes.rst` ("Make sure something else is generating odom->base_link... should not fuse the global data").
- Fix: exactly one publisher per edge. Local EKF owns `odom->base_link`; global EKF owns `map->odom`; disable the robot driver's odom TF broadcast.
- Smoke test: `ros2 run tf2_tools view_frames` → inspect `frames.pdf`; each frame must have one parent and one broadcaster. `ros2 run tf2_ros tf2_monitor map odom` flags multiple authorities / large rates.

**5. Inconsistent `use_sim_time` (or bad/zero timestamps) → filter stalls or silently ignores data.**
- Source: ROS 2 per-node `use_sim_time`; `reset_on_time_jump` (`state_estimation_nodes.rst`).
- Fix: set `use_sim_time` identically on every node; ensure valid monotonic `header.stamp`; `reset_on_time_jump:true` for looped bags.
- Smoke test: `ros2 param get <node> use_sim_time` for every node — all must agree. `ros2 topic echo --field header.stamp /imu/data` vs `ros2 topic echo /clock` — stamps must be on the same timeline and advancing. EKF `print_diagnostics:true` → watch `/diagnostics` for "timestamp" / "older than" warnings.

Bonus (covariance): GPS fused with `_differential:true`, or IMU variables ignored via inflated covariance instead of `*_config:false` — both silently degrade. Verify with `ros2 topic echo /odometry/gps` covariance and your `*_config`/`*_differential` params.
