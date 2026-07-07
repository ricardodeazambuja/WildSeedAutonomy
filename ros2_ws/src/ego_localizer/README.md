# ego_localizer

The ego-pose EKF node — wraps [`fusion_core`](../fusion_core) to fuse the spine
IMU + odometry + the odometry frontends into one `nav_msgs/Odometry`.
PLAN §5/§5.1, **milestones 3 + 4**.

Sensor-agnostic **by demonstration, not just construction**: the M3 visual
frontend (OpenVINS, `visual_delta_update`) and the M4 lidar frontend
(KISS-ICP, `lidar_delta_update`) are the SAME body-frame-delta measurement
model — the second one landed as a new subscription + one delegating method,
zero filter-code changes ([`docs/m4-lio.md`](../../../docs/m4-lio.md)). GNSS
(M5) is the droppable absolute fix.

## Layout
| File | Purpose |
|---|---|
| `ego_localizer/estimator.py` | ROS-free `PlanarPoseEstimator` — state `[px,py,yaw,vx,vy,wz]`, CV predict; IMU, odom, VIO-delta (M3), lidar-delta (M4), GNSS + course updates (yaw innovations wrapped). Deterministic, unit-tested. |
| `ego_localizer/node.py` | thin ROS plumbing: subscriptions per enabled source, predict-to-now, publish fused odometry (+ optional TF). `lidar_min_dt` gives the lidar deltas a minimum baseline (scan-to-scan ICP deltas have SNR<1 at UGV speeds — see m4-lio.md). |
| `config/ego_localizer.yaml` | Husky-sim topics + tuning (absolute-odom mode, M3 foundation). |
| `config/ego_localizer_visual.yaml` | M3 chart config: VIO alone + IMU yaw-rate (no wheel odom, no GNSS). |
| `config/ego_localizer_lidar.yaml` | M4 chart config: lidar odometry alone + IMU yaw-rate; σ fit from measured residuals — NOT copied from the VIO config. |
| `config/ego_localizer_gnss.yaml` | M5 keystone config: relative odom + droppable GNSS. |
| `launch/…launch.py` | one launcher per config. |
| `test/` | offline pytest (11): heading fusion beats odom heading, fused position beats raw odom + tracks truth, VIO/lidar frame-cancellation + hook equivalence, keystone drift→reacquire, covariance stays sym-PSD. |

## Run
```bash
# against the live Husky sim (deploy.sh up compute), from a fusion container:
ros2 launch ego_localizer ego_localizer.launch.py    # after colcon build + source
# publishes /ego_localizer/odom (frame odom -> base_link), publish_tf off by default
```

## Verified (laptop sim)
- Offline: 6 pytest green — fused **heading** ~12× better than raw odom heading;
  fused **position** beats raw odom and tracks truth to ~10 cm.
- Live on the Husky sim: drove ~4 m forward; `/ego_localizer/odom` tracked
  `/a200_0000/platform/odom` to ~1 mm (−1.96→2.02 vs −2.00→2.02).

## GPS-denied keystone (M5, in sim) — `odom_mode: relative` + GNSS
`config/ego_localizer_gnss.yaml` / `ego_localizer_gnss.launch.py` run the node with
wheel odom as a **relative** twist source (dead-reckons) + GNSS (`NavSatFix`→ENU) as
the **droppable** absolute fix. Toggle GPS at runtime:
```
ros2 topic pub --once /ego_localizer/set_gps_enabled std_msgs/msg/Bool '{data: false}'
```
`scripts/gps_denied_demo.py` drives + drops + reacquires and logs ego vs GPS.
Heading is anchored to ENU by **GPS course-over-ground** (`heading_update`), with the
IMU contributing **yaw-rate only** (`imu_rate_update`) — because the gz IMU's absolute
yaw is in the gz world frame, not ENU (§17.4). This is what stops the dead-reckoning
from spiralling.
- **Offline:** `test_gps_denied_keystone...` (both dense-GPS and course-aided variants)
  pass — bounded → drift → recover.
- **Live (sim):** the §17.4 spiral is **fixed** and the keystone is demonstrated —
  `results/gps_denied_keystone.png` (slow drive, 40 s outage): mean |ego−GPS|
  on 0.12 → denied 0.20 → reacquire 0.14 m; error envelope rises during the outage and
  recovers after. Reproduce: `scripts/gps_denied_demo.py` + `scripts/plot_gps_denied.py`
  (sim up + `ego_localizer_gnss.launch.py`). See PLAN §19.1–§19.2.

## Frontends (M3 + M4, both done sim-first)
- **Visual (M3):** stereo OpenVINS → `/odomimu` → `visual_delta_update`; raw
  ATE 0.069 m / fused 0.077 m over the 20.5 m chart drive
  ([`docs/m3-vio.md`](../../../docs/m3-vio.md)).
- **Lidar (M4):** KISS-ICP → `/kiss/odometry` → `lidar_delta_update`; A/B'd
  against the VIO in the same drive on four worlds — complementary failure
  modes measured ([`docs/m4-lio.md`](../../../docs/m4-lio.md),
  `results/m4_terrain_sweep.png`).

## Remaining (real-data tiers)
OpenVINS on EuRoC vs Vicon (M3b) and KISS-ICP on NTU VIRAL (M4 real tier) —
the dataset/eval layers on top of this node. See PLAN §12.
