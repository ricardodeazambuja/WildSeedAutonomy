# ego_localizer

The ego-pose EKF node — wraps [`fusion_core`](../fusion_core) to fuse the spine
IMU + odometry into one `nav_msgs/Odometry`. PLAN §5/§5.1, **milestone 3
foundation**.

Sensor-agnostic by construction: adding a frontend (lidar M4, visual M4, GNSS
M5) is a new subscription + `*_update` call, **not** new filter code.

## Layout
| File | Purpose |
|---|---|
| `ego_localizer/estimator.py` | ROS-free `PlanarPoseEstimator` — state `[px,py,yaw,vx,vy,wz]`, CV predict, IMU + odom updates (yaw innovations wrapped). Deterministic, unit-tested. |
| `ego_localizer/node.py` | thin ROS plumbing: subscribe IMU + odom, predict-to-now, publish fused odometry (+ optional TF). |
| `config/ego_localizer.yaml` | Husky-sim topics + tuning (process / measurement noise). |
| `launch/ego_localizer.launch.py` | launch with that config. |
| `test/` | offline pytest: heading fusion beats odom heading, fused position beats raw odom + tracks truth, covariance stays sym-PSD. |

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

## Remaining for full M3
Visual frontend (OpenVINS) on EuRoC + ATE/RPE vs `robot_localization` and Vicon
truth — the dataset/eval layer on top of this node. See PLAN M3 / §19.1.
