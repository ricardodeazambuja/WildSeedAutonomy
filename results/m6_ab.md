| scenario | backend | pos RMSE [m] | yaw RMSE [rad] | update mean [µs] | update p95 [µs] |
|---|---|---|---|---|---|
| keystone (odom+IMU+GNSS, denial window) | EKF (hand-rolled) | 0.925 | 0.0039 | 188 | 279 |
| keystone (odom+IMU+GNSS, denial window) | GTSAM ISAM2 (factor graph) | 0.910 | 0.0302 | 662 | 1383 |
| frontend (LIO/VIO deltas + IMU, no GNSS) | EKF (hand-rolled) | 0.194 | 0.0278 | 210 | 302 |
| frontend (LIO/VIO deltas + IMU, no GNSS) | GTSAM ISAM2 (factor graph) | 0.184 | 0.0274 | 221 | 370 |
