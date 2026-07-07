# fusion_core

ROS-free estimation library — the shared estimation core the fusion nodes
wrap. PLAN §5/§6, **milestone 2** (EKF, numpy only) + **milestone 6** (the
GTSAM factor-graph variant, optional `gtsam` wheel). Zero ROS deps →
unit-testable in milliseconds and a 10-minute read that stands on its own.

## What's here
| File | Purpose |
|---|---|
| `fusion_core/ekf.py` | generic (E)KF: `predict(F, Q)` + `update(z, H, R)`, Joseph-form covariance, NIS + Mahalanobis gating |
| `fusion_core/models.py` | constant-velocity transition `F`, white-noise-acceleration `Q`, position measurement `H`/`R` |
| `fusion_core/factor_graph.py` | **M6**: `PlanarFactorGraph` — GTSAM/ISAM2 pose graph behind the exact `PlanarPoseEstimator` interface (relative hooks as native `BetweenFactorPose2`, GNSS/heading as partial priors). Needs the `gtsam==4.2.1` PyPI wheel (in the fusion image); import is guarded so the EKF path stays numpy-only. A/B vs the EKF: `scripts/m6_ab_benchmark.py` → `results/m6_ab.md` (accuracy parity, EKF 3.5× cheaper on GNSS-heavy streams). |
| `test/` | pytest (18): covariance behaviour, Joseph PSD invariance, convergence, *filter-beats-raw-measurements* RMSE, NIS consistency + factor-graph behavioural twins (frame cancellation, keystone drift→reacquire) |

## Design
- **One core, two wrappers.** `ego_localizer` (pose+velocity+bias) and
  `object_tracker` (per-track position+velocity) both wrap this EKF with their
  own state vectors / measurement models — they do **not** duplicate the algebra.
- **Linear KF or first-order EKF from the same code.** Pass only the Jacobians
  `F`/`H` → linear KF; also pass the non-linear propagated mean `x_pred` /
  predicted measurement `z_pred` → EKF. Same covariance path either way.
- **Numerics:** Joseph-form covariance update (stays symmetric PSD under
  round-off), Kalman gain via `solve` (no explicit inverse), symmetry enforced
  every step.

## Run the tests
```bash
# in the fusion image (numpy/scipy/pytest present), from the repo root:
docker run --rm -v "$PWD/ros2_ws/src/fusion_core":/pkg:ro sensing-node/fusion:local \
  bash -lc 'cd /pkg && PYTHONPATH=/pkg python3 -m pytest test/ -v -p no:cacheprovider'
# or inside a colcon workspace:  colcon test --packages-select fusion_core
```

## Not yet (future milestones)
UKF variant; IMU-bias state. The GTSAM factor-graph variant is DONE (M6,
above); the relative/absolute measurement models live with `ego_localizer`
(M3/M4 — its `PlanarPoseEstimator` is the EKF twin of `PlanarFactorGraph`).
