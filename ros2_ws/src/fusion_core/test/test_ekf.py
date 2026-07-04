"""Unit tests for the EKF core. Pure numpy, deterministic (seeded RNG)."""
import numpy as np
import pytest

from fusion_core.ekf import EKF
from fusion_core.models import (cv_process_noise, cv_transition,
                                measurement_noise, position_measurement)


def _is_symmetric(P, tol=1e-12):
    return np.allclose(P, P.T, atol=tol)


def _is_psd(P, tol=1e-9):
    return np.all(np.linalg.eigvalsh(0.5 * (P + P.T)) > -tol)


def test_predict_propagates_mean_and_grows_covariance():
    dim = 2
    F = cv_transition(dt=1.0, dim=dim)
    Q = cv_process_noise(dt=1.0, sigma_a=1.0, dim=dim)
    # start at origin moving +x at 2 m/s, +y at -1 m/s
    f = EKF(x0=[0, 0, 2, -1], P0=np.eye(4) * 0.1)
    tr0 = np.trace(f.P)
    f.predict(F, Q)
    # CV: position advances by velocity * dt
    assert np.allclose(f.x, [2, -1, 2, -1])
    # no measurement → uncertainty must grow
    assert np.trace(f.P) > tr0
    assert _is_symmetric(f.P) and _is_psd(f.P)


def test_update_reduces_covariance_and_pulls_toward_measurement():
    dim = 2
    H = position_measurement(dim)
    R = measurement_noise(sigma_z=0.5, dim=dim)
    f = EKF(x0=[0, 0, 0, 0], P0=np.eye(4) * 10.0)
    tr_before = np.trace(f.P)
    res = f.update(z=[1.0, 2.0], H=H, R=R)
    assert np.trace(f.P) < tr_before                 # measurement informs us
    # with a large prior and a position measurement, estimate jumps most of the way
    assert f.x[0] > 0.5 and f.x[1] > 1.0
    assert res.nis >= 0.0
    assert _is_symmetric(f.P) and _is_psd(f.P)


def test_joseph_form_keeps_covariance_symmetric_psd_over_many_updates():
    dim = 3
    H = position_measurement(dim)
    R = measurement_noise(sigma_z=0.2, dim=dim)
    F = cv_transition(dt=0.1, dim=dim)
    Q = cv_process_noise(dt=0.1, sigma_a=2.0, dim=dim)
    rng = np.random.default_rng(0)
    f = EKF(x0=np.zeros(2 * dim), P0=np.eye(2 * dim))
    for _ in range(500):
        f.predict(F, Q)
        z = rng.normal(size=dim)
        f.update(z, H, R)
        assert _is_symmetric(f.P), "Joseph form must preserve symmetry"
        assert _is_psd(f.P), "covariance must stay positive semi-definite"


def test_static_state_converges_to_truth():
    """Estimating a constant 1-D position from noisy fixes → converges, P shrinks."""
    truth = 5.0
    H = np.array([[1.0]])
    R = np.array([[0.4 ** 2]])
    f = EKF(x0=[0.0], P0=[[100.0]])
    rng = np.random.default_rng(42)
    for _ in range(300):
        z = truth + rng.normal(scale=0.4, size=1)
        f.update(z, H, R)
    assert abs(f.x[0] - truth) < 0.1
    assert f.P[0, 0] < 0.01            # confident after many fixes


def test_filter_beats_raw_measurements_on_cv_trajectory():
    """The whole point: fused estimate RMSE < raw-measurement RMSE."""
    dim = 2
    dt = 0.1
    sigma_a = 0.3        # true process agitation
    sigma_z = 0.8        # noisy position sensor
    F = cv_transition(dt, dim)
    Q = cv_process_noise(dt, sigma_a, dim)
    H = position_measurement(dim)
    R = measurement_noise(sigma_z, dim)
    rng = np.random.default_rng(7)

    x_true = np.array([0.0, 0.0, 1.0, 0.5])     # start moving
    f = EKF(x0=[0, 0, 0, 0], P0=np.eye(4) * 5.0)

    est_err2, meas_err2, n = 0.0, 0.0, 0
    for _ in range(400):
        # propagate ground truth with real process noise
        accel = rng.normal(scale=sigma_a, size=dim)
        x_true = F @ x_true
        x_true[dim:] += accel * dt
        x_true[:dim] += 0.5 * accel * dt ** 2
        # noisy position measurement
        z = x_true[:dim] + rng.normal(scale=sigma_z, size=dim)

        f.predict(F, Q)
        f.update(z, H, R)

        est_err2 += np.sum((f.x[:dim] - x_true[:dim]) ** 2)
        meas_err2 += np.sum((z - x_true[:dim]) ** 2)
        n += 1

    rmse_est = np.sqrt(est_err2 / n)
    rmse_meas = np.sqrt(meas_err2 / n)
    assert rmse_est < rmse_meas, (rmse_est, rmse_meas)
    # should be a clear win, not marginal
    assert rmse_est < 0.7 * rmse_meas


def test_nis_consistency_matches_measurement_dimension():
    """A correctly-tuned filter is consistent: mean NIS ≈ dim(z)."""
    dim = 2
    dt = 0.1
    sigma_a = 0.5
    sigma_z = 0.5
    F = cv_transition(dt, dim)
    Q = cv_process_noise(dt, sigma_a, dim)
    H = position_measurement(dim)
    R = measurement_noise(sigma_z, dim)
    rng = np.random.default_rng(123)

    x_true = np.array([0.0, 0.0, 1.0, -0.5])
    f = EKF(x0=[0, 0, 1, -0.5], P0=np.eye(4) * 0.5)

    nis_vals = []
    for _ in range(2000):
        accel = rng.normal(scale=sigma_a, size=dim)
        x_true = F @ x_true
        x_true[dim:] += accel * dt
        x_true[:dim] += 0.5 * accel * dt ** 2
        z = x_true[:dim] + rng.normal(scale=sigma_z, size=dim)
        f.predict(F, Q)
        res = f.update(z, H, R)
        nis_vals.append(res.nis)

    mean_nis = float(np.mean(nis_vals))
    # E[NIS] = dim(z) = 2; allow a generous band (sampling + EKF linearisation)
    assert 1.5 < mean_nis < 2.6, mean_nis


def test_mahalanobis_does_not_mutate_filter():
    dim = 2
    H = position_measurement(dim)
    R = measurement_noise(0.3, dim)
    f = EKF(x0=[1.0, 2.0, 0, 0], P0=np.eye(4))
    x0, P0 = f.x.copy(), f.P.copy()
    d2 = f.mahalanobis2(z=[1.1, 1.9], H=H, R=R)
    assert d2 >= 0.0
    assert np.array_equal(f.x, x0) and np.array_equal(f.P, P0)


def test_shape_validation():
    f = EKF(x0=[0, 0], P0=np.eye(2))
    with pytest.raises(ValueError):
        f.predict(F=np.eye(3), Q=np.eye(2))
    with pytest.raises(ValueError):
        f.update(z=[0], H=np.zeros((1, 3)), R=np.eye(1))
