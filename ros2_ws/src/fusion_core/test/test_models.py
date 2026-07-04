"""Unit tests for the CV motion/measurement models."""
import numpy as np

from fusion_core.models import (cv_process_noise, cv_transition,
                                measurement_noise, position_measurement)


def test_cv_transition_shape_and_values():
    F = cv_transition(dt=0.5, dim=2)
    assert F.shape == (4, 4)
    expected = np.array([[1, 0, 0.5, 0],
                         [0, 1, 0, 0.5],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]], dtype=float)
    assert np.allclose(F, expected)


def test_cv_transition_zero_dt_is_identity():
    assert np.allclose(cv_transition(dt=0.0, dim=3), np.eye(6))


def test_cv_process_noise_symmetric_psd_and_known_block():
    dt, sigma_a, dim = 0.2, 1.5, 1
    Q = cv_process_noise(dt, sigma_a, dim)
    assert Q.shape == (2, 2)
    assert np.allclose(Q, Q.T)
    assert np.all(np.linalg.eigvalsh(Q) >= -1e-12)
    s2 = sigma_a ** 2
    expected = s2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                              [dt ** 3 / 2, dt ** 2]])
    assert np.allclose(Q, expected)


def test_cv_process_noise_scales_with_sigma_squared():
    q1 = cv_process_noise(0.1, 1.0, 2)
    q2 = cv_process_noise(0.1, 2.0, 2)
    assert np.allclose(q2, 4.0 * q1)        # quadratic in sigma_a


def test_position_measurement_selects_position():
    H = position_measurement(dim=3)
    assert H.shape == (3, 6)
    x = np.array([1, 2, 3, 9, 9, 9], dtype=float)   # pos=(1,2,3), vel=(9,9,9)
    assert np.allclose(H @ x, [1, 2, 3])


def test_measurement_noise_isotropic():
    R = measurement_noise(sigma_z=0.5, dim=2)
    assert np.allclose(R, np.diag([0.25, 0.25]))
