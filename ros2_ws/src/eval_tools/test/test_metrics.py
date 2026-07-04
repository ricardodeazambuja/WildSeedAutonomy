"""Deterministic tests for ATE/RPE metrics (synthetic trajectories, known truth)."""
import numpy as np

from eval_tools.metrics import apply_sim3, ate, rpe, umeyama


def _traj(n=400):
    """A smooth 3D path: planar arc with a little climb."""
    t = np.linspace(0, 4 * np.pi, n)
    return np.stack([np.cos(t) * (1 + 0.1 * t),
                     np.sin(t) * (1 + 0.1 * t),
                     0.05 * t], axis=1)


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_umeyama_recovers_known_rigid_transform():
    gt = _traj()
    R0, t0 = _rot_z(0.7), np.array([3.0, -2.0, 1.0])
    est = apply_sim3(gt, R0, t0, 1.0)               # est = R0 gt + t0
    R, t, s = umeyama(est, gt, with_scale=False)     # should invert it
    aligned = apply_sim3(est, R, t, s)
    assert np.allclose(aligned, gt, atol=1e-9)
    assert abs(s - 1.0) < 1e-9


def test_ate_zero_for_rigidly_offset_trajectory():
    gt = _traj()
    est = apply_sim3(gt, _rot_z(-1.2), np.array([10.0, 5.0, -3.0]), 1.0)
    stats, _ = ate(est, gt)
    assert stats.rmse < 1e-9            # alignment removes a pure rigid offset


def test_ate_recovers_gaussian_noise_level():
    rng = np.random.default_rng(1)
    gt = _traj(2000)
    sigma = 0.05
    est = gt + rng.normal(scale=sigma, size=gt.shape)
    stats, _ = ate(est, gt)
    # per-point 3D distance RMSE ≈ sqrt(3)*sigma; alignment trims it slightly
    expected = np.sqrt(3) * sigma
    assert 0.7 * expected < stats.rmse < 1.15 * expected, (stats.rmse, expected)


def test_scale_only_error_needs_sim3():
    gt = _traj()
    est = gt * 0.5                       # monocular up-to-scale
    rigid, _ = ate(est, gt, with_scale=False)
    sim3, _ = ate(est, gt, with_scale=True)
    assert sim3.rmse < 1e-9             # scale absorbed
    assert rigid.rmse > 0.5            # rigid cannot fix a scale error (≈0.84 m here)


def test_rpe_zero_for_identical_trajectory():
    gt = _traj()
    assert rpe(gt, gt, delta=1).rmse < 1e-9


def test_rpe_detects_local_drift_but_ignores_global_offset():
    gt = _traj()
    # global rigid offset → RPE ~0 (it's relative)
    offset = apply_sim3(gt, _rot_z(0.9), np.array([4.0, 4.0, 0.0]), 1.0)
    assert rpe(offset, gt, delta=5).rmse < 1e-9
    # growing scale drift → RPE > 0
    s = 1.0 + 0.001 * np.arange(len(gt))[:, None]
    drift = gt * s
    assert rpe(drift, gt, delta=5).rmse > 1e-3
