"""Reusable motion / measurement models for the EKF (numpy only, no ROS).

The first concrete model is **constant velocity (CV)** with a white-noise
acceleration process — the workhorse for both `ego_localizer` (pose+velocity
prediction between odometry corrections) and `object_tracker` (per-track
position+velocity). More models (constant-acceleration, unicycle) land with the
nodes that need them.

State layout for the CV model (dim = spatial dimensions, 1/2/3):
    x = [ p_0 … p_{dim-1},  v_0 … v_{dim-1} ]      (positions then velocities)
so the Jacobians are clean 2×2 block matrices of dim×dim identities.
"""
from __future__ import annotations

import numpy as np


def cv_transition(dt: float, dim: int = 2) -> np.ndarray:
    """Constant-velocity state-transition (Jacobian) F.

    F = [[I, dt·I],
         [0,    I]]      shape (2·dim, 2·dim)
    """
    I = np.eye(dim)
    Z = np.zeros((dim, dim))
    return np.block([[I, dt * I],
                     [Z, I]])


def cv_process_noise(dt: float, sigma_a: float, dim: int = 2) -> np.ndarray:
    """Discrete white-noise-acceleration process noise Q for the CV model.

    Per axis the canonical 2×2 block is
        sigma_a² · [[dt⁴/4, dt³/2],
                    [dt³/2, dt²  ]]
    assembled here as dim×dim identity blocks. `sigma_a` is the std-dev of the
    unmodelled acceleration (m/s²) — the single tuning knob.
    """
    I = np.eye(dim)
    q11 = (dt ** 4) / 4.0
    q12 = (dt ** 3) / 2.0
    q22 = (dt ** 2)
    return (sigma_a ** 2) * np.block([[q11 * I, q12 * I],
                                      [q12 * I, q22 * I]])


def position_measurement(dim: int = 2) -> np.ndarray:
    """Measurement Jacobian H selecting position out of a CV state.

    H = [I, 0]      shape (dim, 2·dim)
    Models an absolute position fix (e.g. GNSS, or a tracked detection's xy).
    """
    return np.block([np.eye(dim), np.zeros((dim, dim))])


def measurement_noise(sigma_z: float, dim: int = 2) -> np.ndarray:
    """Isotropic measurement-noise covariance R = sigma_z²·I (shape dim×dim)."""
    return (sigma_z ** 2) * np.eye(dim)
