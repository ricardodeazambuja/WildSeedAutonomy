"""A small, dependency-light Extended Kalman Filter (numpy only, no ROS).

This is the shared estimation core (PLAN §5/§6): `ego_localizer` and
`object_tracker` both wrap it with different state vectors and measurement
models. Keeping it ROS-free makes it unit-testable in milliseconds and a
10-minute read.

Conventions
-----------
- State mean `x`        : shape (n,)
- State covariance `P`  : shape (n, n), kept symmetric PSD
- The filter is generic: `predict`/`update` take the *linearised* models
  (Jacobians `F`, `H`) plus, for genuinely non-linear models, the propagated
  mean (`x_pred`) / predicted measurement (`z_pred`). Pass only `F`/`H` and the
  filter behaves as a linear Kalman filter; pass `x_pred`/`z_pred` too and it is
  a first-order EKF. Both paths share the same covariance algebra.

Numerics
--------
- Covariance update uses the **Joseph form**
  `P = (I-KH) P (I-KH)ᵀ + K R Kᵀ`, which stays symmetric PSD under round-off far
  better than the textbook `(I-KH)P`.
- The Kalman gain is obtained via `solve` (not an explicit inverse) on the
  innovation covariance `S`.
- `update` returns the innovation, `S`, and the NIS (normalised innovation
  squared, `yᵀ S⁻¹ y`) so callers can do consistency checks and Mahalanobis
  gating (the tracker, M7).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _symmetrise(P: np.ndarray) -> np.ndarray:
    """Force exact symmetry (kills accumulated round-off asymmetry)."""
    return 0.5 * (P + P.T)


@dataclass
class EKFResult:
    """What an `update` produced — for logging, gating and consistency tests."""
    innovation: np.ndarray   # y = z - h(x)
    innovation_cov: np.ndarray  # S
    nis: float               # yᵀ S⁻¹ y  (E[NIS] = dim(z) for a consistent filter)
    gain: np.ndarray         # K


class EKF:
    """Generic (E)KF over a numpy state. See module docstring."""

    def __init__(self, x0, P0):
        self.x = np.asarray(x0, dtype=float).reshape(-1)
        n = self.x.shape[0]
        self.P = _symmetrise(np.asarray(P0, dtype=float).reshape(n, n))
        self.n = n

    # ── prediction ─────────────────────────────────────────────────────────
    def predict(self, F, Q, x_pred=None):
        """Propagate the state through the (linearised) motion model.

        x⁻ = x_pred if given else F x        (F is the Jacobian either way)
        P⁻ = F P Fᵀ + Q
        """
        F = np.asarray(F, dtype=float)
        Q = np.asarray(Q, dtype=float)
        if F.shape != (self.n, self.n):
            raise ValueError(f"F must be {(self.n, self.n)}, got {F.shape}")
        if Q.shape != (self.n, self.n):
            raise ValueError(f"Q must be {(self.n, self.n)}, got {Q.shape}")

        self.x = (np.asarray(x_pred, dtype=float).reshape(-1)
                  if x_pred is not None else F @ self.x)
        self.P = _symmetrise(F @ self.P @ F.T + Q)
        return self.x

    # ── correction ─────────────────────────────────────────────────────────
    def update(self, z, H, R, z_pred=None) -> EKFResult:
        """Fuse a measurement `z` with measurement model (Jacobian `H`, noise `R`).

        y = z - (z_pred if given else H x)   innovation
        S = H P Hᵀ + R
        K = P Hᵀ S⁻¹                         (via solve)
        x = x + K y
        P = (I-KH) P (I-KH)ᵀ + K R Kᵀ        (Joseph form)
        """
        z = np.asarray(z, dtype=float).reshape(-1)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)
        m = z.shape[0]
        if H.shape != (m, self.n):
            raise ValueError(f"H must be {(m, self.n)}, got {H.shape}")
        if R.shape != (m, m):
            raise ValueError(f"R must be {(m, m)}, got {R.shape}")

        z_hat = (np.asarray(z_pred, dtype=float).reshape(-1)
                 if z_pred is not None else H @ self.x)
        y = z - z_hat
        PHt = self.P @ H.T
        S = _symmetrise(H @ PHt + R)
        # K = PHt S⁻¹  ⇔  Sᵀ Kᵀ = PHtᵀ  → solve instead of inverting S.
        K = np.linalg.solve(S.T, PHt.T).T

        self.x = self.x + K @ y
        IKH = np.eye(self.n) - K @ H
        self.P = _symmetrise(IKH @ self.P @ IKH.T + K @ R @ K.T)

        nis = float(y @ np.linalg.solve(S, y))
        return EKFResult(innovation=y, innovation_cov=S, nis=nis, gain=K)

    # ── helpers ──────────────────────────────────────────────────────────────
    def mahalanobis2(self, z, H, R, z_pred=None) -> float:
        """Squared Mahalanobis distance of `z` to the predicted measurement.

        Same quantity as `update`'s NIS but WITHOUT mutating the filter — for
        gating a detection before deciding to associate it (tracker, M7).
        """
        z = np.asarray(z, dtype=float).reshape(-1)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)
        z_hat = (np.asarray(z_pred, dtype=float).reshape(-1)
                 if z_pred is not None else H @ self.x)
        y = z - z_hat
        S = _symmetrise(H @ self.P @ H.T + R)
        return float(y @ np.linalg.solve(S, y))
