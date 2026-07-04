"""Trajectory error metrics — ATE / RPE with alignment (numpy only, no ROS).

The evaluation backbone for the "money charts" (PLAN §6 eval_tools): given an
estimated trajectory and a ground-truth trajectory (Nx3 positions, time-aligned),
compute the standard SLAM/odometry metrics used to compare frontends and the
fusion core against truth (Vicon on EuRoC M3, RTK-GNSS on MARS-LVIG M5).

- **ATE** (Absolute Trajectory Error): RMSE of position after a least-squares
  rigid (or similarity) alignment of est→gt — the headline accuracy number.
- **RPE** (Relative Pose Error): RMSE of the *relative* displacement error over a
  fixed step `delta` — local drift, insensitive to a global offset.

Alignment uses Umeyama (1991): the closed-form least-squares similarity transform.
`with_scale=False` (rigid SE(3)) is the default for metric sensors; `True`
(Sim(3)) is for monocular up-to-scale trajectories.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = False):
    """Least-squares similarity transform mapping src→dst (both Nx3).

    Returns (R (3x3), t (3,), s (float)) minimising ||dst - (s R src + t)||².
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.shape[1] != 3:
        raise ValueError("src and dst must both be Nx3 with equal N")
    n = src.shape[0]
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    Sigma = (dc.T @ sc) / n
    U, D, Vt = np.linalg.svd(Sigma)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1.0
    R = U @ S @ Vt
    if with_scale:
        var_s = (sc ** 2).sum() / n
        s = float(np.trace(np.diag(D) @ S) / var_s)
    else:
        s = 1.0
    t = mu_d - s * R @ mu_s
    return R, t, s


def apply_sim3(traj: np.ndarray, R, t, s) -> np.ndarray:
    return (s * (R @ np.asarray(traj, dtype=float).T)).T + t


@dataclass
class ErrorStats:
    rmse: float
    mean: float
    median: float
    std: float
    max: float

    @staticmethod
    def from_errors(e: np.ndarray) -> "ErrorStats":
        e = np.asarray(e, dtype=float)
        return ErrorStats(
            rmse=float(np.sqrt(np.mean(e ** 2))),
            mean=float(np.mean(e)),
            median=float(np.median(e)),
            std=float(np.std(e)),
            max=float(np.max(e)),
        )


def ate(est: np.ndarray, gt: np.ndarray, with_scale: bool = False):
    """Absolute Trajectory Error after Umeyama alignment of est→gt.

    Returns (ErrorStats, aligned_est). `rmse` is the headline ATE.
    """
    est = np.asarray(est, dtype=float)
    gt = np.asarray(gt, dtype=float)
    R, t, s = umeyama(est, gt, with_scale=with_scale)
    aligned = apply_sim3(est, R, t, s)
    err = np.linalg.norm(aligned - gt, axis=1)
    return ErrorStats.from_errors(err), aligned


def rpe(est: np.ndarray, gt: np.ndarray, delta: int = 1):
    """Relative Pose Error (translational) over a fixed step `delta`.

    Compares relative displacement vectors est[i+δ]-est[i] vs gt[i+δ]-gt[i] in
    the gt frame (after rigid alignment of the displacement clouds), so a global
    offset doesn't count — only local drift. Returns ErrorStats.
    """
    est = np.asarray(est, dtype=float)
    gt = np.asarray(gt, dtype=float)
    if delta < 1 or delta >= len(est):
        raise ValueError("delta must be in [1, N-1]")
    d_est = est[delta:] - est[:-delta]
    d_gt = gt[delta:] - gt[:-delta]
    # align the displacement directions (rotation only) so a frame offset between
    # est and gt doesn't inflate RPE.
    R, _, _ = umeyama(d_est, d_gt, with_scale=False)
    err = np.linalg.norm((R @ d_est.T).T - d_gt, axis=1)
    return ErrorStats.from_errors(err)
