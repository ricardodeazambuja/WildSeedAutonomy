"""CLI: compute ATE/RPE for one or more estimated trajectories vs ground truth,
and render the money chart (top-down trajectories + error stats). Headless (Agg).

Trajectory files: TUM (`t x y z qx qy qz qw`) or simple CSV (`x,y,z` or `x,y`),
whitespace- or comma-separated, '#'-comments skipped. Trajectories are compared
index-wise, so they must be the same length / already time-synced (the dataset
publishers and recorders will emit synced TUM in later milestones).

Usage:
  evaluate.py --gt gt.tum --est ego:ego.tum --est odom:odom.tum --out results/m3.png
"""
import argparse
import csv
import os
import sys

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from eval_tools.metrics import ate, rpe  # noqa: E402


def load_trajectory(path: str) -> np.ndarray:
    """Load an Nx3 position array from a TUM or CSV trajectory file."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                continue
            if len(vals) >= 8:          # TUM: t x y z qx qy qz qw
                rows.append(vals[1:4])
            elif len(vals) == 3:        # x y z
                rows.append(vals)
            elif len(vals) == 2:        # x y  -> z=0
                rows.append([vals[0], vals[1], 0.0])
    if not rows:
        raise ValueError(f"no trajectory points parsed from {path}")
    return np.asarray(rows, dtype=float)


def evaluate(gt, ests, out_png, with_scale=False, rpe_delta=10):
    """ests: list of (label, Nx3). Returns list of (label, ate_stats, rpe_stats)."""
    results = []
    fig, (axt, axe) = plt.subplots(1, 2, figsize=(12, 5.5))
    axt.plot(gt[:, 0], gt[:, 1], "k-", lw=2.5, label="ground truth", alpha=0.7)
    bars = []
    for label, est in ests:
        n = min(len(gt), len(est))
        a, aligned = ate(est[:n], gt[:n], with_scale=with_scale)
        r = rpe(est[:n], gt[:n], delta=min(rpe_delta, n - 1))
        results.append((label, a, r))
        axt.plot(aligned[:, 0], aligned[:, 1], "-", lw=1.5,
                 label=f"{label} (ATE {a.rmse:.3f} m)")
        bars.append((label, a.rmse, r.rmse))
    axt.set_aspect("equal"); axt.grid(alpha=0.3); axt.legend()
    axt.set_title("Trajectories (aligned, top-down)")
    axt.set_xlabel("x [m]"); axt.set_ylabel("y [m]")

    labels = [b[0] for b in bars]
    x = np.arange(len(labels))
    axe.bar(x - 0.2, [b[1] for b in bars], 0.4, label="ATE rmse [m]")
    axe.bar(x + 0.2, [b[2] for b in bars], 0.4, label="RPE rmse [m]")
    axe.set_xticks(x); axe.set_xticklabels(labels)
    axe.grid(alpha=0.3, axis="y"); axe.legend(); axe.set_title("Error (lower is better)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110, bbox_inches="tight")
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gt", required=True, help="ground-truth trajectory file")
    ap.add_argument("--est", action="append", default=[], required=True,
                    help="label:path (repeatable)")
    ap.add_argument("--out", required=True, help="output chart PNG")
    ap.add_argument("--scale", action="store_true", help="Sim(3)/monocular align")
    ap.add_argument("--rpe-delta", type=int, default=10)
    args = ap.parse_args(argv)

    gt = load_trajectory(args.gt)
    ests = []
    for spec in args.est:
        label, _, path = spec.partition(":")
        ests.append((label or os.path.basename(path), load_trajectory(path)))

    results = evaluate(gt, ests, args.out, args.scale, args.rpe_delta)
    csv_path = os.path.splitext(args.out)[0] + "_metrics.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trajectory", "ate_rmse", "ate_mean", "ate_max",
                    "rpe_rmse", "rpe_mean"])
        for label, a, r in results:
            w.writerow([label, f"{a.rmse:.6f}", f"{a.mean:.6f}", f"{a.max:.6f}",
                        f"{r.rmse:.6f}", f"{r.mean:.6f}"])
    for label, a, r in results:
        print(f"{label:>12}:  ATE rmse={a.rmse:.4f} m  RPE rmse={r.rmse:.4f} m")
    print(f"wrote {args.out} and {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
