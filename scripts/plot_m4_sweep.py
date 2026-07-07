#!/usr/bin/env python3
"""Terrain-complexity sweep chart — VIO vs LIO vs fused across worlds.

Aggregates the per-world eval_tools metrics CSVs (trajectory,ate_rmse,...)
into one grouped-bar chart: ATE and RPE per estimator per world. Log scale —
the spread (VIO ~0.006 m RPE vs LIO ~4 m ATE) is the finding.

Usage (inside the fusion image — host conda has the numpy2/matplotlib clash):
  plot_m4_sweep.py out.png LABEL:metrics.csv [LABEL:metrics.csv ...]
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

EST = [("kiss_raw", "KISS-ICP raw (lidar-only)", "#8172b3"),
       ("openvins_raw", "OpenVINS raw (stereo VIO)", "#55a868"),
       ("ego_localizer", "fused ego_localizer (LIO+IMU)", "#4c72b0")]


def main():
    out = sys.argv[1]
    worlds = []          # (label, {traj: (ate, rpe)})
    for arg in sys.argv[2:]:
        label, path = arg.split(":", 1)
        m = {}
        with open(path) as f:
            for row in csv.DictReader(f):
                m[row["trajectory"]] = (float(row["ate_rmse"]), float(row["rpe_rmse"]))
        worlds.append((label, m))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), sharey=False)
    x = np.arange(len(worlds))
    w = 0.26
    for ax, metric, title in ((axes[0], 0, "ATE rmse [m]"), (axes[1], 1, "RPE rmse [m]")):
        for i, (key, name, color) in enumerate(EST):
            vals = [wm[1].get(key, (np.nan, np.nan))[metric] for wm in worlds]
            bars = ax.bar(x + (i - 1) * w, vals, w, label=name, color=color)
            for b, v in zip(bars, vals):
                if np.isfinite(v):
                    ax.annotate(f"{v:.3g}", (b.get_x() + b.get_width() / 2, v),
                                ha="center", va="bottom", fontsize=7, rotation=0)
        ax.set_yscale("log")
        ax.set_xticks(x, [wl for wl, _ in worlds])
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.3, which="both")
    axes[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("M4/M3 frontends across terrain complexity — same drive, same spine "
                 "(log scale)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
