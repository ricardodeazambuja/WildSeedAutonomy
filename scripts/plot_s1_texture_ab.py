#!/usr/bin/env python3
"""S1 texture-A/B chart — same seed/layout/route, only the ground compositor
changes (--texture 0.0 uniform vs 1.0 patchy).

Left: RPE per estimator per seed, uniform-vs-patchy paired bars (RPE, not ATE —
the slow-UGV under-report bias integrates path-dependently into ATE; m4-lio.md
war story #5). Right: the mechanism check — KLT forward-backward survival
logged along the drive (s1_corner_log.py). Measured outcome (2026-07-09): a
controlled NEGATIVE — corners AND survival stay high on both variants, VIO
unimpaired; with corridor scatter in view the ground compositor is a minor
part of the feature diet. The M4 alpine divergence needed route-wide
starvation (sparse scatter AND weak ground), which this dial alone doesn't
reproduce.

Usage (inside the fusion image — host conda has the numpy2/matplotlib clash):
  plot_s1_texture_ab.py out.png SEED:TEX:metrics.csv:corners.csv [...]
where TEX is 'uniform' or 'patchy'.
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

# estimator hues (CVD-validated set); uniform variant = tint + hatch, so
# texture is never encoded by color alone
EST = [("openvins_raw", "OpenVINS raw (stereo VIO)", "#1baf7a"),
       ("kiss_raw", "KISS-ICP raw (lidar-only)", "#eda100"),
       ("ego_localizer", "fused ego_localizer (LIO+IMU)", "#2a78d6")]
TEX_STYLE = {"uniform": dict(alpha=0.45, hatch="//"), "patchy": dict(alpha=1.0)}


def tint(hexcolor, alpha):
    r, g, b = (int(hexcolor[i:i + 2], 16) / 255 for i in (1, 3, 5))
    return (1 - alpha * (1 - r), 1 - alpha * (1 - g), 1 - alpha * (1 - b))


def main():
    out = sys.argv[1]
    runs = []                     # (seed, tex, {traj: (ate, rpe)}, corners)
    for arg in sys.argv[2:]:
        seed, tex, mpath, cpath = arg.split(":", 3)
        m = {}
        with open(mpath) as f:
            for row in csv.DictReader(f):
                m[row["trajectory"]] = (float(row["ate_rmse"]),
                                        float(row["rpe_rmse"]))
        c = np.genfromtxt(cpath, delimiter=",", names=True) if cpath else None
        runs.append((seed, tex, m, c))

    seeds = sorted({r[0] for r in runs}, key=int)
    by_key = {(r[0], r[1]): r for r in runs}

    fig, (axb, axc) = plt.subplots(1, 2, figsize=(13, 4.6),
                                   gridspec_kw={"width_ratios": [1.15, 1]})

    # ── left: paired RPE bars ────────────────────────────────────────────
    x = np.arange(len(seeds))
    nbars = len(EST) * 2
    w = 0.8 / nbars
    for i, (key, _, color) in enumerate(EST):
        for j, tex in enumerate(("uniform", "patchy")):
            vals = [by_key[(s, tex)][2].get(key, (np.nan, np.nan))[1]
                    if (s, tex) in by_key else np.nan for s in seeds]
            off = (i * 2 + j - (nbars - 1) / 2) * w
            style = TEX_STYLE[tex]
            bars = axb.bar(x + off, vals, w * 0.92,
                           color=tint(color, style["alpha"]),
                           hatch=style.get("hatch", ""),
                           edgecolor=color, linewidth=0.8)
            for b, v in zip(bars, vals):
                if np.isfinite(v):
                    axb.annotate(f"{v:.3g}",
                                 (b.get_x() + b.get_width() / 2, v),
                                 ha="center", va="bottom", fontsize=6.5)
    axb.set_yscale("log")
    all_rpe = [m.get(k, (np.nan, np.nan))[1] for _, _, m, _ in runs
               for k, _, _ in EST]
    axb.set_ylim(top=np.nanmax(all_rpe) * 2.2)
    axb.set_xticks(x, [f"seed {s}" for s in seeds])
    axb.set_ylabel("RPE rmse [m]  (log)")
    axb.set_title("estimator local error — geometry/route held constant")
    axb.grid(axis="y", alpha=0.3, which="both")
    axb.legend(handles=[Patch(facecolor=c, label=n) for _, n, c in EST] +
               [Patch(facecolor=tint("#666666", 0.45), hatch="//",
                      edgecolor="#666666", label="texture 0.0 (uniform)"),
                Patch(facecolor="#666666", label="texture 1.0 (patchy)")],
               fontsize=7, loc="upper left", ncol=1)

    # ── right: KLT forward-backward survival along the drive ─────────────
    # (corner COUNT does not discriminate here — the uniform compositor is an
    # ALIASING worst case: plenty of corners, ambiguity is the failure mode;
    # measured ~190 mean Shi-Tomasi corners on both variants)
    TEXC = {"uniform": ("#e34948", "--"), "patchy": ("#008300", "-")}
    mean_corners = []
    for seed in seeds:
        for tex in ("uniform", "patchy"):
            r = by_key.get((seed, tex))
            if r is None or r[3] is None or r[3].size == 0:
                continue
            c = np.atleast_1d(r[3])
            if "klt_surv" not in (c.dtype.names or ()):
                continue
            ok = np.isfinite(c["klt_surv"])
            t = c["t_sim"][ok] - c["t_sim"][0]
            color, ls = TEXC[tex]
            axc.plot(t, c["klt_surv"][ok], ls, color=color, linewidth=1.4,
                     alpha=0.85)
            axc.annotate(f"s{seed}", (t[-1], c["klt_surv"][ok][-1]),
                         fontsize=6.5, color=color,
                         xytext=(3, 0), textcoords="offset points")
            mean_corners.append(np.nanmean(c["n_shi"]))
    axc.set_ylim(0, 1.02)
    axc.set_xlabel("drive time [sim s]")
    axc.set_ylabel("KLT fwd-bwd survival (camera_0, consecutive frames)")
    axc.set_title("the mechanism — corner trackability along the route")
    if mean_corners:
        axc.annotate("corners stay plentiful (mean "
                     f"{np.mean(mean_corners):.0f}/frame) AND survival stays\n"
                     "high on both variants — the forward stereo view feeds\n"
                     "on the 3D scatter, not the ground",
                     (0.02, 0.05), xycoords="axes fraction", fontsize=6.5,
                     color="#555555")
    xmax = axc.get_xlim()[1]
    axc.set_xlim(right=xmax * 1.08)   # room for the seed line-end labels
    axc.grid(alpha=0.3)
    axc.legend(handles=[
        plt.Line2D([], [], color="#e34948", ls="--",
                   label="texture 0.0 (uniform)"),
        plt.Line2D([], [], color="#008300", ls="-",
                   label="texture 1.0 (patchy)")], fontsize=7,
        loc="upper right")

    fig.suptitle("S1 — ground-texture A/B at fixed geometry: a controlled "
                 "negative — no VIO degradation while 3D scatter lines the route",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
