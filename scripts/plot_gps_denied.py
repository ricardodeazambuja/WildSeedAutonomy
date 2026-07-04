#!/usr/bin/env python3
"""Plot the GPS-denied keystone chart (#1) from scripts/gps_denied_demo.py output.

Two panels: (left) top-down ego vs GPS(=truth) trajectory; (right) |ego - GPS|
error over time with the GPS-denied window shaded. Run in the fusion image
(numpy 1.26 + matplotlib). Headless (Agg).

Usage: plot_gps_denied.py <in.csv> <out.png>
"""
import csv
import os
import sys

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    if len(sys.argv) != 3:
        print("usage: plot_gps_denied.py <in.csv> <out.png>", file=sys.stderr)
        return 2
    inp, outp = sys.argv[1], sys.argv[2]
    t, ph, ex, ey, gx, gy, err = [], [], [], [], [], [], []
    with open(inp) as f:
        for r in csv.DictReader(f):
            t.append(float(r["t"])); ph.append(r["phase"])
            ex.append(float(r["ego_x"])); ey.append(float(r["ego_y"]))
            gx.append(float(r["gps_x"])); gy.append(float(r["gps_y"]))
            err.append(float(r["err"]))
    if not t:
        print(f"no rows in {inp}", file=sys.stderr); return 1

    denied = [i for i, p in enumerate(ph) if p == "denied"]
    d0, d1 = (t[denied[0]], t[denied[-1]]) if denied else (None, None)

    def mean_in(label):
        e = [err[i] for i, p in enumerate(ph) if p == label]
        return sum(e) / len(e) if e else float("nan")

    fig, (axt, axe) = plt.subplots(1, 2, figsize=(12, 5.5))
    axt.plot(gx, gy, "k-", lw=2.5, alpha=0.7, label="GPS (truth)")
    axt.plot(ex, ey, "r-", lw=1.5, label="ego_localizer (fused)")
    if denied:
        axt.plot([ex[i] for i in denied], [ey[i] for i in denied], "-",
                 color="orange", lw=2.5, label="ego while GPS denied")
    axt.plot(gx[0], gy[0], "go", ms=9, label="start")
    axt.set_aspect("equal"); axt.grid(alpha=0.3); axt.legend()
    axt.set_title("Top-down: fused pose vs GPS")
    axt.set_xlabel("east [m]"); axt.set_ylabel("north [m]")

    axe.plot(t, err, "b-", lw=1.5)
    if denied:
        axe.axvspan(d0, d1, color="orange", alpha=0.2, label="GPS denied")
    axe.set_title("Position error |ego - GPS| over time")
    axe.set_xlabel("t [s]"); axe.set_ylabel("error [m]")
    axe.grid(alpha=0.3); axe.legend()
    txt = (f"mean |err|   on={mean_in('on'):.2f}  denied={mean_in('denied'):.2f}"
           f"  reacq={mean_in('reacq'):.2f} m")
    axe.text(0.02, 0.96, txt, transform=axe.transAxes, va="top", fontsize=9,
             bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    fig.suptitle("GPS-denied keystone (sim): bounded → drift on outage → reacquire",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(outp, dpi=110, bbox_inches="tight")
    print(f"wrote {outp}  (on={mean_in('on'):.3f} denied={mean_in('denied'):.3f} "
          f"reacq={mean_in('reacq'):.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
