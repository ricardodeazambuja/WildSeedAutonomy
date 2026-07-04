#!/usr/bin/env python3
"""Plot an odom trajectory CSV (x,y[,z] per line) to a PNG. Headless (Agg).

Usage: plot_trajectory.py <in.csv> <out.png>
Reads the output of `ros2 topic echo <odom> --field pose.pose.position --csv`.
Runs in the fusion image (matplotlib). Used by scripts/demo_n1_teleop.sh (N1).
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
        print("usage: plot_trajectory.py <in.csv> <out.png>", file=sys.stderr)
        return 2
    inp, outp = sys.argv[1], sys.argv[2]
    xs, ys = [], []
    with open(inp) as f:
        for row in csv.reader(f):
            if len(row) < 2:
                continue
            try:
                xs.append(float(row[0]))
                ys.append(float(row[1]))
            except ValueError:
                continue  # skip a header / partial line
    if not xs:
        print(f"no points parsed from {inp}", file=sys.stderr)
        return 1

    plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, "-", lw=2, color="#1f77b4")
    plt.plot(xs[0], ys[0], "o", color="green", ms=9, label="start")
    plt.plot(xs[-1], ys[-1], "s", color="red", ms=9, label="end")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title("N1 teleop drive — odom trajectory (pipeline world)")
    plt.xlabel("x [m]  (odom frame)")
    plt.ylabel("y [m]  (odom frame)")
    plt.savefig(outp, dpi=110, bbox_inches="tight")
    print(f"wrote {outp} ({len(xs)} pts, "
          f"x {min(xs):.2f}..{max(xs):.2f}, y {min(ys):.2f}..{max(ys):.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
