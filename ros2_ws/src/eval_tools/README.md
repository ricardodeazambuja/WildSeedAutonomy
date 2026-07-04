# eval_tools

ROS-free trajectory evaluation — the backbone for the milestone "money charts"
(PLAN §6). Computes the standard accuracy metrics and renders the comparison
chart used to judge every frontend and the fusion core against ground truth
(Vicon on EuRoC M3, RTK-GNSS on MARS-LVIG M5, …).

## What's here
| File | Purpose |
|---|---|
| `eval_tools/metrics.py` | `ate` (Absolute Trajectory Error, Umeyama-aligned), `rpe` (Relative Pose Error / local drift), `umeyama` similarity alignment. numpy only. |
| `eval_tools/evaluate.py` | CLI: load TUM/CSV trajectories, compute ATE/RPE for N estimates vs ground truth, render the chart + `*_metrics.csv`. |
| `test/` | pytest: alignment recovers a known transform, ATE≈0 for a rigid offset, ATE≈noise level, Sim(3) absorbs scale, RPE ignores global offset but catches drift. |

## Use
```bash
evaluate --gt gt.tum --est ego:ego.tum --est odom:odom.tum --out results/m3.png
# prints ATE/RPE per trajectory; writes the chart + results/m3_metrics.csv
```

## Verified
6 pytest green; CLI exercised end-to-end on synthetic trajectories → ego ATE
0.051 m vs a drifting odom 0.455 m, chart + CSV produced (left: aligned
top-down trajectories; right: ATE/RPE bars). This is the format the M3/M4/M5
charts will use with real data.
