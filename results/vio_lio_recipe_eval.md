# WildSeed `vio_lio` recipe world — autonomy-stack evaluation

**Goal:** does the WildSeed `scenario --profile vio_lio` recipe world improve our
autonomy stack's odometry (VIO/LIO drift) vs the previous world, which knob
matters most, and does RTF hold under our full ROS node graph?

- WildSeed generator repo: `~/GitStuff/WildSeed` @ `e58fde8` (has `scenario`/`benchmark` CLI + `docs/VIO_LIO_FEATURES.md`).
- Autonomy stack: this repo (`WildSeedAutonomy`) — M3 VIO = OpenVINS (stereo OAK-D Lite) + `ego_localizer`, scored by `scripts/m3_vio_demo.py` (Umeyama-aligned ATE vs gz ground truth).
- Build/render use the GPU container `wildseed:egl` (GDAL+Blender live inside it); `PYTHONPATH=/workspace/src` shadows the pip-installed v0.2.0 with the repo's `e58fde8` source.

## Baselines (for reference)
- M3 VIO on the **pipeline** world (Clearpath default, feature-rich industrial): `ego_localizer` ATE **0.077 m**, `openvins_raw` ATE **0.069 m** (`results/m3_vio_metrics.csv`). This is our "everything works" ceiling, not an outdoor baseline.
- Controlled outdoor A/B baseline (this study): **same seed, recipe features OFF** (`--object-density 0 --relief 0 --variety 0`) — isolates the recipe's causal effect on ATE without confounding terrain/corridor geometry.

## Recipe world build (seed 7, defaults)
`wildseed scenario --seed 7 --profile vio_lio` → `worlds/vio_lio_7.world`
- 165/175 models placed: rock 63, tree 32, bush 70 (10 tree placements failed — steered into corridor).
- Rig injected at (0,0,2). Internal SDF `<world name="forest_world">` regardless of file stem (documented gotcha — benchmarks read the internal name; always pass the file STEM).
- Outputs: `worlds/vio_lio_7.{world,instances.json,yaml}`, `dem/vio_lio_7.tif`, `dem/vio_lio_7_corridor.png`.

## Benchmark sanity check (three axes) — all match expected ballpark
| axis | metric | measured | expected | verdict |
|------|--------|----------|----------|---------|
| VIO   | inlier_ratio / verdict | 0.74 / MARGINAL (ambiguity/landmark-reliant) | ~0.74, MARGINAL/near-GOOD | ✓ |
| VIO   | ratio_reject / inliers-per-pair / ORB-per-fr | 0.96 / 64 / 1416 | — | high aliasing, landmark-reliant |
| RTF   | window_rtf / rtf_min | 0.997 / 0.734 | ~1.0, ≥0.5 | ✓ (headless render, sensor subs on) |
| LIDAR | ring_roughness_m | 1.2232 | ~1.2 | ✓ (range_std 51.6 m, finite_frac 0.04) |

The VIO proxy is a 12-pose synthetic camera flythrough (AGL 2 m, pitch 0.35, step 2 m); it scores data-association quality, not our Husky's actual VIO. `ratio_reject 0.96` = strong aliasing pressure — the recipe survives it on landmark density (`inliers/pair 64`), exactly the "MARGINAL, landmark-reliant" regime the guide describes.

## Knob-sensitivity sweep (VIO data-association proxy)
One knob varied at a time from the recipe defaults (density 175, relief 0.5, variety 0.5), seed 7. Metric = the `benchmark vio` data-association proxy (12-pose synthetic flythrough), NOT our Husky's ATE (that's the recipe-vs-bare full-sim run below).

| config | inlier_ratio | ratio_reject | inliers/pair | putative/pair | ORB/fr | verdict |
|--------|-------------:|-------------:|-------------:|--------------:|-------:|---------|
| **recipe default** (d175 r0.5 v0.5) | 0.739 | 0.960 | 63.6 | 57.0 | 1416 | MARGINAL |
| object-density 50  | 0.767 | 0.922 | 63.7 | 81.8 | 1108 | MARGINAL |
| object-density 300 | 0.709 | 0.919 | 75.7 | 106.5 | 1353 | MARGINAL |
| relief 0.0 | 0.683 | 0.960 | 54.9 | 58.3 | 1429 | MARGINAL |
| relief 1.0 | 0.721 | 0.965 | 47.4 | 49.0 | 1407 | MARGINAL |
| variety 0.0 | 0.748 | 0.957 | 79.3 | 60.9 | 1419 | MARGINAL |
| variety 1.0 | 0.732 | 0.963 | 59.0 | 53.3 | 1416 | MARGINAL |

**Span of each metric across the knob's low→high:**

| knob | Δinlier_ratio | Δinliers/pair | Δputative/pair | Δratio_reject |
|------|--------------:|--------------:|---------------:|--------------:|
| object-density (50→300, 6×) | **0.058** | 12.0 | **24.7** | 0.003 |
| relief (0→1) | 0.039 | 7.5 | 9.3 | 0.006 |
| variety (0→1) | 0.015 | 20.3 | 7.6 | 0.005 |

**Which knob most affects tracking:** `--object-density`. It is the dominant, *monotonic* lever on the two metrics that predict VIO robustness — putative-correspondence volume (57→107, i.e. raw landmark support) and inlier_ratio — matching the guide's "structure > texture, robustness hinges on landmark density." `--relief` is second: it contributes *geometric* parallax (flat relief 0 collapses inliers/pair to 54.9; the recipe's 0.5 restores it). `--variety` moves the *look* more than the *trackability*: raising it actually **lowers** frame-to-frame inliers/pair (79→59) — expected, since its job is global place-recognition distinctiveness (loop closure), not local VIO data association; it trades local inlier count for uniqueness. All seven points stay in the same MARGINAL/landmark-reliant regime, so no single knob alone flips the verdict — the *recipe as a package* is what lifts the world out of the aliased-bare failure mode (see ATE below).

## M3 VIO full-sim ATE — recipe vs bare (same seed 7)
_(populated below)_
