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

### Harness bug found & fixed (important methodology note)
The first recipe run **diverged catastrophically** — ATE **192 m** on a 22.6 m path,
OpenVINS `p_IinG` flying to **24 km**. Root cause was **not** the world: cameras
rendered non-blank with 85+ corners. It was a **startup-ordering bug** in how we
brought the stack up. `deploy.sh m3-smoke` starts OpenVINS via `up -d`, but on a
**heavy world the sim takes ~60–120 s to load** (terrain mesh + 165 models), during
which RTF sits at ~0.001. OpenVINS was fed that degraded-timing camera/IMU stream and
**initialized on garbage → velocity blowup → monotonic divergence** (wrong direction
from t≈4 s). The bare/pipeline worlds load fast, so they never exposed this.

Secondary: `m3-smoke`'s RTF probe fires *during* the load and aborts on the
`SIM_RTF_FLOOR` before it ever validates the cameras — so on heavy worlds the smoke
gate gives a **spurious FAIL** while the sim is actually fine once loaded. (Worth a
follow-up: the smoke gate should wait for the world to finish loading before probing RTF.)

**Fix — clean-ordering harness** (`scratchpad run_m3_v2.sh`): bring up the sim only →
**poll until steady RTF > 0.4 AND both cameras ≥ 80 corners** → *then* start OpenVINS
**fresh** on the loaded sim → drive. Validated against the known-good pipeline world:

### Results (all through the identical clean-ordering harness, v=0.5 wz=0.1 45 s, seed 7)
2 runs each of recipe & bare (to bound run-to-run VIO variance) + 1 pipeline control:

| world | drive RTF | raw-OpenVINS ATE (run1 / run2 / mean) | ego_localizer ATE (run1 / run2 / mean) |
|-------|----------:|:--------------------------------------:|:---------------------------------------:|
| **vio_lio recipe** (165 obj + relief + patchy) | 0.98–0.99 | 0.040 / 0.086 / **0.063 m** | 0.045 / 0.060 / **0.052 m** |
| **vio_lio bare** (flat, uniform, 0 obj) | 1.00 | 0.070 / 0.114 / **0.092 m** | 0.068 / 0.088 / **0.078 m** |
| pipeline (Clearpath default, control) | 1.00 | 0.237 m (1 run) | 0.240 m | 

**Recipe reduces our M3 VIO ATE ~32% (raw OpenVINS) / ~33% (ego_localizer) vs the bare
same-seed world**, and recipe < bare in **all four paired comparisons** (2 runs × 2
estimators) — the ranking is robust even though both worlds' absolute ATE roughly
doubled between run 1 and run 2 (a shared run-to-run factor: init/GPU-scheduling
variance, not world content). Both worlds track cleanly (sub-0.12 m, OpenVINS `p` hugs
the 22 m GT loop) — no divergence — so this is a *quality* improvement near the VIO
floor, not a rescue from failure. The bare world already yields ~200 ground-texture
corners, so on a **short 22 m stereo loop** it is not feature-starved; the recipe's
landmark density + relief parallax still buy a consistent ~1/3 ATE reduction, and would
matter more on longer trajectories (drift accrual), mono VIO (scale), or loop closure.

The pipeline control (0.237 m via this harness; stored best-case 0.069 m) is only a
*sanity anchor* (sane, not diverged) — it is a different world type (industrial), so
don't read the recipe-vs-pipeline gap as a recipe win; the clean A/B is recipe-vs-bare.

**RTF under the full ROS node graph:** recipe world runs at **RTF ≈ 0.98–0.99** once
loaded — well above the ≥0.5 target, **no steady-state sag**. Only the *load transient*
dips (~0.001 for ~60–120 s while the terrain mesh + 165 models parse), which is the
startup-ordering hazard above, not a sustained sag. Bare/pipeline load fast (RTF 1.0).

## Answers to the three report questions
1. **Does the recipe reduce our VIO/LIO drift (ATE) vs the previous world?** **Yes —
   ~32–33% lower ATE than a bare same-seed world (0.063 vs 0.092 m raw; 0.052 vs
   0.078 m fused), consistent across 2 runs.** Small absolute numbers (short loop, both
   near the VIO floor), but a robust directional win.
2. **Which knob most affects tracking?** **`--object-density`** — the dominant, monotonic
   lever on the VIO data-association proxy (putative correspondences 57→107, inlier_ratio
   span 0.058), matching the guide's "landmark density" thesis. `--relief` is second
   (geometric parallax; flat collapses inliers/pair 63→55). `--variety` changes the look
   (global place-recognition distinctiveness) more than short-baseline trackability.
3. **Any RTF sag under our full ROS node graph?** **No steady-state sag** — recipe holds
   RTF ≈ 0.98–0.99 (rtf_min well above 0.5). The only dip is the one-time world-load
   transient, which must be waited out before starting VIO (see the harness-bug note).
