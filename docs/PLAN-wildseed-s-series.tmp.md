# TEMP PLAN — WildSeed session-2 integration + stressor-axes (S-series)

> Working plan derived from reviewing the 21 new WildSeed commits
> (`e5c5b5e..c472085` in `~/GitStuff/WildSeed`, UNPUSHED to origin at review
> time, 2026-07-09). Once executed, the durable parts get folded into
> `docs/PLAN.md` §12 (milestone rows) and `docs/status-and-testing.md`
> (verification log); this file is then deleted.

## Context — what the review found

WildSeed's session-2 work (experiment-spec program + deferred axes) breaks
**nothing** in this repo:

- `wildseed rig --inject --shell-only --no-labels` (used by
  `scripts/prepare_wildseed_world.sh`) and `wildseed height --json -b` are
  untouched. New rig flags `--calib/--calib-seed` default off and only affect
  the model-writing path we don't use.
- `SCENARIO_FORMAT` bumped 3→4, but the new sun/weather randomness is an
  **appended** `SeedSequence` child: with the new dials unset, same seed →
  byte-identical world (pinned by WildSeed's G4 determinism gates + format-3
  golden fixture). `wildseed_42` and `vio_lio_recipe` bundles still reproduce.
- Dynamics distractors are record-path only (`wildseed record --distractors`);
  they never appear in scenario worlds → bundles unaffected.
- New capabilities relevant here: `--texture` dial (**binary in effect**:
  <0.5 composites the uniform/aliasing-worst-case ground, ≥0.5 the patchy
  de-aliased one), `--photometric` 0..1 sun-stress dial (elevation 55°→5°,
  intensity 1→5×, emissive glare disk at ≥0.75), `--weather` presets
  (clear/overcast/fog/rain/snow/sunglare; WildSeed's own offscreen render
  crashed on these pre-commit-7708155 — treat as render risk in our container
  too), `wildseed experiment`/`wildseed sweep` (hypothesis + dial-distribution
  specs → seeded world batches + report cards), `rig --calib` (perturbed SDF +
  `rig_calibration.json` truth export — flying-rig only, but the pattern maps
  to a Husky robustness milestone), `--biome-file` (custom biome YAML;
  explicit-select only, never joins the seed-random pool).

Existing harness to reuse: `scripts/prepare_wildseed_world.sh` →
`deploy.sh world <bundle>` → `scripts/m4_lio_eval.sh` (steady-RTF gate, FRESH
KISS-ICP restart, A/B drive, eval_tools ATE/RPE) → `scripts/plot_m4_sweep.py`.
Known measurement rules: chart **RPE not ATE** (slow-UGV under-report bias);
texture is a **route** property (probe mid-route corners, not spawn);
`m3-smoke` gates every new bundle; OpenVINS must start FRESH after steady
RTF > 0.4 (load-transient trap).

---

## Part A — Updates needed now (one short session, ends in a commit)

> **DONE 2026-07-09** (session 1): A1 pushed (`74a5f5e..c472085`, clean tree);
> A2 `wildseed-worlds.md` stressor-dial section; A3 `provenance.json` step in
> `prepare_wildseed_world.sh` (tested: spec-sidecar + no-sidecar branches);
> A4 S1–S5 block in PLAN §12 + bullet in status-and-testing §5.
>
> **S1 DONE 2026-07-09** (same session): `results/s1_texture_ab.png` — a
> controlled NEGATIVE (texture dial alone doesn't degrade VIO with corridor
> scatter present; mechanism logged: corners AND KLT survival high on both
> variants). Three walls fixed en route: hardlink-bundle corruption (bundler
> real-copies now), vio_lio corridor blocked by rock_moss_set footprints
> (`clear_drive_lane.py`; UPSTREAM FIX WANTED in WildSeed: footprint-aware
> corridor keep-out), OpenVINS load-transient init wedge (`m4_lio_eval.sh`
> restarts both frontends at the RTF gate). Verification log:
> status-and-testing §4. **Next session: S2 (render-gate fog/sunglare first);
> S5's landmark-density lever is the one S1's negative points at.**

**A1. Push WildSeed + pin the hash (user action).** The 21 commits are
unpushed; every bundle/result generated against them is irreproducible until
they're on origin. After the push, cite the hash wherever bundles are
documented.

**A2. Refresh `docs/wildseed-worlds.md`.** Add:
- Format-4 note: same seed reproduces old worlds byte-identically (appended
  sun/weather stream; dials unset = format-3 output).
- The new dials in the workflow section: `--texture`, `--photometric`,
  `--weather`, `--biome-file` (one line each, with the binary-texture caveat).
- Weather-bundle caution: must pass the render gate (GL_RENDERER check +
  camera frame sanity + `m3-smoke`) before metrics from it are trusted.
- Note that record-path distractors never appear in scenario worlds.

**A3. Bundle provenance.** Extend `scripts/prepare_wildseed_world.sh` to write
`provenance.json` next to `spawn.json`: WildSeed git hash (`git -C $WILDSEED
rev-parse HEAD` + dirty flag), scenario format, resolved spec/manifest if
present next to the world file, dial values if known, generation timestamp.
A few lines; makes every future sweep row traceable. **No other script
changes are needed** — rig-inject, height, and the `grep model://` snapshot
(which auto-picks-up weather-emitted model dirs) were all verified compatible.

**A4. Register the S-series** as a block in `docs/PLAN.md` §12 (riding on the
sim like the N-series does) + one bullet in `docs/status-and-testing.md` §5.

---

## Part B — S-series: stressor axes (all laptop-closable)

### S1 — Texture A/B at fixed geometry (upgrade of the M4 terrain sweep)
*Why:* the M4 sweep used biome as a texture proxy, so geometry and texture
varied together. The dial isolates the variable: same seed, same layout, same
route — only the ground compositor changes.
*How:* for seeds {42, +2 more}: `wildseed scenario --seed N --profile vio_lio
--texture 0.0` and `--texture 1.0` → bundle each immediately (models/ground is
overwritten per run) → `m3-smoke` → `m4_lio_eval.sh` drive → aggregate.
Probe mid-route corner counts along the drive (`scripts/check_sim_texture.py`)
to evidence the mechanism, not just the outcome.
*Deliverable:* chart (RPE) — VIO degrades on uniform ground, LIO flat, geometry
held constant. One session; hand-loop (only 6 worlds), don't wait for S3.

### S2 — Photometric + weather axes (second complementarity chart)
*Gate first:* build ONE fog and ONE sunglare bundle; load; check GL_RENDERER,
a camera frame (`scripts/peek_cam.py`), RTF (`scripts/bench_rtf.sh`). Our
ogre2/EGL container path has never rendered particle emitters; WildSeed's own
renderer crashed on weather pre-fix. If particles fail/starve RTF → restrict
the axis to fog + sunglare (scene-level, cheap) and document.
*Sweep:* `--photometric {0, 0.5, 1.0}` at fixed seed — camera stress via
contrast/shadows/glare; lidar invariant. Optionally the 2×3
texture×photometric grid if cheap.
*Honesty note for the writeup:* gz fog attenuates cameras but NOT `gpu_lidar`
— "lidar survives fog" is true in sim BY CONSTRUCTION. Frame as sensor-stress
asymmetry; the real-snow tier (M7b Boreas/CADC) carries the physical claim.

### S3 — Hypothesis-driven sweep harness (`wildseed experiment` integration)
New `scripts/wildseed_axis_sweep.sh`: consume a WildSeed experiment spec YAML
(hypothesis + dial distributions) → `wildseed experiment --count N` in the
checkout → bundle each world IMMEDIATELY → `m4_lio_eval.sh` per bundle →
aggregate WildSeed's report card with our ATE/RPE table (extend
`plot_m4_sweep.py`). Provenance from A3 ties every row to
(WildSeed hash, spec, seed). Build after S1 proves the loop by hand; use it to
run S2's grid. Itself a portfolio artifact (hypothesis-driven benchmark
harness = "test instrument, not demo").

### S4 — Calibration-robustness milestone (biggest new chart, most work)
Adapt WildSeed's perturbed-SDF + truth-JSON pattern to the Husky: seeded
perturbation of OAK-D mount extrinsics (mm / tenths of a degree) + IMU noise
in the robot description — via the **Clearpath generator patch path** (the
generator overrides xacro defaults; a xacro-level sed is a silent no-op, see
M4 lesson) — exporting true values to JSON.
Matrix: perturbed sim × {OpenVINS fed nominals, OpenVINS online extrinsic
calibration enabled} → fused ATE/RPE.
*Deliverable:* estimator error vs miscalibration dial, with/without online
calibration — the most defensible robustness figure for a sensor-fusion role.
Schedule after S1/S2 (robot-description generation has bitten before).

### S5 (optional) — custom biome via `--biome-file`
Route-texture biome YAML steering landmark density along the corridor. Only if
S1's binary texture lever is too coarse for the story; otherwise fold the idea
into the S1 writeup.

---

## Sequencing

| Session | Work | Ends with |
|---|---|---|
| 1 | A1–A4 (push, docs, provenance, PLAN registration) | commit |
| 2 | S1 end-to-end | commit + chart |
| 3 | S2 render gate → photometric sweep | commit + chart |
| 4 | S3 harness; re-run S2 grid through it | commit |
| later | S4 as its own milestone; S5 only if needed | commit + chart |

Every new bundle passes `m3-smoke` before metrics are trusted; every eval
respects the steady-RTF gate and FRESH-frontend-start rules already encoded in
`m4_lio_eval.sh`.
