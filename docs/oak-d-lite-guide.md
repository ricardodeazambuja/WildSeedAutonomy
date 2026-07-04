# OAK-D Lite — Complete Field Guide (IMU, Cameras, Sync, VIO, depthai versions)

A self-contained, evidence-based reference for working with the Luxonis **OAK-D Lite**
(RVC2 / Intel Movidius Myriad X) — how to confirm and characterise the IMU, how the
camera↔IMU timing works for VIO, the depthai version landscape, and the full write-up of a
camera crash we root-caused to a firmware regression.

| | |
|---|---|
| **Device** | Luxonis OAK-D Lite, serial `18443010013BCB0F00` |
| **SoC / platform** | Intel Movidius Myriad X = **RVC2** |
| **IMU** | Bosch **BMI270** (6-axis), IMU firmware `1.0.0` |
| **Mono/stereo cameras** | 2× **OV7251** — 640×480, **global shutter**, monochrome |
| **Color camera** | **IMX214** — 13 MP, **rolling shutter**, autofocus |
| **Host** | Linux 6.8, USB3 (negotiates `SUPER`; `HIGH`/USB2 via the powered hub) |
| **Verified** | depthai 2.32 / 3.0.0 / 3.4.0 / 3.6.1 / 3.7.1 |

> Every number, table, and claim below was measured on this unit or read from primary
> sources (Luxonis docs, the device crash dump, the `depthai-core` git history). Where a
> claim is reasoning rather than measurement, it says so.
>
> How this hardware fits the project's *design* (don't rely on the BMI270 for tight VIO;
> sim-vs-real differences to bake in) is summarized in **PLAN §8** ([`PLAN.md`](PLAN.md)).
> The probe scripts and venvs in §11 live at the **repo root**, not in `docs/`.

---

## Table of contents
1. [TL;DR — the decisions](#1-tldr--the-decisions)
2. [Quick start](#2-quick-start)
3. [Hardware overview & the RVC2/3/4 platforms](#3-hardware-overview--the-rvc234-platforms)
4. [The IMU (BMI270)](#4-the-imu-bmi270)
5. [Camera ↔ IMU synchronisation for VIO](#5-camera--imu-synchronisation-for-vio)
6. [The depthai 3.7.1 mono-camera crash — full investigation](#6-the-depthai-371-mono-camera-crash--full-investigation)
7. [depthai version guide (v2 vs v3, ROS, firmware, speed)](#7-depthai-version-guide)
8. [Recommended VIO setup](#8-recommended-vio-setup)
9. [Troubleshooting playbook & gotchas](#9-troubleshooting-playbook--gotchas)
10. [Reproduce / inspect — command reference](#10-reproduce--inspect--command-reference)
11. [Files in this project](#11-files-in-this-project)
12. [Open items & reporting upstream](#12-open-items--reporting-upstream)
13. [Appendix — raw measurements & identifiers](#13-appendix--raw-measurements--identifiers)

---

## 1. TL;DR — the decisions

- **This unit HAS an IMU** (BMI270). Confirm any unit with one line:
  `dai.Device().getConnectedIMU()` → `"BMI270"` or `"NONE"`. (Kickstarter-era units
  shipped *without* an IMU; purchase date proves nothing; the IMU is invisible to `lsusb`.)
- **Cameras and the IMU share one on-device hardware clock** — confirmed empirically.
  This is the property that makes the OAK-D Lite usable for VIO.
- **Use depthai `3.6.1` (newest working v3) or `2.x`. Avoid `3.7.1`** — it crashes the
  **mono** camera via a device-firmware regression (color and IMU are unaffected).
  Fix: `pip install depthai==3.6.1`.
- For **VIO**: mono (global-shutter) + IMU @ **200 Hz**, **Kalibr** the cam–IMU extrinsics,
  enable online time-offset (`td`) estimation, expect **yaw drift** (no magnetometer).

---

## 2. Quick start

```bash
# Working v3 environment (this project ships requirements.txt):
python3 -m venv oak-venv3
./oak-venv3/bin/pip install -r requirements.txt        # depthai==3.6.1, numpy==2.2.6

# Confirm the IMU and stream a camera:
./oak-venv3/bin/python -c "import depthai as dai; print(dai.Device().getConnectedIMU())"
./oak-venv3/bin/python probe_cam_only.py B 640 480      # mono left; 'A 1920 1080' = color
```

If camera streaming throws `X_LINK_ERROR` and the device "crashes", you are almost
certainly on **depthai 3.7.1** — see §6. Pin 3.6.1.

---

## 3. Hardware overview & the RVC2/3/4 platforms

### Sensors (verified against Luxonis docs)
| Role | Part | Resolution | Shutter | Notes |
|---|---|---|---|---|
| Left/right mono (stereo) | **OV7251** | 640×480 | **Global** | Monochrome; good for VIO |
| Color | **IMX214** | 4208×3120 (13 MP) | **Rolling** | Autofocus |
| IMU | **BMI270** | — | — | 6-axis; see §4 |

Camera sockets in software: `CAM_A` = color (IMX214), `CAM_B` = left mono, `CAM_C` = right
mono.

### RVC2 vs RVC3 vs RVC4 — different chips, not modes
These are hardware generations; you cannot "switch" a board between them.

| Platform | SoC | Status | This device? |
|---|---|---|---|
| **RVC2** | Intel Movidius **Myriad X** | Actively supported | **✅ yes** |
| RVC3 | Intel Movidius **Keem Bay** | **Deprecated** (Intel deprioritised the chip) | no |
| RVC4 | **Qualcomm** QCS6490 | Current next-gen (OAK4) | no |

The OAK-D Lite is permanently RVC2. depthai v3 *bundles* firmware blobs for RVC2/RVC3/RVC4
(you'll see `depthaiDeviceRVC3Version` / `...RVC4Version` strings), but only the **RVC2**
blob is ever loaded onto a Myriad X device — the others are inert payloads for other
hardware. "Using RVC3/RVC4" is impossible and would gain nothing (RVC3 is a dead-end line).

---

## 4. The IMU (BMI270)

### 4.1 How to confirm a unit has an IMU
The IMU sits on the VPU's internal bus and is **invisible at the OS level** — `lsusb` only
ever shows the VPU (`03e7:2485`). Purchase date is unreliable (chip-shortage "new-old-stock"
without IMUs circulated for a long time). The only definitive check talks to the firmware:

```bash
python -m venv /tmp/oak && /tmp/oak/bin/pip install depthai
/tmp/oak/bin/python -c "import depthai as dai; print(dai.Device().getConnectedIMU())"
# -> "BMI270"  (has IMU)   |   "NONE"  (no IMU)
```

This unit returns **`BMI270`** (IMU firmware `1.0.0`).

### 4.2 BMI270 chip capability vs what DepthAI exposes
The BMI270 is a "smart" wearables IMU; most of its on-chip cleverness is **not surfaced**
by DepthAI on the OAK-D Lite.

| BMI270 chip feature | On the chip | Exposed via DepthAI (RVC2)? |
|---|---|---|
| Accelerometer ±2/4/8/16 g, 16-bit | ✅ | ✅ (raw / uncalibrated / calibrated) |
| Gyroscope ±125…2000 °/s, 16-bit | ✅ | ✅ (raw / uncalibrated / calibrated) |
| Magnetometer | ❌ (none on board) | ❌ |
| On-chip orientation fusion (quaternion) | ❌ | ❌ (`ROTATION_VECTOR` unsupported) |
| Max ODR | 1.6 kHz acc / 6.4 kHz gyro | capped **~250 Hz** on RVC2 |
| 2 KB FIFO / batched reports | ✅ | ✅ (`setBatchReportThreshold`) |
| Temperature sensor | ✅ | ❌ (not in `IMUData`) |
| Feature engine (step/gesture/motion) | ✅ | ❌ |
| Gyro CRT motionless self-calibration | ✅ | ❌ (internal) |
| Aux I²C for external magnetometer | ✅ | ❌ (not wired on OAK-D Lite) |
| OIS high-speed SPI tap | ✅ | ❌ |
| Typical current (full accel+gyro) | ~685 µA | — |

**Bottom line:** you get **timestamped raw accel + gyro**, no magnetometer, no on-chip
fusion. Do fusion host-side (Madgwick/Mahony/EKF or a full VIO stack), and expect
**unobservable yaw drift** without an external heading reference.

### 4.3 Measured IMU characteristics (this unit, depthai 3.7.1)
| Requested | Effective rate | Δt mean | Δt std | Jitter (max−min) |
|---|---|---|---|---|
| 200 Hz | 197.1 Hz | 5.078 ms | **0.002 ms** | **0.063 ms (63 µs)** |
| 400 Hz | **250.1 Hz** (caps) | 4.001 ms | 1.253 ms | 2.628 ms |

- **Use ~200 Hz.** Timestamping is rock-solid (63 µs jitter). Requesting 400 Hz caps at
  ~250 Hz *and* makes Δt irregular (alternating ~2.5/5 ms) — worse for VIO.
- **Accel and gyro share identical timestamps** (offset 0.000 ± 0.000 ms) → sampled
  together, no inter-axis interpolation needed.

### 4.4 Factory camera↔IMU extrinsics (and why you still need Kalibr)
`readCalibration().getImuToCameraExtrinsics(socket)` returns 4×4 transforms, but they are
**partly nominal** on this unit:
- IMU→`CAM_A` (color): rotation is an *exact* axis-flip `diag(1, −1, −1)`; spec == measured
  translation → **nominal/design** value.
- IMU→`CAM_B`/`CAM_C` (mono): small real off-diagonal rotation terms (~0.001–0.016 rad) and
  spec ≠ measured translation (e.g. CAM_B z: −0.084 cm measured vs −0.319 cm spec) → real
  per-unit *stereo* calibration is folded in, but the **IMU's own mounting rotation is still
  nominal**.
- **For tight VIO, calibrate the cam–IMU transform with Kalibr** and keep **online `td`
  (time-offset) estimation** on (VINS-Mono `estimate_td`, OpenVINS `calib_camimu_dt`).

---

## 5. Camera ↔ IMU synchronisation for VIO

**Confirmed: a single shared on-device clock.** All messages (cameras + IMU) are
timestamped from the device's monotonic clock (`getTimestampDevice()` / `dai.Clock.now()`),
and the device clock is synced to the host to **< 200 µs over USB** (Luxonis docs). The
two mono cameras are global-shutter (hardware-syncable); the IMU **free-runs** at its ODR
(not exposure-triggered) but is timestamped on the same clock — so VIO interpolates IMU
between frame times, which is exactly how VINS/OpenVINS/Kalibr expect to work.

**Empirical proof on this unit** (mono CAM_B @30 Hz + IMU @200 Hz, depthai 2.32):
```
camera: n=148  rate=30.2 Hz   device-ts = [2.1677, 7.0669] s
imu:    n=969  rate=197.1 Hz  device-ts = [2.1687, 7.0843] s
per-frame nearest-IMU gap: mean=1.275 ms, max=2.533 ms
```
- Both streams' timestamps start at the same instant (~2.168 s) on one timeline.
- Every camera frame has an IMU sample within **2.533 ms** ≈ *half* the IMU period
  (1/197 Hz = 5.08 ms → 2.54 ms). Mean gap 1.275 ms ≈ a quarter-period. This is the
  textbook signature of a single shared hardware clock.

This shared timebase is the real enabler for VIO; it's far better than a USB webcam + a
separate USB/serial IMU on independent clocks.

---

## 6. The depthai 3.7.1 mono-camera crash — full investigation

### Symptom
Starting **any** camera output crashes the device within ~1 s; IMU-only runs forever:
```
[depthai] error: Communication exception ... 'Couldn't read data from stream: '__x_0_0' (X_LINK_ERROR)'
[host] warning: Closed connection / Attempting to reconnect
[depthai] error: Device ... has crashed. Crash dump logs are stored in: .../crashdumps/...
RESULT: received 0 frames
```

### Hypotheses tested (and why each was wrong)
| # | Hypothesis | Test | Result |
|---|---|---|---|
| 1 | USB **power** | Powered USB hub | ❌ still crashed |
| 2 | **`usbfs_memory_mb`** too small | raised 16 → 256 MB | ❌ still crashed |
| 3 | **IMU concurrency** | camera-only pipeline | ❌ crashes camera-alone |
| 4 | **USB3 instability** | retried on USB2 (`HIGH`) hub | ❌ crashes at USB2 too |
| 5 | **Pixel format** | NV12, GRAY8, RAW8, RAW10, full-res | ❌ all crash |
| 6 | **depthai version** | v2.32 + older v3 | ✅ **root cause** |

### Root cause (from the on-device crash dump)
The Myriad X runs RTEMS; the dump (`~/.cache/depthai/crashdumps/<hash>/crash_dump_*.tar.gz`
→ `crash_reports.json`) shows a **firmware** crash:
```
platform: RVC2,  depthaiVersion: 3.7.1,  deviceFw: 8d6a04d…
"[FAIL] - Start Source: Invalid config steps, 1 -- .../plgSrcMipi/leon/PlgSrcMipi.cpp: threadFunc:1019"
assertion "0" failed: file ".../PlgSrcMipi.cpp", line 1020, function: PlgSrcMipi::threadFunc
Fatal error RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE
```
The firmware asserts in its **MIPI camera-source plugin** (`PlgSrcMipi.cpp`) on "Invalid
config steps", corrupts the heap, and panics. The host sees only the aftermath. This rules
out power/usbfs/cable/USB-speed — the crash is **inside the chip, after** the USB link is
up — and explains why the IMU (which never touches the MIPI source) is unaffected.

### Scope, pinned by bisection (each result bracketed by a known-good v2 control)
| Test | Result |
|---|---|
| v2.32 mono OV7251 | ✅ 45 frames |
| v3.0.0 mono | ✅ 39 frames |
| v3.4.0 mono | ✅ 39 frames |
| v3.6.1 mono | ✅ 39 frames |
| **v3.7.1 mono** | ❌ crash (all formats) |
| v3.7.1 **color** IMX214 | ✅ 39 frames @1080p |
| v3.7.1 IMU | ✅ |

The regression entered **between 3.6.1 and 3.7.1** and affects **only the mono OV7251**.
A v2.32 control before *and* after each failing test always returned 45 frames, proving the
device was healthy and not merely wedged by a prior crash — that control is what makes the
bisection trustworthy. (Earlier wrong conclusions: "v3 library problem" → "v3 RVC2 path
broken" → "mono path broken in v3" → finally the correct, narrow "**3.7.1-only mono
regression**".)

### Confirmed by source diff: it's the firmware blob, not host code
`gh api repos/luxonis/depthai-core/compare/v3.6.1...v3.7.1`: **224 files changed, none a
camera/mono/sensor node implementation** (no `Camera.cpp`, `MonoCamera.cpp`,
`ColorCamera.cpp`, `StereoDepth.cpp`). The only camera-relevant change is the pinned
device-firmware commit:
```diff
# cmake/Depthai/DepthaiDeviceSideConfig.cmake
-set(DEPTHAI_DEVICE_SIDE_COMMIT "358472a96039cc24ae416b9612210f04544c1928")  # v3.6.1 (works)
+set(DEPTHAI_DEVICE_SIDE_COMMIT "8d6a04d380ce182e0cb3a4694309426370295a9f")  # v3.7.1 (crashes)
```
(The 3.7.1 hash matches `depthaiDeviceVersion` in the crash dump.) So the host sends the
*same* mono config in both releases — only the **firmware blob** changed. The asserting
file `PlgSrcMipi.cpp` lives in Intel's proprietary `mdk` (Myriad X SDK) and the firmware is
built from Luxonis's **private** device repo, so the exact buggy line isn't publicly
readable — but the regression is bounded to firmware commit range **`358472a` → `8d6a04d`**.

### Why it isn't a mass outcry
RVC2 is **legacy** in v3 (which targets RVC4); the RVC2 camera path is less exercised; most
OAK-D Lite users are on v2; color works even on 3.7.1; and the usual advice ("update / bad
cable / re-seat ribbon") doesn't fix a firmware bug. The failure class *is* reported, just
scattered: `luxonis/depthai #1231`, `luxonis/depthai-ros #308` (PlgSrcMipi fatal), `#245`.

### Ruled-out hardware hypothesis
Luxonis docs note a rare loose **camera ribbon** (identical symptom). Ruled out here: v2.32
streamed full video, so the camera hardware is good — the fault is purely firmware.

---

## 7. depthai version guide

### Which version to use
| Use case | Recommendation |
|---|---|
| Local Python prototyping | **depthai 3.6.1** (newest working v3) |
| ROS2 / max RVC2 stability | **depthai 2.x** (mature, battle-tested RVC2 line) |
| Anything | **Avoid 3.7.1** (broken mono on RVC2) |

### Do you ever need v2.x? Yes — mainly for ROS
- **ROS2 (`depthai-ros`)**: the mature driver line is **v2-based** and is the best-trodden
  path for OAK-D Lite; depthai-ros notes **VIO/SLAM only work on RVC2**. The newer
  `depthai-ros` 3.x line uses depthai **v3** and *may pin 3.7.1* — which would reintroduce
  the mono crash. **Check the exact depthai version your depthai-ros branch pins.**
- Most existing OAK examples/tutorials use the **v2 API** (`Pipeline`+`XLinkOut`+`getOutputQueue`).
- v2 is the mature RVC2 line; v3 treats RVC2 as legacy.
- Keep `oak-venv2` around — cheap insurance for the ROS side.

### Is v2 faster than v3? No (measured)
Same mono camera, same request: **v2.32 → 30.0 fps, v3.6.1 → 30.1 fps** (identical). At the
ceiling, v2 ran the sensor's high-FPS mode up to the **USB2 bandwidth limit (~93 fps)** for
640×480 grayscale, while v3's `Camera` node *rejected* 120 fps for that exact format
(`No available sensor config` — graceful error, an API-negotiation difference, not speed).
v2 and v3 ship **different firmware blobs** (three versions = three blobs); not the same.

### Can you mix a 3.7.1 host with the 3.6.1 firmware?
Technically yes — depthai exposes **`DEPTHAI_DEVICE_BINARY`** to boot a custom firmware (the
blob is `depthai-device-fwp-<commit>.tar.xz` embedded in `libdepthaicore.so`, extracted to
`/tmp/depthai_*` at boot). **But don't:** host and firmware are version-matched (mixing is
unsupported, protocol/schema mismatch risk), and 3.7.1's only host-side additions (Samsung
ToF sensor, CBA calibration, BNO086 fix) are irrelevant to OAK-D Lite VIO. Just install
3.6.1. The swap is only worth it if you ever need a specific 3.7.1 host feature *and*
working cameras.

---

## 8. Recommended VIO setup

1. **depthai 2.x or 3 ≤ 3.6.1** (never 3.7.1).
2. **Mono (global-shutter) + IMU**; leave the rolling-shutter color camera out of VIO.
3. **IMU at ~200 Hz** (clean 63 µs jitter; avoid 400 Hz).
4. USB2 (`HIGH`) is fine for mono+IMU; USB3 only if you add high-res color.
5. **Kalibr** the cam–IMU extrinsics + intrinsics + IMU noise model; enable **online `td`**.
6. No magnetometer → constrain **yaw** with vision/GPS/external heading as needed.
7. Power from a hub if you ever see instability (wasn't the cause here, but good hygiene).

---

## 9. Troubleshooting playbook & gotchas

- **`X_LINK_ERROR` on mono-camera open + "device crashed", IMU fine → you're on 3.7.1.**
  Pin `depthai==3.6.1`. Don't chase power/usbfs/cable; read `crash_reports.json` — a
  `PlgSrcMipi` / "Invalid config steps" entry means a device-side firmware crash. Changing
  resolution/format will **not** help (all formats crash on 3.7.1).
- **Bisecting hardware-dependent failures: bracket every test with a known-good control.**
  A device wedged by a prior crash looks identical to "this version is broken." The control
  (a v2 run that returns frames) is what makes the bisect trustworthy.
- A crashed pipeline holds the device through a ~10 s reconnect timeout; **kill stuck
  processes** before retrying or they fight over the device.
- **`pgrep -f "<pattern>"` can match its own shell** if the pattern appears in the running
  command line — `pgrep … | xargs kill -9` then kills your own script. Use a pattern that
  can't match the killer command.
- Raising `usbfs_memory_mb` (default 16) is good general hygiene for OAK cameras even though
  it wasn't the cause here: `sudo sh -c 'echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb'`
  (persist via `usbcore.usbfs_memory_mb=256` on the kernel cmdline).
- **venvs don't survive renaming** (absolute paths baked into `activate`/shebangs). To
  "rename": `pip freeze > requirements.txt` → new venv → install → test → delete the old.

---

## 10. Reproduce / inspect — command reference

```bash
# Is the device enumerated? (only the VPU shows; the IMU never does)
lsusb | grep -i 03e7                          # 03e7:2485 = Movidius MyriadX

# Confirm IMU type
./oak-venv3/bin/python -c "import depthai as dai; print(dai.Device().getConnectedIMU())"

# Reproduce the 3.7.1 crash in a throwaway venv (then it works on color / older v3)
python3 -m venv /tmp/v371 && /tmp/v371/bin/pip install depthai==3.7.1 numpy
/tmp/v371/bin/python probe_cam_only.py B 640 480     # mono -> crash
/tmp/v371/bin/python probe_cam_only.py A 1920 1080   # color -> works

# Read a crash dump
d=$(ls -1dt ./.cache/depthai/crashdumps/*/ ~/.cache/depthai/crashdumps/*/ 2>/dev/null | head -1)
tar xzf "$d"/crash_dump_*.tar.gz -C /tmp/cd && less /tmp/cd/crash_reports.json
# -> errorSource + the "prints" array name the failing component (here: PlgSrcMipi)

# Confirm the firmware-pin regression in the open host repo
gh api repos/luxonis/depthai-core/compare/v3.6.1...v3.7.1 \
  --jq '.files[] | select(.filename | test("DeviceSideConfig")) | .patch'
```

---

## 11. Files in this project

**Environments** (both verified working; 3.7.1 deliberately not kept):
- `oak-venv3/` — depthai **3.6.1** (newest working v3). Rebuild:
  `python3 -m venv oak-venv3 && ./oak-venv3/bin/pip install -r requirements.txt`.
- `oak-venv2/` — depthai **2.32** (mature RVC2 / ROS path).
- `requirements.txt` — `depthai==3.6.1`, `numpy==2.2.6`.

**Probe / benchmark scripts** (each takes the venv via `./<venv>/bin/python`):
| Script | Purpose |
|---|---|
| `probe_imu.py` | IMU rate/jitter characterisation (v3 API) |
| `probe_calib.py` | dump factory IMU→camera extrinsics (v3 API) |
| `probe_cam_only.py` | camera test, args `SOCKET W H` (`B 640 480` mono, `A 1920 1080` color) |
| `probe_cam_only_v3.py` | copy used during bisection |
| `probe_mono_formats.py` | tries GRAY8/RAW8/RAW10/NV12 on mono (all crash on 3.7.1) |
| `probe_sync_v2.py` | camera+IMU shared-clock capture (v2 API) |
| `probe_cam_v2.py` | v2 camera smoke test — doubles as the known-good **control** |
| `probe_sync.py` | v3 camera+IMU sync (crashes on 3.7.1) |
| `bench_fps_v2.py` / `bench_fps_v3.py` | fair FPS benchmark, arg = requested fps |

> Crash dumps land in `./.cache/depthai/crashdumps/<hash>/crash_dump_*.tar.gz`.

---

## 12. Open items & reporting upstream

- **Report the regression to Luxonis.** We have everything their device team needs: a
  bisected firmware commit range (**`358472a` good → `8d6a04d` bad**), the crash dump
  (`PlgSrcMipi.cpp:1020` "Invalid config steps" → `RTEMS_FATAL`), and the fact that color +
  IMU are unaffected and the host source is unchanged. `git log 358472a..8d6a04d` in their
  private device repo points straight at the offending change.
- **Not established:** the exact firmware source line (closed `mdk`), and whether any future
  v3 (> 3.7.1) fixes it (3.7.1 is the latest v3 as of writing). Re-test new v3 releases
  before adopting them for mono cameras.
- **Unverified niceties:** whether the OAK-D Lite mono pair is FSIN hardware-synced to each
  other (global-shutter, so likely; not re-verified here) — irrelevant to the shared-clock
  result, which is proven.

---

## 13. Appendix — raw measurements & identifiers

- **Device serial:** `18443010013BCB0F00` · **VPU USB id:** `03e7:2485` (Movidius MyriadX)
- **IMU:** BMI270, IMU FW `1.0.0`
- **depthai 3.7.1 device fw (bad):** `8d6a04d380ce182e0cb3a4694309426370295a9f`
- **depthai 3.6.1 device fw (good):** `358472a96039cc24ae416b9612210f04544c1928`
- **Bootloader:** `0.0.28`
- **IMU @200 Hz:** 197.1 Hz, Δt mean 5.078 ms, std 0.002 ms, jitter 63 µs; accel/gyro
  co-timestamped (offset 0.000 ms).
- **IMU @400 Hz:** caps 250.1 Hz, Δt std 1.253 ms (irregular).
- **Sync (v2.32):** cam 30.2 Hz + IMU 197.1 Hz; per-frame nearest-IMU gap mean 1.275 ms,
  max 2.533 ms; shared device clock.
- **FPS parity:** v2.32 30.0 fps vs v3.6.1 30.1 fps @30; v2 ceiling ~93 fps (USB2);
  v3.6.1 rejected 120 fps for 640×480 GRAY8.
- **Available depthai:** v3 = 3.0.0, 3.1.0, 3.2.0/.1, 3.3.0, 3.4.0, 3.5.0, 3.6.1, 3.7.1;
  v2 latest = 2.32.0.0.

### Primary sources
- [OAK-D Lite hardware](https://docs.luxonis.com/hardware/products/OAK-D%20Lite) ·
  [OV7251](https://docs.luxonis.com/projects/hardware/en/latest/pages/articles/sensors/ov7251.html)
- [DepthAI IMU node (v3)](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu) ·
  [IMUData / timestamps](https://docs.luxonis.com/projects/api/en/latest/components/messages/imu_data/)
- [CalibrationHandler.getImuToCameraExtrinsics](http://docs.ros.org/en/noetic/api/depthai/html/classdai_1_1CalibrationHandler.html) ·
  [Kalibr→imuExtrinsics thread](https://discuss.luxonis.com/d/2347-how-to-convert-kalibrs-t-cam-imu-to-depthais-imuextrinsics)
- [depthai-core](https://github.com/luxonis/depthai-core) ·
  [depthai-ros](https://github.com/luxonis/depthai-ros) ·
  [depthai-ros 3.0 notes](https://discuss.luxonis.com/blog/6392-depthai-ros-release-30)
- Related issues: [depthai #1231](https://github.com/luxonis/depthai/issues/1231) ·
  [depthai-ros #308](https://github.com/luxonis/depthai-ros/issues/308) ·
  [#245](https://github.com/luxonis/depthai-ros/issues/245)

---

*Compiled from measurements on this unit and primary sources. If you change the
depthai version or hardware, re-verify the camera/IMU sections — especially anything tied to
the 3.7.1 firmware regression.*
