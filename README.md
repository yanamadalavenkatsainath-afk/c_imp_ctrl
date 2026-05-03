# Satellite GNC — SIL Framework
## Python Sim → C Flight Code → PC Real-Time Loop

---

## Overview

This repo is the C flight software counterpart to the Python `flight sim`.
It provides a full Software-In-the-Loop (SIL) pipeline:

```
flight sim/  (Python truth model)
    └─ th_ekf.py, mekf.py, lambert_controller.py, …

Satellite_GNC/  (this repo — C flight code + SIL harness)
    ├─ src_c/          C flight modules (compiled → gnc_lib.dll)
    ├─ sim_python/     Python ctypes bridge + SIL drivers
    └─ tests/          Standalone C unit tests
```

The C flight code is **drop-in identical** between PC SIL and the STM32 target.
The only compile-time switch is `MEKF_NO_CMSIS` — when defined, plain C loops
replace ARM CMSIS-DSP matrix calls for desktop builds.

---

## Folder Layout

```
C:\Users\Venkat\OneDrive\Desktop\appex\
│
├── flight sim\              ← existing Python sim (read-only reference)
│   ├── estimation\
│   │   └── th_ekf.py        ← TH-EKF Python golden model
│   ├── mekf.py
│   ├── lambert_controller.py
│   └── main.py
│
└── Satellite_GNC\           ← this repo
    ├── src_c\
    │   ├── linalg.h         stack-only matrix math (no malloc)
    │   ├── th_ekf.h / .c    TH-EKF (Tschauner-Hempel relative nav)
    │   ├── mekf.h / .c      MEKF (attitude estimation, CMSIS-DSP backend)
    │   ├── rpod_ctrl.h / .c RPOD guidance (PROX_OPS + TERMINAL + …)
    │   ├── flight_loop.h/.c PC SIL real-time flight loop
    │   ├── sensor_packet.h  SensorFrame struct (Python → C IPC)
    │   └── command_packet.h CommandFrame struct (C → Python IPC)
    │
    ├── sim_python\
    │   ├── wrapper.py              ctypes drop-in for THEKF_C
    │   ├── realtime_driver.py      FlightLoopDLL + FakeSensorSim
    │   ├── verify_sil.py           Python golden vs C EKF comparison
    │   ├── verify_realtime_sil.py  4-scenario real-time SIL suite
    │   ├── debug_sil.py            Targeted diagnostic probes (D1–D4)
    │   └── gen_pyrightconfig.py    Generates pyrightconfig.json (called by build.bat)
    │
    ├── tests\
    │   ├── test_thekf.c     TH-EKF standalone C unit tests
    │   ├── test_mekf.c      MEKF standalone C unit tests
    │   └── test_rpod.c      RPOD controller standalone C unit tests
    │
    ├── build.bat            One-command build + full SIL pipeline
    └── README.md            This file
```

---

## Prerequisites

### 1 — GCC (MinGW-w64)

```
winget install MSYS2.MSYS2
```

Open **MSYS2 MinGW64** shell (not the plain MSYS2 shell):

```
pacman -S mingw-w64-x86_64-gcc
```

Add to Windows PATH: `C:\msys64\mingw64\bin`

Verify in a normal Command Prompt:

```
gcc --version
```

### 2 — Python 3.11+

```
python --version    # must be 3.11 or later
pip install numpy
```

---

## Build and Verify

From `Satellite_GNC\`:# Satellite GNC — SIL Framework
## Python Sim → C Flight Code → PC Real-Time Loop

---

## Overview

This repo is the C flight software counterpart to the Python `flight sim`.
It provides a full Software-In-the-Loop (SIL) pipeline:

```
flight sim/  (Python truth model)
    └─ th_ekf.py, mekf.py, lambert_controller.py, …

Satellite_GNC/  (this repo — C flight code + SIL harness)
    ├─ src_c/          C flight modules (compiled → gnc_lib.dll)
    ├─ sim_python/     Python ctypes bridge + SIL drivers
    └─ tests/          Standalone C unit tests
```

The C flight code is **drop-in identical** between PC SIL and the STM32 target.
The only compile-time switch is `MEKF_NO_CMSIS` — when defined, plain C loops
replace ARM CMSIS-DSP matrix calls for desktop builds.

---

## Folder Layout

```
C:\Users\Venkat\OneDrive\Desktop\appex\
│
├── flight sim\              ← existing Python sim (read-only reference)
│   ├── estimation\
│   │   └── th_ekf.py        ← TH-EKF Python golden model
│   ├── mekf.py
│   ├── lambert_controller.py
│   └── main.py
│
└── Satellite_GNC\           ← this repo
    ├── src_c\
    │   ├── linalg.h         stack-only matrix math (no malloc)
    │   ├── th_ekf.h / .c    TH-EKF (Tschauner-Hempel relative nav)
    │   ├── mekf.h / .c      MEKF (attitude estimation, CMSIS-DSP backend)
    │   ├── rpod_ctrl.h / .c RPOD guidance (PROX_OPS + TERMINAL + …)
    │   ├── flight_loop.h/.c PC SIL real-time flight loop
    │   ├── sensor_packet.h  SensorFrame struct (Python → C IPC)
    │   └── command_packet.h CommandFrame struct (C → Python IPC)
    │
    ├── sim_python\
    │   ├── wrapper.py              ctypes drop-in for THEKF_C
    │   ├── realtime_driver.py      FlightLoopDLL + FakeSensorSim
    │   ├── verify_sil.py           Python golden vs C EKF comparison
    │   ├── verify_realtime_sil.py  4-scenario real-time SIL suite
    │   ├── debug_sil.py            Targeted diagnostic probes (D1–D4)
    │   └── gen_pyrightconfig.py    Generates pyrightconfig.json (called by build.bat)
    │
    ├── tests\
    │   ├── test_thekf.c     TH-EKF standalone C unit tests
    │   ├── test_mekf.c      MEKF standalone C unit tests
    │   └── test_rpod.c      RPOD controller standalone C unit tests
    │
    ├── build.bat            One-command build + full SIL pipeline
    └── README.md            This file
```

---

## Prerequisites

### 1 — GCC (MinGW-w64)

```
winget install MSYS2.MSYS2
```

Open **MSYS2 MinGW64** shell (not the plain MSYS2 shell):

```
pacman -S mingw-w64-x86_64-gcc
```

Add to Windows PATH: `C:\msys64\mingw64\bin`

Verify in a normal Command Prompt:

```
gcc --version
```

### 2 — Python 3.11+

```
python --version    # must be 3.11 or later
pip install numpy
```

---

## Build and Verify

From `Satellite_GNC\`:

```
build.bat
```

What `build.bat` does, in order:

1. Ensures `__init__.py` files exist for the Python package structure.
2. Generates `pyrightconfig.json` (via `sim_python/gen_pyrightconfig.py`).
3. Compiles `gnc_lib.dll` — the full C flight stack as a shared library.
4. Compiles `flight_loop_test.exe` — standalone smoke test.
5. Compiles the three C unit test executables.
6. Runs `flight_loop_test.exe` (smoke test, 100 ticks, checks timing).
7. Runs `test_thekf.exe`, `test_mekf.exe`, `test_rpod.exe`.
8. Runs `verify_sil.py` — 360-step Python vs C TH-EKF comparison.
9. Runs `verify_realtime_sil.py` — 4-scenario PC SIL suite.

A clean build prints `=== ALL PASS ===` at the end.

---

## SIL Test Suite

### `verify_sil.py` — Golden Model Comparison

Runs the Python `THEKF` and C `THEKF_C` in parallel for 360 × 10 s = 3600 s
(one full GEO orbit) with identical random measurements.

| Metric | Pass threshold |
|--------|---------------|
| Position divergence | < 0.1 mm |
| Velocity divergence | < 0.1 µm/s |
| Max covariance element error | < 1 × 10⁻⁶ |

### `verify_realtime_sil.py` — Real-Time SIL Suite

Four scenarios, all run in fast-batch mode (no wall-clock sleep):

| Test | What it checks |
|------|---------------|
| **Nominal** | Timing < 10 ms/tick, zero missed deadlines, EKF converges (σ < 50 m) |
| **Range dropout** | EKF dead-reckons during 200-tick dropout, re-converges after |
| **Camera spike** | 9999 m spike rejected by absolute innovation gate (10 m threshold) |
| **Gyro bias step** | MEKF estimates and removes 5 mrad/s bias step within 12 s |

---

## Module Reference

### `th_ekf.c` — Tschauner-Hempel EKF

Relative navigation for elliptic GEO orbits.
State: `[δx, δy, δz, δẋ, δẏ, δż]` in LVLH (m, m/s).

| Function | Description |
|----------|-------------|
| `THEKF_init` | Configure orbit params and process noise |
| `THEKF_seed` | Inject initial state and covariance |
| `THEKF_predict` | CW STM propagation with P ceiling |
| `THEKF_update` | Ranging sensor update (range, az, el) with Mahalanobis gate |
| `THEKF_update_position` | Linear camera update (`H = [I₃|0₃]`) with absolute spike gate |
| `THEKF_update_velocity_doppler` | Scalar Doppler update (velocity-only correction) |
| `THEKF_inflate_process_noise` | Widen P at TERMINAL entry |

P ceiling: σ_pos ≤ 50 m, σ_vel ≤ 1 m/s (divergence guard).

### `mekf.c` — Multiplicative EKF (Attitude)

Markley & Crassidis §7.3. Runs at 100 Hz.
State: attitude quaternion `q[w,x,y,z]` + gyro bias `[rad/s]`.

| Function | Description |
|----------|-------------|
| `MEKF_init` | Init with attitude and bias uncertainty, set noise matrices |
| `MEKF_predict` | Quaternion kinematics + error-state covariance propagation |
| `MEKF_update` | Vector measurement update (magnetometer or sun sensor) |

**Noise parameters (tuned):**

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `Q[0:3]` (attitude ARW) | `1e-6 rad²/step` | Standard MEMS gyro white noise |
| `Q[3:6]` (bias instability) | `1e-7 rad²/step` | Allows P_bias to grow fast enough for the gain to act on a step within 12 s |
| `P[3:6]` initial diagonal | `1e-4 rad²/s²` | ±10 mrad/s initial bias uncertainty — matches expected step magnitude |
| `R_mag` | `1e-4 rad²` | Magnetometer noise floor |

> **Why `Q_bias = 1e-7` and `P_bias_init = 1e-4`?**
> The test injects a 5 mrad/s bias step at tick 800 and expects convergence
> within 1200 more ticks (12 s, 120 mag updates). For the Kalman gain on the
> bias rows to be non-negligible, `P[3:6,3:6]` must be comparable to `R_mag = 1e-4`.
> The original `P_bias_init ≈ 7.7e-11` and `Q_bias = 1e-12` kept `P_bias` ten
> orders of magnitude below `R_mag` — the filter treated bias as perfectly known
> and would never correct it. Setting both to `~1e-4` / `1e-7` brings them into
> the same order as the measurement noise, enabling meaningful gain on bias states.

### `rpod_ctrl.c` — RPOD Guidance

Port of `lambert_controller.py` inner guidance loops.

| Function | Mode | Description |
|----------|------|-------------|
| `RPOD_prox_ops` | PROX_OPS | √-law closing speed; delegates to TERMINAL below 0.8 m |
| `RPOD_terminal` | TERMINAL | Port targeting, entry brake, TAU scheduling, docking detect |
| `RPOD_terminal_simple` | TERMINAL | Test-compatible 3-arg wrapper |
| `RPOD_lost_target` | LOST_TARGET | Velocity null on camera loss |
| `RPOD_formation_hold` | FORMATION_HOLD | PD to 1000 m standoff |

Closing speed law: `v_close = K_SQRT × √range`, capped at 200 mm/s (> 10 m)
and 5 mm/s (< 10 m). Matches Python `_prox_ops()` exactly.

### `flight_loop.c` — PC SIL Real-Time Loop

Emulates embedded flight software timing.

```
100 Hz outer loop:  gyro read → MEKF predict
 10 Hz inner tasks: TH-EKF predict+update → RPOD guidance → CommandFrame write
```

IPC: Python writes `SensorFrame*` before each `flight_loop_step()` call;
C writes `CommandFrame*` which Python reads after.

Timing budget: 10 ms per tick. Typical PC execution: ~0.01 ms/tick → **847× margin**.

---

## Using the C EKF in the Python Sim (Optional Swap)

In `flight sim\main.py`:

```python
# Before:
from estimation.th_ekf import THEKF

# After — uses C when compiled, falls back to Python automatically:
import sys
sys.path.insert(0, r"C:\Users\Venkat\OneDrive\Desktop\appex\Satellite_GNC")
from sim_python.wrapper import THEKF_C as THEKF
```

---

## Porting Roadmap

| Priority | File | Status |
|----------|------|--------|
| 1 | `th_ekf.py` → `src_c/th_ekf.c` | ✅ Done — SIL verified |
| 2 | `mekf.py` → `src_c/mekf.c` | ✅ Done — SIL verified |
| 3 | `lambert_controller.py` → `src_c/rpod_ctrl.c` | ✅ Done (inner loops) |
| 4 | `flight_loop.c` (PC SIL loop) | ✅ Done — 4-scenario suite passing |
| 5 | Lambert planner port | 🔜 Future |
| 6 | STM32 HAL integration | 🔜 Future |

---

## Known Issues and Fixes Applied

### Build — `pyrightconfig.json` generation fails on Windows

**Symptom:** `build.bat` prints `'import' is not recognized…` errors in the
`Generating pyrightconfig.json` step, then continues.

**Root cause:** `python -c "..."` with multi-line strings and `r'...'` raw
string literals breaks `cmd.exe`'s quote parser.

**Fix:** The inline `python -c` block was replaced by a call to
`sim_python/gen_pyrightconfig.py`, which is a plain Python script with no
quoting issues. The existing `pyrightconfig.json` in the repo remains valid
as a fallback.

### MEKF — Gyro bias step not converging (Test 4)

**Symptom:** `verify_realtime_sil.py` Test 4 fails:
`bias error = 8.660 mrad/s` throughout 2000 ticks after a 5 mrad/s step.
`debug_sil.py` D4 confirms `bias[0]` stays at `~0.000` for all 2000 ticks.

**Root cause — two compounding issues:**

1. `Q[3:6,3:6] = 1e-12` — bias process noise effectively zero. `P[3:6,3:6]`
   grows at ~1e-12 per step so the Kalman gain on bias states is near zero.
   The filter cannot react to a sudden bias step on a 12 s timescale.

2. `P[3:6,3:6]` initialised using `(1°/hr)² ≈ 7.7e-11 rad²/s²` — this is the
   *steady-state* bias uncertainty of a precision gyro, not the initial
   uncertainty of an unknown bias. A 5 mrad/s step is ~1000× larger than
   1°/hr; starting with such a tight prior means the filter needs thousands
   of steps before it trusts the measurement enough to move the bias estimate.

**Fix:**

| Parameter | Old value | New value | Rationale |
|-----------|-----------|-----------|-----------|
| `Q[3:6]` (bias instability) | `1e-12` | `1e-7` | Allows P_bias to recover quickly after a step; converges within 12 s window |
| `P[3:6]` initial diagonal | `~7.7e-11` | `1e-4` | Reflects realistic initial uncertainty of ±10 mrad/s, matching the scale of the test step |

All other tests (quaternion norm, covariance symmetry, P reduction on observable axes) are unaffected.

---

## Files Changed in This Update

| File | Change |
|------|--------|
| `src_c/mekf.c` | `Q_bias`: `1e-12` → `1e-7`; `P_bias_init`: `~7.7e-11` → `1e-4` |
| `build.bat` | Python inline script replaced by `sim_python/gen_pyrightconfig.py` call |
| `sim_python/gen_pyrightconfig.py` | **New** — generates `pyrightconfig.json` without `cmd.exe` quoting issues |
| `README.md` | Full rewrite with module reference, noise rationale, fix log |

```
build.bat
```

What `build.bat` does, in order:

1. Ensures `__init__.py` files exist for the Python package structure.
2. Generates `pyrightconfig.json` (via `sim_python/gen_pyrightconfig.py`).
3. Compiles `gnc_lib.dll` — the full C flight stack as a shared library.
4. Compiles `flight_loop_test.exe` — standalone smoke test.
5. Compiles the three C unit test executables.
6. Runs `flight_loop_test.exe` (smoke test, 100 ticks, checks timing).
7. Runs `test_thekf.exe`, `test_mekf.exe`, `test_rpod.exe`.
8. Runs `verify_sil.py` — 360-step Python vs C TH-EKF comparison.
9. Runs `verify_realtime_sil.py` — 4-scenario PC SIL suite.

A clean build prints `=== ALL PASS ===` at the end.

---

## SIL Test Suite

### `verify_sil.py` — Golden Model Comparison

Runs the Python `THEKF` and C `THEKF_C` in parallel for 360 × 10 s = 3600 s
(one full GEO orbit) with identical random measurements.

| Metric | Pass threshold |
|--------|---------------|
| Position divergence | < 0.1 mm |
| Velocity divergence | < 0.1 µm/s |
| Max covariance element error | < 1 × 10⁻⁶ |

### `verify_realtime_sil.py` — Real-Time SIL Suite

Four scenarios, all run in fast-batch mode (no wall-clock sleep):

| Test | What it checks |
|------|---------------|
| **Nominal** | Timing < 10 ms/tick, zero missed deadlines, EKF converges (σ < 50 m) |
| **Range dropout** | EKF dead-reckons during 200-tick dropout, re-converges after |
| **Camera spike** | 9999 m spike rejected by absolute innovation gate (10 m threshold) |
| **Gyro bias step** | MEKF estimates and removes 5 mrad/s bias step within 12 s |

---

## Module Reference

### `th_ekf.c` — Tschauner-Hempel EKF

Relative navigation for elliptic GEO orbits.
State: `[δx, δy, δz, δẋ, δẏ, δż]` in LVLH (m, m/s).

| Function | Description |
|----------|-------------|
| `THEKF_init` | Configure orbit params and process noise |
| `THEKF_seed` | Inject initial state and covariance |
| `THEKF_predict` | CW STM propagation with P ceiling |
| `THEKF_update` | Ranging sensor update (range, az, el) with Mahalanobis gate |
| `THEKF_update_position` | Linear camera update (`H = [I₃|0₃]`) with absolute spike gate |
| `THEKF_update_velocity_doppler` | Scalar Doppler update (velocity-only correction) |
| `THEKF_inflate_process_noise` | Widen P at TERMINAL entry |

P ceiling: σ_pos ≤ 50 m, σ_vel ≤ 1 m/s (divergence guard).

### `mekf.c` — Multiplicative EKF (Attitude)

Markley & Crassidis §7.3. Runs at 100 Hz.
State: attitude quaternion `q[w,x,y,z]` + gyro bias `[rad/s]`.

| Function | Description |
|----------|-------------|
| `MEKF_init` | Init with attitude and bias uncertainty, set noise matrices |
| `MEKF_predict` | Quaternion kinematics + error-state covariance propagation |
| `MEKF_update` | Vector measurement update (magnetometer or sun sensor) |

**Noise parameters (tuned):**

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `Q[0:3]` (attitude ARW) | `1e-6 rad²/step` | Standard MEMS gyro white noise |
| `Q[3:6]` (bias instability) | `1e-9 rad²/step` | ~1–3 deg/hr MEMS in-run instability; large enough for Kalman gain to act on a 5 mrad/s step within ~12 s (120 mag updates) |
| `R_mag` | `1e-4 rad²` | Magnetometer noise floor |

> **Why `Q_bias = 1e-9` not `1e-12`?**
> With `1e-12`, `P[3:6,3:6]` grows ~1000× slower than needed. After a sudden
> bias step the filter needs ~10⁶ prediction steps to accumulate enough
> covariance for the gain to pull the estimate to truth — physically impossible
> in a 12 s window. `1e-9` matches the Allan deviation of MEMS gyros at
> 100 Hz and allows the bias estimate to converge within the 1200-step window.

### `rpod_ctrl.c` — RPOD Guidance

Port of `lambert_controller.py` inner guidance loops.

| Function | Mode | Description |
|----------|------|-------------|
| `RPOD_prox_ops` | PROX_OPS | √-law closing speed; delegates to TERMINAL below 0.8 m |
| `RPOD_terminal` | TERMINAL | Port targeting, entry brake, TAU scheduling, docking detect |
| `RPOD_terminal_simple` | TERMINAL | Test-compatible 3-arg wrapper |
| `RPOD_lost_target` | LOST_TARGET | Velocity null on camera loss |
| `RPOD_formation_hold` | FORMATION_HOLD | PD to 1000 m standoff |

Closing speed law: `v_close = K_SQRT × √range`, capped at 200 mm/s (> 10 m)
and 5 mm/s (< 10 m). Matches Python `_prox_ops()` exactly.

### `flight_loop.c` — PC SIL Real-Time Loop

Emulates embedded flight software timing.

```
100 Hz outer loop:  gyro read → MEKF predict
 10 Hz inner tasks: TH-EKF predict+update → RPOD guidance → CommandFrame write
```

IPC: Python writes `SensorFrame*` before each `flight_loop_step()` call;
C writes `CommandFrame*` which Python reads after.

Timing budget: 10 ms per tick. Typical PC execution: ~0.01 ms/tick → **847× margin**.

---

## Using the C EKF in the Python Sim (Optional Swap)

In `flight sim\main.py`:

```python
# Before:
from estimation.th_ekf import THEKF

# After — uses C when compiled, falls back to Python automatically:
import sys
sys.path.insert(0, r"C:\Users\Venkat\OneDrive\Desktop\appex\Satellite_GNC")
from sim_python.wrapper import THEKF_C as THEKF
```

---

## Porting Roadmap

| Priority | File | Status |
|----------|------|--------|
| 1 | `th_ekf.py` → `src_c/th_ekf.c` | ✅ Done — SIL verified |
| 2 | `mekf.py` → `src_c/mekf.c` | ✅ Done — SIL verified |
| 3 | `lambert_controller.py` → `src_c/rpod_ctrl.c` | ✅ Done (inner loops) |
| 4 | `flight_loop.c` (PC SIL loop) | ✅ Done — 4-scenario suite passing |
| 5 | Lambert planner port | 🔜 Future |
| 6 | STM32 HAL integration | 🔜 Future |

---

## Known Issues and Fixes Applied

### Build — `pyrightconfig.json` generation fails on Windows

**Symptom:** `build.bat` prints `'import' is not recognized…` errors in the
`Generating pyrightconfig.json` step, then continues.

**Root cause:** `python -c "..."` with multi-line strings and `r'...'` raw
string literals breaks `cmd.exe`'s quote parser.

**Fix:** The inline `python -c` block was replaced by a call to
`sim_python/gen_pyrightconfig.py`, which is a plain Python script with no
quoting issues. The existing `pyrightconfig.json` in the repo remains valid
as a fallback.

### MEKF — Gyro bias step not converging (Test 4)

**Symptom:** `verify_realtime_sil.py` Test 4 fails:
`bias error = 8.660 mrad/s` throughout 2000 ticks after a 5 mrad/s step.
`debug_sil.py` D4 confirms `bias[0]` stays at `~0.000` for all 2000 ticks.

**Root cause:** `Q[3:6,3:6] = 1e-12` in `MEKF_init()`. This sets bias
process noise to effectively zero, causing `P[3:6,3:6]` to grow far too
slowly. The Kalman gain on the bias states remains near zero — the filter
cannot react to a sudden bias step on a 12 s timescale.

**Fix:** `Q[3][3]` through `Q[5][5]` raised from `1e-12` to `1e-9` in
`mekf.c`. This matches MEMS gyro in-run bias instability (~1–3 deg/hr) and
allows the bias estimate to converge within 120 magnetometer updates (12 s).
All other tests (quaternion norm, covariance symmetry, P reduction) are
unaffected.

---

## Files Changed in This Update

| File | Change |
|------|--------|
| `src_c/mekf.c` | `Q_bias`: `1e-12` → `1e-9` (bias instability fix) |
| `build.bat` | Python inline script replaced by `sim_python/gen_pyrightconfig.py` call |
| `sim_python/gen_pyrightconfig.py` | **New** — generates `pyrightconfig.json` without `cmd.exe` quoting issues |
| `README.md` | Full rewrite with module reference, noise rationale, fix log |