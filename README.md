# Satellite GNC Embedded C

Embedded C guidance, navigation, and control software for a GEO rendezvous, proximity operations, and docking (RPOD) stack. This repository is the flight-software/SIL port of the companion Python simulation in `C:\Users\Venkat\OneDrive\Desktop\appex\flight sim`.

The intent is simple: keep the Python model as the high-fidelity design and Monte Carlo environment, and keep this repository as the embedded-C implementation that can be built, unit-tested, and driven in software-in-the-loop.

## Current Verification Status

Latest verified C build: `build_results/run_20260516_231455/build.log`

Result: `=== ALL PASS ===`

The build completed:

- C DLL build for `gnc_lib.dll`
- C unit tests for navigation, attitude, ADCS, mode management, Lambert utilities, chief pose, and RPOD
- Python/C TH-EKF parity checks
- real-time SIL timing/fault-injection checks
- closed-loop SIL scenarios for detumble, acquisition, proximity ops, and long fine-pointing guard behavior

Build outputs and logs are local artifacts. Do not push `build_results/`, generated executables, DLLs, or generated Python config files unless intentionally sharing a run artifact.

## Repository Layout

```text
Satellite_GNC/
  build.bat                  Windows build and verification entry point
  src_c/
    flight_loop.*            Top-level fixed-rate flight-software loop
    sensor_packet.h          SensorFrame ABI shared with Python ctypes
    command_packet.h         CommandFrame ABI shared with Python ctypes
    th_ekf.*                 Relative navigation EKF
    mekf.*                   Attitude MEKF
    quest.*                  QUEST/TRIAD attitude initialization
    adcs.*                   B-dot, reaction wheel, MTQ, attitude control
    mode_manager.*           FSW mode transitions and safe-mode guards
    rpod_ctrl.*              PROX_OPS, TERMINAL, LOST_TARGET RPOD guidance
    terminal_filter.*         Terminal relative-nav smoothing
    port_tracker.*           Docking-port measurement gate and short-coast tracker
    lambert_solver.*         Lambert solver utilities
    chief_pose_estimator.*   Chief pose estimator prototype
  sim_python/
    wrapper.py               ctypes wrapper for C modules
    realtime_driver.py       100 Hz flight-loop driver
    verify_sil.py            Python/C TH-EKF parity check
    verify_realtime_sil.py   Timing, dropout, spike, and gyro-bias tests
    closed_loop_sil.py       C flight loop closed around Python plant
  tests/                     C unit tests
  build_results/             Generated logs, ignored for normal commits
```

## Architecture

The C flight loop runs at 100 Hz. Navigation and guidance logic run as lower-rate tasks inside that loop, while Python supplies the simulated plant and sensor packets during SIL.

```text
Python plant / truth model
        |
        v
SensorFrame
        |
        v
flight_loop_step()
        |
        +-- MEKF attitude propagation and vector updates
        +-- TH-EKF relative navigation propagation and measurement updates
        +-- FSW mode manager
        +-- ADCS control
        +-- RPOD guidance and FDIR guards
        |
        v
CommandFrame
        |
        v
Python plant applies commanded accel, wheel torque, and MTQ dipole
```

The interface is plain C by design:

- `SensorFrame` carries gyro, range, camera, docking-port, magnetometer, sun-sensor, and fault inputs.
- `CommandFrame` carries navigation state, attitude state, actuator commands, RPOD commands, timing, and mode telemetry.

That keeps the SIL boundary close to a flight packet interface rather than a Python-only function call.

## Implemented Flight Software

### Relative Navigation

`th_ekf.c` implements the C relative navigation filter used by the flight loop. It supports propagation, range/angle updates, close-range camera position updates, covariance tracking, and innovation gating. The build checks it against the Python reference model.

### Attitude Estimation

`mekf.c` implements quaternion attitude estimation with gyro-bias handling. `quest.c` provides vector-based attitude initialization. Unit tests cover quaternion norm behavior, covariance sanity, vector updates, and bias response.

### ADCS

`adcs.c` includes B-dot detumble, reaction-wheel torque command handling, magnetorquer momentum dumping, and fine-pointing PD control. The SIL plant clamps torque consistently with the C actuator model so the commanded and simulated dynamics stay aligned.

### Mode Management

`mode_manager.c` handles:

```text
DETUMBLE -> SUN_ACQUISITION -> FINE_POINTING -> MOMENTUM_DUMP
```

It also includes safe-mode and recovery guard logic for excessive angular rate and external fault injection.

### RPOD Guidance

`rpod_ctrl.c` implements the embedded version of the Python RPOD law.

Current phases:

- `FORMATION_HOLD`: station keeping before RPOD activation.
- `PROX_OPS`: translational closure with a square-root closing-speed law.
- `TERMINAL`: close-range docking-port approach with filtered terminal navigation.
- `LOST_TARGET`: velocity-null hold when terminal camera validity is lost for the debounce window.
- `DOCKED`: latched in `flight_loop.c` after successful docking confirmation.

Current terminal tuning mirrors the cleaned Python simulation:

```c
RPOD_TERMINAL_M          = 5.0      // PROX_OPS -> TERMINAL handoff
RPOD_V_TERM_MAX_MS       = 0.025    // 25 mm/s terminal max command
RPOD_V_APPROACH_MS       = 0.010    // 10 mm/s below 0.8 m
RPOD_V_CAPTURE_MS        = 0.005    // 5 mm/s below 0.3 m
RPOD_DOCK_RANGE_M        = 0.30     // capture zone
RPOD_DOCK_DONE_M         = 0.20     // docking-complete range
RPOD_DOCK_MAX_SPEED_MS   = 0.050    // 50 mm/s docking speed gate
```

## Terminal Filtering And FDIR

The terminal C path now includes lightweight filters added in Python:

- `terminal_filter.c/.h`: terminal alpha-beta relative navigation filter
- `port_tracker.c/.h`: docking-port position tracker with short coast capability
- innovation gating for terminal measurements
- velocity clamp on terminal filtered velocity
- camera-loss debounce before `LOST_TARGET`

These are intentionally small flight-software filters, not a new full EKF. They smooth the final few meters and prevent one noisy camera/port sample from dominating the command.

Terminal camera validity behavior:

- valid camera: update terminal nav filter and continue terminal guidance
- short invalid camera: coast/hold through the filter
- invalid camera for 2 s: enter `LOST_TARGET` and command velocity-null hold
- recovered camera: reset terminal filter and resume guidance

## Build And Test

Prerequisites on Windows:

- Python available as `python`
- GCC available on `PATH`
- PowerShell available for timestamped logging
- companion Python simulation at `C:\Users\Venkat\OneDrive\Desktop\appex\flight sim`

Run the full verification pipeline from the repo root:

```bat
cmd /c build.bat
```

The script builds the C DLL and test executables, runs unit tests, then runs Python/C SIL checks. Logs are saved under:

```text
build_results/run_YYYYMMDD_HHMMSS/build.log
```

After a build, individual tests can be run directly:

```bat
test_thekf.exe
test_mekf.exe
test_rpod.exe
test_quest.exe
test_adcs.exe
test_mode_manager.exe
test_lambert.exe
test_chief_pose.exe
```

Python SIL checks can also be run directly:

```bat
python sim_python\verify_sil.py
python sim_python\verify_realtime_sil.py
python sim_python\closed_loop_sil.py
```

## CDR Notes

Implemented CDR-grade items:

- Python/C TH-EKF parity check
- 100 Hz flight-loop timing telemetry
- closed-loop SIL with C commands driving a Python plant
- RPOD terminal docking-port targeting
- DOCKED latch after terminal contact
- docking speed gate
- terminal navigation filter and port tracker
- terminal camera-loss debounce and `LOST_TARGET` hold
- generated build logs for traceability

FDIR/FMECA status:

- safe-mode guard exists for high angular rate and external fault input
- sensor innovation gates exist in navigation and terminal filters
- actuator commands are limited
- camera dropout and spike behavior are exercised in SIL
- deadline counters are reported
- central fault-word management is not complete yet

Known gaps before calling this flight-grade:

- central fault manager with latched fault words and severity levels
- sensor freshness counters for every packet field
- finite/NaN checks and covariance bound checks at the packet boundary
- multi-sample docking confirmation
- approach-axis and attitude-alignment gates for final docking
- explicit retreat/abort strategy after prolonged terminal target loss
- separate embedded target build from PC/SIL build
- full HIL packet specification

## Push Hygiene

Commit source, tests, and documentation. Do not commit generated artifacts from a local build:

- `build_results/`
- `*.exe`
- `*.dll`
- `*.o` / `*.obj`
- `pyrightconfig.json`

If `git status` shows tracked build metadata such as `build_results/latest_path.txt`, restore or unstage it before pushing unless you intentionally want that local run pointer in the commit.

## Author

Venkat Sainath  
MSc Space Engineering



