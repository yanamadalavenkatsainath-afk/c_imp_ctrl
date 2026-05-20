# Satellite GNC Embedded C

Embedded C guidance, navigation, and control software for a GEO rendezvous, proximity operations, and docking (RPOD) stack. The repository contains the flight-software/SIL implementation of the RPOD, relative navigation, attitude estimation, ADCS, and mode-management logic used for a 50 kg GEO servicing deputy concept.

The code is structured as a portable C core with a Python software-in-the-loop harness. Python supplies the simulated plant and sensor packets during SIL; the C side owns the flight-loop state, estimators, mode transitions, guidance commands, actuator limits, and FDIR guards.

> Status: research/prototype flight software. This is not flight-certified code.

## Verification Status

Latest local verification run: `build_results/run_20260520_175716/build.log`

Result: `=== ALL PASS ===`

The verification pipeline completed:

- C DLL build for `gnc_lib.dll`
- C unit tests for relative navigation, MEKF, QUEST, ADCS, mode management, Lambert utilities, chief pose estimation, and RPOD guidance
- Python/C TH-EKF parity checks
- real-time SIL timing and fault-injection checks
- closed-loop SIL scenarios for detumble, acquisition, proximity operations, terminal soft capture, docking, and long fine-pointing guard behavior

Build outputs are local artifacts and are intentionally ignored for normal commits.

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
    rpod_ctrl.*              PROX_OPS, TERMINAL, SOFT_CAPTURE, LOST_TARGET RPOD guidance
    terminal_filter.*        Terminal relative-nav smoothing
    port_tracker.*           Docking-port measurement gate and short-coast tracker
    lambert_solver.*         Lambert solver utilities
    chief_pose_estimator.*   Chief pose estimator prototype
  sim_python/
    wrapper.py               ctypes wrapper for C modules
    realtime_driver.py       100 Hz flight-loop driver
    verify_sil.py            Python/C TH-EKF parity check
    verify_realtime_sil.py   Timing, dropout, spike, and gyro-bias tests
    closed_loop_sil.py       C flight loop closed around a Python plant
  tests/                     C unit tests
  build_results/             Generated logs, ignored for normal commits
```

## Architecture

The C flight loop runs at 100 Hz. Navigation and guidance tasks execute inside that loop at their configured rates, while SIL drivers provide simulated sensor packets and consume command packets.

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

The C/Python interface is intentionally packet-like:

- `SensorFrame` carries gyro, range, camera, docking-port, magnetometer, sun-sensor, and fault inputs.
- `CommandFrame` carries navigation state, attitude state, actuator commands, RPOD commands, timing, and mode telemetry.

## Implemented Flight Software

### Relative Navigation

`th_ekf.c` implements the relative navigation filter used by the flight loop. It supports propagation, range/angle updates, close-range camera position updates, covariance tracking, and innovation gating. The SIL pipeline checks it against the Python reference model.

### Attitude Estimation

`mekf.c` implements quaternion attitude estimation with gyro-bias handling. `quest.c` provides vector-based attitude initialization. Unit tests cover quaternion norm behavior, covariance sanity, vector updates, and bias response.

### ADCS

`adcs.c` includes B-dot detumble, reaction-wheel torque command handling, magnetorquer momentum dumping, and fine-pointing PD control. The SIL plant clamps torque consistently with the C actuator model so commanded and simulated dynamics stay aligned.

### Mode Management

`mode_manager.c` handles the primary ADCS mode chain:

```text
DETUMBLE -> SUN_ACQUISITION -> FINE_POINTING -> MOMENTUM_DUMP
```

It also includes safe-mode and recovery guard logic for excessive angular rate and external fault injection.

### RPOD Guidance

`rpod_ctrl.c` implements the embedded RPOD guidance law and mirrors the Python terminal-contact sequence closely enough for SIL parity.

Current phases:

- `FORMATION_HOLD`: station keeping before RPOD activation
- `PROX_OPS`: translational closure with a square-root closing-speed law
- `TERMINAL`: close-range docking-port approach with filtered terminal navigation
- `SOFT_CAPTURE`: compliant post-contact port hold before hard-capture latch
- `LOST_TARGET`: velocity-null hold when terminal camera validity is lost for the debounce window
- `DOCKED`: latched in `flight_loop.c` after successful docking confirmation

Current terminal tuning:

```c
RPOD_TERMINAL_M          = 5.0      // PROX_OPS -> TERMINAL handoff
RPOD_V_TERM_MAX_MS       = 0.025    // 25 mm/s terminal max command
RPOD_V_APPROACH_MS       = 0.010    // 10 mm/s below 0.8 m
RPOD_V_CAPTURE_MS        = 0.005    // 5 mm/s below 0.3 m
RPOD_DOCK_RANGE_M        = 0.30     // soft-capture port gate
RPOD_DOCK_DONE_M         = 0.20     // legacy capture alias
RPOD_DOCK_MAX_SPEED_MS   = 0.050    // 50 mm/s docking speed gate
RPOD_HARD_CAPTURE_RANGE_M = 0.08    // hard-capture latch range
RPOD_HARD_CAPTURE_VREL_MS = 0.010   // hard-capture latch speed
RPOD_HARD_CAPTURE_HOLD_S  = 5.0     // latch dwell time
RPOD_DOCK_CONE_HALF_ANGLE_DEG = 15.0
RPOD_DOCK_ALIGN_MAX_DEG       = 10.0
```

Terminal capture uses the docking-port frame, not just chief-COM range. The C state includes port range, port velocity, port axis, approach-cone error, lateral aperture error, and attitude-alignment fields. `RPOD_terminal()` returns `RPOD_RET_SOFT_CAPTURE_READY` when the soft-capture gate is met. `flight_loop.c` then enters soft capture and latches `DOCKED` after the hard-capture hold.

The soft-capture controller is intentionally lightweight: a small spring-damper holds the deputy at the port until the hard-capture range and velocity criteria are stable. This matches the Python simulation's current soft/hard capture abstraction without pretending to model detailed mechanical latch hardware.

## Terminal Filtering And FDIR

The terminal C path includes two small flight-software filters:

- `terminal_filter.c/.h`: alpha-beta relative navigation smoothing in the terminal phase
- `port_tracker.c/.h`: docking-port measurement gate and short-coast tracker

Implemented terminal guards:

- terminal measurement innovation gating
- filtered velocity clamp
- camera-loss debounce before `LOST_TARGET`
- velocity-null hold during `LOST_TARGET`
- terminal filter reset after target reacquisition
- finite chief-body / docking-port geometry gate
- approach-cone gate
- hard-capture dwell confirmation after soft capture

These are intentionally lightweight filters, not a second full EKF. They smooth the final few meters and prevent one noisy terminal measurement from dominating the command.

## Build And Test

Prerequisites on Windows:

- Python available as `python`
- GCC available on `PATH`
- PowerShell available for timestamped logging
- Python SIL dependencies installed for the scripts under `sim_python/`

Run the full verification pipeline from the repository root:

```bat
cmd /c build.bat
```

The script builds the C DLL and test executables, runs unit tests, then runs Python/C SIL checks. Logs are generated under:

```text
build_results/run_YYYYMMDD_HHMMSS/build.log
```

Individual C tests can be run directly after building:

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

## CDR / FDIR Status

Implemented CDR-grade items:

- Python/C TH-EKF parity check
- 100 Hz flight-loop timing telemetry
- closed-loop SIL with C commands driving a Python plant
- RPOD terminal docking-port targeting
- soft-capture and hard-capture state sequence
- docking range, speed, approach-cone, and geometry gates
- terminal navigation filter and port tracker
- terminal camera-loss debounce and `LOST_TARGET` hold
- generated build logs for traceability

Implemented FDIR/FMECA coverage:

- safe-mode guard for high angular rate and external fault input
- sensor innovation gates in navigation and terminal filters
- actuator command limiting
- camera dropout and spike tests in SIL
- flight-loop deadline telemetry

Known gaps before flight-grade maturity:

- central fault manager with latched fault words and severity levels
- sensor freshness counters for every packet field
- finite/NaN checks and covariance bound checks at packet boundaries
- target-hardware build separate from PC/SIL build
- explicit retreat/abort strategy after prolonged terminal target loss
- HIL packet specification and hardware timing validation

## Git Hygiene

Commit source, tests, and documentation. Do not commit generated local artifacts unless intentionally publishing a run package:

- `build_results/`
- `*.exe`
- `*.dll`
- `*.o` / `*.obj`
- generated Python config files

If a tracked local build pointer changes, restore or unstage it before pushing unless it is intentionally part of the commit.

## Author

Venkat Sainath

MSc Space Engineering
