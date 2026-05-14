# Satellite GNC

Embedded C guidance, navigation, and control software for a GEO rendezvous, proximity operations, and docking (RPOD) simulation.

This repository contains the C flight-software side of the project, plus Python software-in-the-loop (SIL) harnesses that drive the C code against a simulated spacecraft plant. The original high-level algorithm development lives in the companion Python flight simulation; this repo is the embedded-port and verification workspace.

## Current Status

The current build passes the full C unit test and Python SIL sequence.

Latest verified closed-loop result:

- RPOD enters terminal at about 6110 s.
- Dock contact is latched at about 6125.5 s.
- First DOCKED report is at about 6126 s.
- Final reported contact range is about 0.21 m.
- The full build ends with `ALL PASS`.

The latest build log is saved under:

```text
build_results/run_YYYYMMDD_HHMMSS/build.log
```

The current run writes the newest path to:

```text
build_results/latest_path.txt
```

## Repository Layout

```text
Satellite_GNC/
  build.bat                  Windows build and verification entry point
  src_c/                     Embedded C flight-software modules
    flight_loop.*            Top-level fixed-rate FSW loop
    sensor_packet.h          Sensor-frame ABI shared with Python ctypes
    command_packet.h         Command-frame ABI shared with Python ctypes
    th_ekf.*                 Relative navigation EKF
    mekf.*                   Attitude MEKF
    quest.*                  QUEST attitude determination
    adcs.*                   B-dot, reaction wheel, MTQ, attitude control
    mode_manager.*           FSW mode transitions
    rpod_ctrl.*              PROX_OPS and TERMINAL RPOD guidance
    lambert_solver.*         Lambert solver utilities
    chief_pose_estimator.*   Chief pose estimator prototype
  sim_python/                SIL drivers and Python wrappers around C DLL
    wrapper.py               ctypes wrapper for standalone C modules
    realtime_driver.py       100 Hz flight-loop driver
    verify_sil.py            C/Python TH-EKF parity check
    verify_realtime_sil.py   Timing, dropout, spike, and gyro-bias tests
    closed_loop_sil.py       C flight loop closed around Python plant
  tests/                     C unit tests
```

## Flight Software Architecture

The top-level C flight loop runs at 100 Hz. Lower-rate navigation and guidance tasks run at 10 Hz inside that loop.

```text
Python plant / sensors
        |
        v
SensorFrame
        |
        v
flight_loop_step()
        |
        +-- MEKF attitude propagation and vector updates
        +-- TH-EKF relative navigation propagation and measurement updates
        +-- Mode manager
        +-- ADCS control
        +-- RPOD guidance
        |
        v
CommandFrame
        |
        v
Python plant applies accel, wheel torque, and MTQ dipole
```

The interface between Python and C is intentionally plain C data structures:

- `SensorFrame` carries gyro, range, camera, docking-port, magnetometer, and sun-sensor data.
- `CommandFrame` carries navigation state, attitude state, guidance commands, and timing telemetry.

This keeps the SIL boundary close to what an embedded packet interface would look like.

## Implemented Modules

### Relative Navigation

`th_ekf.c` implements a Tschauner-Hempel style relative navigation EKF for GEO proximity operations.

It supports:

- state propagation with CW/TH-style dynamics,
- range, azimuth, and elevation measurement updates,
- close-range camera position updates,
- covariance tracking and gating.

The build compares this C implementation against the Python reference model.

### Attitude Estimation

`mekf.c` implements a quaternion MEKF with gyro-bias estimation.

`quest.c` implements QUEST/TRIAD-style attitude initialization from vector observations.

The attitude stack is tested for:

- quaternion norm preservation,
- covariance symmetry,
- vector update behavior,
- gyro-bias recovery.

### ADCS

`adcs.c` contains:

- B-dot detumbling,
- reaction-wheel torque command handling,
- magnetorquer momentum dumping,
- PD attitude control.

The closed-loop SIL uses a plant-side torque clamp so the C wheel momentum estimate and the simulated physical torque remain consistent.

### Mode Manager

`mode_manager.c` handles the main flight-software modes:

```text
DETUMBLE -> SUN_ACQUISITION -> FINE_POINTING -> MOMENTUM_DUMP
```

It also handles safe-mode recovery guards and momentum-dump entry/exit logic.

### RPOD Guidance

`rpod_ctrl.c` contains the C port of the Python RPOD guidance inner loop.

Current guidance phases:

- `PROX_OPS`: range-based approach using a sqrt closing-speed law.
- `TERMINAL`: close-range terminal approach with speed limiting, entry braking, and docking detection.
- `DOCKED`: latched in the flight loop once docking is declared.

The terminal path can accept an explicit docking-port measurement through `SensorFrame.port`. If the port packet is invalid, the controller falls back to center-of-mass targeting.

## Build And Test

### Prerequisites

On Windows:

- Python available as `python`
- GCC available on `PATH`
- PowerShell available for timestamped log creation

The build also expects the companion Python simulation directory at:

```text
C:\Users\Venkat\OneDrive\Desktop\appex\flight sim
```

That path is used by the SIL harness for reference Python models.

### Full Build

From the repo root:

```bat
build.bat
```

The script performs:

1. Python package setup for SIL imports.
2. `pyrightconfig.json` generation.
3. C DLL build: `gnc_lib.dll`.
4. Standalone flight-loop smoke-test build.
5. C unit-test builds.
6. C unit-test execution.
7. Python/C EKF parity check.
8. Real-time SIL verification.
9. Closed-loop SIL verification.

The console output is saved automatically to a timestamped log:

```text
build_results/run_YYYYMMDD_HHMMSS/build.log
```

### Running Individual Tests

After `build.bat` compiles the executables, individual C tests can be run directly:

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

Python SIL tests can also be run directly:

```bat
python sim_python\verify_sil.py
python sim_python\verify_realtime_sil.py
python sim_python\closed_loop_sil.py
```

## Verification Coverage

### C Unit Tests

The C tests cover:

- TH-EKF propagation and update behavior,
- MEKF quaternion and covariance behavior,
- RPOD proximity and terminal logic,
- QUEST attitude determination,
- ADCS actuator/control primitives,
- mode-manager transitions,
- Lambert solver utilities,
- chief pose estimator sanity checks.

### Python/C SIL Parity

`verify_sil.py` compares the C TH-EKF against the Python reference model.

The latest verified run shows near machine-precision agreement for state and covariance propagation.

### Real-Time SIL

`verify_realtime_sil.py` checks:

- loop timing margin,
- missed deadlines,
- range-sensor dropout handling,
- camera spike rejection,
- gyro-bias step recovery.

### Closed-Loop SIL

`closed_loop_sil.py` closes the C flight loop around a Python plant.

The current build-gate scenarios are:

1. Detumble and sun acquisition.
2. RPOD closure from 500 m to dock contact.
3. Long fine-pointing / momentum-dump guard behavior.

For the RPOD closure scenario, the harness now stops when the C flight loop latches DOCKED. This avoids reporting meaningless post-contact free-flight drift as the final range.

## Current Design Notes

### Docking Latch

Docking is treated as an absorbing state in the C flight loop. Once `RPOD_terminal()` reports docked, `flight_loop.c` latches DOCKED and commands zero translational acceleration.

This prevents mode chatter caused by EKF range jitter around the docking threshold.

### Docking Threshold

The current docking-complete threshold is:

```c
RPOD_DOCK_DONE_M = 0.20
```

The capture sphere is:

```c
RPOD_DOCK_RANGE_M = 0.30
```

### Dock-Port Measurement

The C sensor ABI includes a `PortPacket`:

```c
typedef struct {
    double  port_lvlh[3];
    double  R_diag[3];
    uint8_t valid;
    uint8_t _pad[7];
} PortPacket;
```

If `sf->port.valid` is true, terminal guidance targets the reported docking port. If false, terminal guidance falls back to the chief center of mass.

## Known Limitations

- The current build-gate RPOD closure is translational. Full 6-DOF contact dynamics are not yet modeled.
- The SIL plant stops at dock contact rather than simulating hard/soft capture mechanics.
- Dock-port measurement is present in the C interface, but the build-gate SIL still uses simple close-range behavior.
- The C RPOD terminal constants should be kept in sync with the cleaned Python terminal approach law.
- The build script currently contains a user-specific path to the companion Python simulation.
- The current SIL is a PC validation environment, not an MCU/HIL deployment build.

## Recommended Next CDR Items

1. Mirror the cleaned Python terminal constants in `rpod_ctrl.h`.
2. Feed a realistic noisy dock-port measurement into `closed_loop_sil.py`.
3. Add a test that verifies `SensorFrame.port` changes terminal targeting.
4. Separate PC SIL build settings from embedded target build settings.
5. Replace hard-coded local paths with an environment variable or config file.
6. Add a short HIL-oriented packet specification for `SensorFrame` and `CommandFrame`.

## Notes For Pushing

Build outputs are generated locally:

- `gnc_lib.dll`
- `*.exe`
- `build_results/`
- `pyrightconfig.json`

Only source, tests, and documentation should be committed unless a generated artifact is intentionally being shared.

## Author

Venkat Sainath  
MSc Space Engineering

