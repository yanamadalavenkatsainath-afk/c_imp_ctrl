# Satellite GNC Embedded C

Embedded C guidance, navigation, and control software for GEO rendezvous,
proximity operations, and docking. This repository is the C/SIL/HIL-facing
flight-software port of the Python reference simulation in:

```text
C:\Users\Venkat\OneDrive\Desktop\appex\flight sim
```

The C side owns the flight-loop state, estimators, mode transitions, RPOD
guidance, FDIR guards, packet validation, actuator command limits, and telemetry
ABI. Python supplies the plant and sensor packets during SIL.

> Status: engineering prototype flight software. It is not flight-certified.

## Current Status

Latest verified desktop SIL build:

```text
build_results\run_20260606_213857\build.log
```

Result:

```text
=== ALL PASS ===
```

The current C stack includes:

- 100 Hz `flight_loop_step()` with 10 Hz navigation/RPOD tasks
- TH-EKF relative navigation
- MEKF attitude estimation, including star-tracker quaternion update
- QUEST attitude initialization
- B-dot, reaction wheel, magnetorquer, and attitude PD control
- Mode manager and safe-mode transitions
- RPOD PROX_OPS, TERMINAL, LOST_TARGET, SOFT_CAPTURE, and DOCKED logic
- Dock-port/flash-lidar terminal handover through `PortPacket`
- Chief pose estimator and spin-sync controller
- Packet version/checksum/timestamp validation for every sensor packet
- Watchdog flags for invalid/stale packets, frame mismatch, safe fallback, and output inhibit
- Final actuator output clamps at the command boundary
- Target-safe logging macro via `target_port.h`

The true target build/HIL steps are scaffolded but require hardware/toolchain:

- `build_target.bat` expects `arm-none-eabi-gcc` or `ARM_GCC`
- `sim_python\hil_serial_runner.py` is the Python-plant to target-hardware serial bridge

## Repository Layout

```text
Satellite_GNC/
  build.bat                    desktop C build + full SIL verification
  build_target.bat             target compiler object/archive build
  src_c/
    flight_loop.*              top-level fixed-rate flight-software loop
    sensor_packet.h            SensorFrame ABI, packet metadata, checksums
    command_packet.h           CommandFrame ABI, watchdog/status telemetry
    target_port.h              target-safe logging shim
    th_ekf.*                   relative navigation EKF
    mekf.*                     attitude MEKF
    quest.*                    QUEST attitude initialization
    adcs.*                     B-dot, RW, MTQ, attitude controller
    mode_manager.*             FSW mode transitions
    rpod_ctrl.*                RPOD guidance and capture logic
    terminal_filter.*          terminal alpha-beta nav smoothing
    port_tracker.*             docking-port tracker
    spin_sync_controller.*     target spin synchronization
    chief_pose_estimator.*     chief attitude/omega estimator
  sim_python/
    realtime_driver.py         100 Hz ctypes SIL driver
    closed_loop_sil.py         C flight loop closed around Python plant
    sil_maturity_suite.py      fault-injection regression matrix
    sil_soak.py                long-duration desktop soak
    cil_mc.py                  deterministic C-in-loop MC/replay
    hil_serial_runner.py       Python plant <-> target serial runner
  tests/                       C unit tests
  tools/
    analyze_stack_usage.py     parser for GCC -fstack-usage output
```

## Flight Loop ABI

Python fills `SensorFrame`; C returns `CommandFrame`.

Each valid sensor packet now carries:

```c
typedef struct {
    uint32_t version;
    uint32_t checksum;
    uint64_t timestamp_tick;
} PacketMeta;
```

The C flight loop rejects a packet if:

- `version != SENSOR_PACKET_VERSION`
- checksum does not match the payload
- packet timestamp is in the future
- packet age exceeds its freshness window

The command packet reports:

- loop timing and missed deadlines
- watchdog bitfield
- invalid packet count
- stale sensor count
- output inhibit state
- RPOD/capture telemetry

## Build And Verification

Desktop SIL build:

```bat
cmd /c build.bat
```

This builds `gnc_lib.dll`, compiles all unit tests, then runs:

- C unit tests
- Python/C SIL parity tests
- realtime SIL tests
- SIL maturity fault matrix
- long-duration soak
- closed-loop SIL

C-in-loop Monte Carlo:

```bat
python sim_python\cil_mc.py --trials 20
python sim_python\cil_mc.py --replay-seed 4200
```

Target compiler build:

```bat
cmd /c build_target.bat
```

If `arm-none-eabi-gcc` is not on `PATH`, set:

```bat
set ARM_GCC=C:\path\to\arm-none-eabi-gcc.exe
cmd /c build_target.bat
```

The target build compiles with:

- `-DTARGET_BUILD`
- `-ffunction-sections`
- `-fdata-sections`
- `-fstack-usage`

and summarizes generated `.su` stack-usage files.

## HIL Entry Point

The serial HIL host runner is:

```bat
python sim_python\hil_serial_runner.py --port COM5 --baud 921600 --ticks 6000
```

Expected target-side protocol:

```text
Host -> target:
  magic 0xA55A5AA5, uint16 payload_len, SensorFrame bytes

Target -> host:
  magic 0x5AA5A55A, uint16 payload_len, CommandFrame bytes
```

The target firmware still needs the board-specific wrapper that:

1. receives and validates the transport frame,
2. copies the payload into `SensorFrame`,
3. calls `flight_loop_step()`,
4. serializes `CommandFrame`,
5. sends it back over the real comms link.

## Hardware Readiness Checklist

Implemented in this repo:

- packet version/checksum fields
- per-sensor timestamp/freshness rejection
- watchdog flags and counters
- safe fallback/output inhibit on prolonged gyro loss
- final output clamps for accel, RW torque, and MTQ dipole
- target-safe `GNC_LOG` macro
- target build script and stack-usage parser
- serial HIL host runner
- deterministic CIL replay/MC scripts
- fault-injection SIL matrix

Still requires actual target hardware:

- compile with the board's flight compiler/toolchain
- link with startup, linker script, HAL, and board comms driver
- run on the MCU/flight computer
- measure real 100 Hz and 10 Hz jitter
- confirm flash/RAM/stack with the final linker map
- measure worst-case execution time on target
- run Python plant <-> target hardware closed loop over the real comms link

## Git Hygiene

Commit source, tests, scripts, and docs. Avoid committing local generated files
unless intentionally archiving a run package:

```text
build_results/
*.exe
*.dll
*.o
*.obj
target_build/
cil_mc_results*.json
```

## Author

Venkat Sainath

MSc Space Engineering
