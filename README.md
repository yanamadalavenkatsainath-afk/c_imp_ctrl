# Satellite GNC — Autonomous RPOD Flight Software (C + SIL)

## Overview

This repository contains a **full-stack spacecraft Guidance, Navigation, and Control (GNC) system** for autonomous rendezvous and docking (RPOD), implemented in **C flight code** and validated against a **Python high-fidelity simulation**.

The system targets **non-cooperative GEO servicing scenarios**, including proximity operations and terminal docking under uncertainty.

This is not a component-level implementation — it is a **closed-loop, system-level autonomy stack**.

---

## System Scope

The implemented stack covers:

- Attitude estimation and control (ADCS)
- Relative navigation (EKF-based)
- Proximity operations guidance
- Terminal docking control
- Flight software mode management
- Real-time execution loop

The architecture is designed for:
- deterministic execution
- embedded deployment
- hardware-in-the-loop integration

---

## Current Implementation Status

### Navigation

- **TH-EKF (Tschauner–Hempel)**  
  Full relative orbit estimator for GEO proximity operations

- **MEKF (Multiplicative EKF)**  
  Quaternion-based attitude estimation with bias tracking

- **QUEST**  
  Attitude determination from vector observations

---

### Control (ADCS)

- **B-dot detumbling**
- **Attitude PD controller**
- **Reaction wheel torque application**
- **Magnetorquer-based momentum dumping**

These operate as a **hybrid ADCS system**, switching based on mode.

---

### Guidance (RPOD)

- **PROX_OPS phase**
  - sqrt-law closing velocity profile
  - stable approach behavior
  - range-based control

- **TERMINAL phase**
  - velocity limiting (mm/s regime)
  - automatic phase transition
  - docking detection

- **Lost target handling**
  - fallback logic integrated into guidance

---

### Flight Software (FSW)

- Mode manager controlling transitions:
DETUMBLE → SUN_ACQ → FINE_POINTING → MOMENTUM_DUMP

- Integrated **real-time flight loop**
- Fixed scheduling (100 Hz / 10 Hz split)
- No dynamic allocation (embedded-safe)

---

## Numerical Validation (SIL)

The C implementation is validated against a Python reference model.

### Accuracy

- Position divergence: ~1e-10 m
- Velocity divergence: ~1e-14 m/s
- Covariance consistency maintained

→ **Sub-mm numerical parity over full GEO orbit**

---

### Real-Time Behavior

- Loop frequency: 100 Hz
- Max execution time: ~1.6 ms
- Budget: 10 ms
- Missed deadlines: 0

→ ~6× real-time margin on PC

---

### Robustness

Tested scenarios include:

- Sensor dropout (range loss)
- Measurement outliers (camera spikes)
- Bias injection (gyro disturbances)

All cases:
- remain stable
- recover without divergence

---

## Guidance Characteristics

### Closing Law
v = K * sqrt(range)

This produces:
- smooth deceleration
- physically consistent approach
- stable terminal convergence

---

### Docking Behavior

- Terminal velocity capped (~mm/s)
- Dock detection below threshold (~cm scale)
- No oscillatory behavior near contact

---

## Architecture
Python Simulation (truth model)
↓
C Flight Code (GNC stack)
↓
Real-Time Loop (SIL)
↓
Command Outputs (actuation)

The system is designed to transition directly into:
- hardware-in-the-loop (HIL)
- embedded deployment

---

## Limitations

- RPOD currently modeled in **3-DOF (translation only)**
- No **6-DOF docking alignment**
- Target assumed attitude-stable
- No onboard orbit determination of the chief
- No vision-based pose estimation in C yet

---

## Work Remaining

- Lambert solver (far-field transfer)
- Vision-based pose estimation (terminal phase)
- Full 6-DOF docking control
- Embedded deployment (STM32 class hardware)

---

## Design Philosophy

- Software-first autonomy layer
- Hardware-agnostic integration
- Deterministic execution
- Simulation-backed validation before hardware

---

## Objective

To develop a **deployable autonomy stack** for:

- Non-cooperative satellite docking
- In-orbit servicing
- Autonomous proximity operations

---

## Author

Venkat Sainath  
MSc Space Engineering  

Building autonomous GNC systems for space applications