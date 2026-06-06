/*
 * capture_gate.h - Pure capture classification logic.
 *
 * C port of:
 *   C:/Users/Venkat/OneDrive/Desktop/appex/flight sim/fsw/capture_gate.py
 *
 * This module has no state and no side effects.  Callers compute geometry,
 * body clearance, and alignment, then this function classifies the current
 * capture envelope.
 */

#ifndef CAPTURE_GATE_H
#define CAPTURE_GATE_H

#include "sim_config.h"

typedef struct {
    double port_range_m;
    double port_vrel_ms;
    double align_deg;       /* negative value means unknown / no align gate */
    int body_clear;
    int capture_core;
    int geometry_ok;
    int align_ok;

    double soft_capture_range_m;
    double soft_capture_vrel_ms;
    double soft_capture_entry_align_max_deg;
    double soft_capture_latch_vrel_ms;
    double soft_capture_core_align_max_deg;
    double hard_capture_range_m;
    double hard_capture_vrel_ms;
    double dock_align_max_deg;
} CaptureGateIn;

typedef struct {
    int soft_capture_ready;
    int hard_capture_ready;
    int soft_core_ready;
    int soft_capture_stable;
    int soft_capture_certified;
} CaptureGateOut;

void CaptureGate_default(CaptureGateIn *in);
void CaptureGate_eval(const CaptureGateIn *in, CaptureGateOut *out);

#endif /* CAPTURE_GATE_H */
