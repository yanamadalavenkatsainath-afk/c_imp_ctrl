#include <stdio.h>
#include "capture_gate.h"

#define CHECK(cond, msg) \
    do { printf("  %s -- %s\n", (cond) ? "PASS" : "FAIL", msg); \
         if (!(cond)) n_fail++; } while(0)

static int n_fail = 0;

static void test_soft_entry_uses_coarse_alignment(void) {
    printf("\nTest 1: soft capture entry uses 30 deg coarse gate\n");
    CaptureGateIn in;
    CaptureGateOut out;
    CaptureGate_default(&in);
    in.port_range_m = 0.10;
    in.port_vrel_ms = 0.020;
    in.align_deg = 29.9;
    in.body_clear = 1;
    CaptureGate_eval(&in, &out);
    CHECK(out.soft_capture_ready == 1, "29.9 deg enters soft capture");

    in.align_deg = 30.0;
    CaptureGate_eval(&in, &out);
    CHECK(out.soft_capture_ready == 0, "30.0 deg does not enter soft capture");
}

static void test_hard_capture_requires_strict_geometry(void) {
    printf("\nTest 2: hard capture requires strict geometry and align\n");
    CaptureGateIn in;
    CaptureGateOut out;
    CaptureGate_default(&in);
    in.port_range_m = 0.02;
    in.port_vrel_ms = 0.002;
    in.align_deg = 5.0;
    in.align_ok = 1;
    in.body_clear = 1;
    in.geometry_ok = 1;
    CaptureGate_eval(&in, &out);
    CHECK(out.hard_capture_ready == 1, "strict hard-capture envelope passes");

    in.geometry_ok = 0;
    CaptureGate_eval(&in, &out);
    CHECK(out.hard_capture_ready == 0, "bad geometry blocks hard capture");
}

static void test_soft_certified_uses_core_and_stability(void) {
    printf("\nTest 3: soft certified uses stable + core + align\n");
    CaptureGateIn in;
    CaptureGateOut out;
    CaptureGate_default(&in);
    in.port_range_m = 0.04;
    in.port_vrel_ms = 0.020;
    in.align_deg = 18.0;
    in.align_ok = 1;
    in.body_clear = 1;
    in.capture_core = 1;
    in.geometry_ok = 0;
    CaptureGate_eval(&in, &out);
    CHECK(out.soft_capture_stable == 1, "below latch velocity is stable");
    CHECK(out.soft_core_ready == 1, "capture core plus align is core-ready");
    CHECK(out.soft_capture_certified == 1,
          "soft certified can pass even when full hard geometry is not strict");

    in.port_vrel_ms = 0.031;
    CaptureGate_eval(&in, &out);
    CHECK(out.soft_capture_certified == 0, "above latch velocity is not certified");
}

int main(void) {
    printf("=== Capture Gate C Verification ===\n");
    test_soft_entry_uses_coarse_alignment();
    test_hard_capture_requires_strict_geometry();
    test_soft_certified_uses_core_and_stability();
    printf("\n=== %s (%d failures) ===\n",
           n_fail == 0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}
