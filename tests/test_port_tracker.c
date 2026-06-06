#include <math.h>
#include <stdio.h>

#include "port_tracker.h"
#include "sim_config.h"

#define CHECK(cond, msg) \
    do { printf("  %s -- %s\n", (cond) ? "PASS" : "FAIL", msg); \
         if(!(cond)) n_fail++; } while(0)

static int n_fail = 0;

static void test_init_and_alpha(void) {
    printf("\nTest 1: tracker initializes and applies configured alpha\n");
    PortTracker pt;
    double out[3];
    PT_reset(&pt);

    double z0[3] = {0.0, 0.0, 0.5};
    int ok = PT_update(&pt, z0, 1, 0.1, out);
    CHECK(ok == 1, "first valid port packet initializes tracker");
    CHECK(fabs(out[2] - 0.5) < 1e-12, "initial output equals first measurement");

    double z1[3] = {0.0, 0.0, 0.55};
    ok = PT_update(&pt, z1, 1, 0.1, out);
    double expected_z = 0.5 + CFG_PORT_TRACK_ALPHA * 0.05;
    CHECK(ok == 1, "second valid port packet remains valid");
    CHECK(fabs(out[2] - expected_z) < 1e-12, "tracker uses CFG_PORT_TRACK_ALPHA");
}

static void test_gate_and_coast(void) {
    printf("\nTest 2: innovation gate clips jumps and coast expires\n");
    PortTracker pt;
    double out[3];
    PT_reset(&pt);

    double z0[3] = {0.0, 0.0, 0.5};
    PT_update(&pt, z0, 1, 0.1, out);

    double z_bad[3] = {0.0, 0.0, 0.5 + CFG_PORT_TRACK_GATE_M + 0.10};
    PT_update(&pt, z_bad, 1, 0.1, out);
    double expected_z = z0[2] + CFG_PORT_TRACK_ALPHA * CFG_PORT_TRACK_GATE_M;
    CHECK(fabs(out[2] - expected_z) < 1e-12,
          "jump larger than CFG_PORT_TRACK_GATE_M is clipped and still applied");

    int ok = 1;
    int n_steps = (int)(CFG_PORT_TRACK_MAX_COAST_S / 0.1) + 2;
    for(int i=0; i<n_steps; i++) {
        ok = PT_update(&pt, z0, 0, 0.1, out);
    }
    CHECK(ok == 0, "tracker invalidates after CFG_PORT_TRACK_MAX_COAST_S without packets");
}

int main(void) {
    printf("=== Port Tracker C Verification ===\n");
    test_init_and_alpha();
    test_gate_and_coast();
    printf("\n=== %s (%d failures) ===\n",
           n_fail == 0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}
