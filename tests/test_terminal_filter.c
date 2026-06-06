#include <math.h>
#include <stdio.h>

#include "terminal_filter.h"
#include "sim_config.h"

#define CHECK(cond, msg) \
    do { printf("  %s -- %s\n", (cond) ? "PASS" : "FAIL", msg); \
         if(!(cond)) n_fail++; } while(0)

static int n_fail = 0;

static void test_soft_gate_clips_and_updates(void) {
    printf("\nTest 1: terminal filter clips large camera innovations\n");
    TermNavFilter f;
    double pos[3], vel[3];
    double z0[3] = {0.0, 0.0, 0.0};
    double v0[3] = {0.0, 0.0, 0.0};
    TNF_reset(&f);
    TNF_update(&f, z0, v0, 1, 0.1, pos, vel);

    double z_jump[3] = {1.0, 0.0, 0.0};
    TNF_update(&f, z_jump, v0, 1, 0.1, pos, vel);

    CHECK(fabs(pos[0] - CFG_TERM_NAV_ALPHA * CFG_TERM_NAV_GATE_M) < 1e-12,
          "position receives configured alpha times clipped residual");
    CHECK(fabs(vel[0] - CFG_TERM_NAV_VMAX_MS) < 1e-12,
          "velocity receives beta update then clips to configured max speed");
}

static void test_velocity_limit(void) {
    printf("\nTest 2: terminal filter velocity is limited\n");
    TermNavFilter f;
    double pos[3], vel[3];
    double z0[3] = {0.0, 0.0, 0.0};
    double v0[3] = {1.0, 0.0, 0.0};
    TNF_reset(&f);
    TNF_update(&f, z0, v0, 1, 0.1, pos, vel);

    double speed = sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
    CHECK(speed <= CFG_TERM_NAV_VMAX_MS + 1e-12,
          "velocity limit uses CFG_TERM_NAV_VMAX_MS");
}

int main(void) {
    printf("=== Terminal Filter C Verification ===\n");
    test_soft_gate_clips_and_updates();
    test_velocity_limit();
    printf("\n=== %s (%d failures) ===\n",
           n_fail == 0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}
