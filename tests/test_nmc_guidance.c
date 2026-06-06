#include <math.h>
#include <stdio.h>
#include "nmc_guidance.h"

#define CHECK(cond, msg) do { \
    printf("  %s - %s\n", (cond) ? "PASS" : "FAIL", msg); \
    if (!(cond)) failures++; \
} while (0)

static int failures = 0;

static double norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int main(void) {
    printf("=== NMC Guidance C Verification ===\n");
    NMC_Config cfg;
    NMC_default_config(&cfg);

    double rp[3], rv[3];
    NMC_reference(&cfg, 0.0, rp, rv);
    CHECK(fabs(rp[0] - 75.0) < 1e-9 && fabs(rp[1]) < 1e-9 && fabs(rp[2]) < 1e-9,
          "reference starts at +X radius");
    CHECK(rv[1] > 0.0 && rv[2] > 0.0,
          "reference velocity has along-track and vertical components at start");

    NMC_State s = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double a[3];
    NMC_compute(&cfg, &s, 0.0, a, rp, rv);
    CHECK(norm3(a) <= cfg.accel_max + 1e-12,
          "acceleration is clamped to configured max");
    CHECK(a[0] > 0.0,
          "acceleration points toward the reference orbit");

    NMC_State on_ref = {{75.0, 0.0, 0.0}, {0.0, 75.0 * 2.0 * 3.141592653589793 / 1800.0,
                                           0.5 * 15.0 * 2.0 * 3.141592653589793 / 1800.0}};
    NMC_compute(&cfg, &on_ref, 0.0, a, rp, rv);
    CHECK(norm3(a) < 1e-12,
          "zero command when state equals reference");

    printf("=== %s (%d failures) ===\n", failures ? "FAIL" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
