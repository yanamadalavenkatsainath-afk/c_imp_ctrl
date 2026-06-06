#include <math.h>
#include <stdio.h>
#include "keepout_planner.h"

#define CHECK(cond, msg) do { \
    printf("  %s - %s\n", (cond) ? "PASS" : "FAIL", msg); \
    if (!(cond)) failures++; \
} while (0)

static int failures = 0;

static double norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int main(void) {
    printf("=== Keepout Planner C Verification ===\n");
    KeepoutPlanner planner;
    Keepout_default_appendage_zones(&planner);
    CHECK(planner.zone_count == 7,
          "default IS-1002 appendage model has 7 keepout spheres");

    double I[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    KeepoutResult out;

    double far_dep[3] = {20.0, 20.0, 20.0};
    Keepout_compute(&planner, far_dep, I, &out);
    CHECK(out.active_count == 0 && norm3(out.accel) < 1e-15,
          "far deputy gets no repulsive command");

    double near_root[3] = {0.0, 2.2, 0.0};
    Keepout_compute(&planner, near_root, I, &out);
    CHECK(out.active_count > 0,
          "deputy inside warning band activates keepout");
    CHECK(out.accel[1] > 0.0,
          "repulsive command points away from +Y solar array root");
    CHECK(norm3(out.accel) <= planner.accel_max + 1e-12,
          "repulsive command is clamped");

    printf("=== %s (%d failures) ===\n", failures ? "FAIL" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
