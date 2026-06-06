#include <stdio.h>
#include <math.h>
#include "spin_sync_controller.h"

static int n_fail = 0;
#define CHECK(cond, msg) do { \
    printf("  %s - %s\n", (cond) ? "PASS" : "FAIL", msg); \
    if(!(cond)) n_fail++; \
} while(0)

static double norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int main(void) {
    printf("=== Spin Sync Controller C Verification ===\n");

    SpinSyncController s;
    SSC_init(&s, 0.35, 0.5);

    double target_body[3] = {0.2, 0.0, 0.0};
    double cmd[3];
    SSC_compute_rate_body(&s, target_body, cmd);
    CHECK(fabs(cmd[0] - 0.07) < 1e-12, "first command applies rate blend");
    CHECK(fabs(cmd[1]) < 1e-12 && fabs(cmd[2]) < 1e-12,
          "uncommanded axes stay zero");

    double big_body[3] = {2.0, 0.0, 0.0};
    SSC_reset(&s);
    SSC_compute_rate_body(&s, big_body, cmd);
    CHECK(fabs(cmd[0] - 0.35 * 0.5) < 1e-12,
          "target rate is clamped before blending");

    double R_l2b[3][3] = {
        {0.0, 1.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double target_lvlh[3] = {0.0, 0.1, 0.0};
    SSC_reset(&s);
    SSC_compute_rate_command(&s, target_lvlh, R_l2b, cmd);
    CHECK(fabs(cmd[0] - 0.035) < 1e-12 && fabs(cmd[1]) < 1e-12,
          "LVLH target rate is transformed into body frame");

    double quality = SSC_sync_quality(cmd, cmd);
    CHECK(fabs(quality - 1.0) < 1e-12, "identical rate and command gives quality 1");
    double far_rate[3] = {10.0, 0.0, 0.0};
    quality = SSC_sync_quality(far_rate, cmd);
    CHECK(quality == 0.0, "large rate error clips quality to zero");

    SSC_reset(&s);
    CHECK(norm3(s.omega_cmd) < 1e-15, "reset clears blended command");

    printf("=== %s (%d failures) ===\n",
           n_fail == 0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}
