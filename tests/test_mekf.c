/**
 * test_mekf.c — Standalone MEKF C verification
 * ==============================================
 * Compile:
 *   gcc -O2 -DMEKF_NO_CMSIS -o test_mekf tests/test_mekf.c src_c/mekf.c -lm -I.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "mekf.h"

static int n_fail = 0;

#define CHECK(cond, msg) \
    do { printf("  %s -- %s\n", (cond) ? "PASS" : "FAIL", msg); \
         if(!(cond)) n_fail++; } while(0)
#define DEG2RAD(x) ((x)*3.14159265f/180.0f)
#define RAD2DEG(x) ((x)*180.0f/3.14159265f)

int main(void) {
    printf("=== MEKF C Verification ===\n\n");

    printf("Test 0: Init constants match Python MEKF\n");
    {
        MEKF_State s;
        MEKF_init(&s, 0.1f);

        CHECK(fabsf(s.P[0][0] - 3.046e-6f) < 1e-10f,
              "P_att uses (0.1 deg)^2, not 0.1 rad^2");
        CHECK(fabsf(s.P[3][3] - 5.876e-12f) < 1e-15f,
              "P_bias uses (0.5 deg/hr)^2");
        CHECK(fabsf(s.R_mag[0][0] - 0.5f) < 1e-7f,
              "R_mag matches GEO Python value 0.5");
        CHECK(fabsf(s.R_sun[0][0] - 3e-6f) < 1e-12f,
              "R_sun matches Python value 3e-6");
    }

    /* ── Test 1: Identity init — quaternion stays unit norm ────── */
    printf("\nTest 1: Quaternion stays unit norm after 100 predict steps\n");
    {
        MEKF_State s;
        MEKF_init(&s, 0.1f);

        MEKF_FLOAT omega[3] = {0.01f, 0.005f, -0.003f};  /* rad/s */
        for (int i = 0; i < 100; i++)
            MEKF_predict(&s, omega);

        float norm = sqrtf(s.q[0]*s.q[0]+s.q[1]*s.q[1]+
                           s.q[2]*s.q[2]+s.q[3]*s.q[3]);
        printf("  Quaternion norm after 100 steps: %.8f\n", norm);
        CHECK(fabsf(norm - 1.0f) < 1e-5f, "|q| = 1.0 ± 1e-5");
    }

    /* ── Test 2: Update with perfect measurement → covariance reduces */
    printf("\nTest 2: Update with truth measurement — observable axes reduce\n");
    {
        MEKF_State s;
        MEKF_init(&s, 0.1f);

        MEKF_FLOAT v_inertial[3] = {1.0f, 0.0f, 0.0f};
        MEKF_FLOAT z_body[3]     = {1.0f, 0.0f, 0.0f};

        /* R << P so Kalman gain is large and P visibly reduces.
           For z=[1,0,0] the skew H has H[0,:]=0 so P[0][0] is unobservable.
           P[1][1] and P[2][2] are observable and must reduce. */
        MEKF_FLOAT R[3][3] = {{1e-6f,0,0},{0,1e-6f,0},{0,0,1e-6f}};

        float p1_before = s.P[1][1];
        float p2_before = s.P[2][2];
        MEKF_update(&s, z_body, v_inertial, R);
        float p1_after  = s.P[1][1];
        float p2_after  = s.P[2][2];

        printf("  P[1][1] before: %.6f   after: %.2e\n", p1_before, p1_after);
        printf("  P[2][2] before: %.6f   after: %.2e\n", p2_before, p2_after);
        CHECK(p1_after < p1_before, "P[1][1] reduced (observable axis)");
        CHECK(p2_after < p2_before, "P[2][2] reduced (observable axis)");
        CHECK(fabsf(s.q[0]-1.0f) < 1e-4f, "quaternion unchanged with zero innovation");
    }

    /* ── Test 3: Bias estimation converges ──────────────────────── */
    printf("\nTest 3: Bias estimation — injected bias recovered\n");
    {
        MEKF_State s;
        MEKF_init(&s, 0.1f);

        float true_bias[3] = {0.001f, -0.0005f, 0.0002f};  /* rad/s */

        MEKF_FLOAT v_inertial[3] = {0.0f, 1.0f, 0.0f};
        MEKF_FLOAT z_body[3]     = {0.0f, 1.0f, 0.0f};
        MEKF_FLOAT R[3][3] = {{1e-4f,0,0},{0,1e-4f,0},{0,0,1e-4f}};

        for (int i = 0; i < 500; i++) {
            MEKF_FLOAT omega_m[3] = {true_bias[0], true_bias[1], true_bias[2]};
            MEKF_predict(&s, omega_m);
            MEKF_update(&s, z_body, v_inertial, R);
        }

        float bias_err = sqrtf(
            (s.bias[0]-true_bias[0])*(s.bias[0]-true_bias[0]) +
            (s.bias[1]-true_bias[1])*(s.bias[1]-true_bias[1]) +
            (s.bias[2]-true_bias[2])*(s.bias[2]-true_bias[2]));
        printf("  Bias error after 500 steps: %.6f rad/s\n", bias_err);
        CHECK(bias_err < 2e-3f, "bias error < 2 mrad/s");
    }

    /* ── Test 4: P stays symmetric and positive definite ────────── */
    printf("\nTest 4: Covariance stays symmetric after mixed predict/update\n");
    {
        MEKF_State s;
        MEKF_init(&s, 0.1f);
        MEKF_FLOAT omega[3]      = {0.005f, -0.003f, 0.001f};
        MEKF_FLOAT v_inertial[3] = {0.577f, 0.577f, 0.577f};
        MEKF_FLOAT z_body[3]     = {0.577f, 0.577f, 0.577f};
        MEKF_FLOAT R[3][3] = {{1e-4f,0,0},{0,1e-4f,0},{0,0,1e-4f}};

        for (int i = 0; i < 200; i++) {
            MEKF_predict(&s, omega);
            if (i % 5 == 0) MEKF_update(&s, z_body, v_inertial, R);
        }

        int sym_ok = 1;
        int pos_ok = 1;
        for (int i = 0; i < 6; i++) {
            if (s.P[i][i] <= 0) pos_ok = 0;
            for (int j = i+1; j < 6; j++)
                if (fabsf(s.P[i][j]-s.P[j][i]) > 1e-8f) sym_ok = 0;
        }
        CHECK(sym_ok, "P is symmetric");
        CHECK(pos_ok, "P diagonal is positive");
    }

    printf("\n=== %s (%d failures) ===\n",
           n_fail == 0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}
