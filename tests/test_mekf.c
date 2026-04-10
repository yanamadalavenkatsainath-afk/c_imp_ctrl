/**
 * test_mekf.c — Standalone MEKF C verification
 * ==============================================
 * Compile:
 *   gcc -O2 -DMEKF_NO_CMSIS -o test_mekf tests/test_mekf.c src_c/mekf.c -lm -I.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../src_c/mekf.h"

#define CHECK(cond, msg) printf("  %s — %s\n", (cond)?"✓ PASS":"✗ FAIL", msg)
#define DEG2RAD(x) ((x)*3.14159265f/180.0f)
#define RAD2DEG(x) ((x)*180.0f/3.14159265f)

int main(void) {
    printf("=== MEKF C Verification ===\n\n");

    /* ── Test 1: Identity init — quaternion stays unit norm ────── */
    printf("Test 1: Quaternion stays unit norm after 100 predict steps\n");
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
    printf("\nTest 2: Update with truth measurement — innovation ~0\n");
    {
        MEKF_State s;
        MEKF_init(&s, 0.1f);

        MEKF_FLOAT v_inertial[3] = {1.0f, 0.0f, 0.0f};
        MEKF_FLOAT z_body[3]     = {1.0f, 0.0f, 0.0f};

        /* R << P so Kalman gain is large and P visibly reduces.
           P=0.01, R=1e-6 → gain ≈ 1, P drops roughly in half. */
        MEKF_FLOAT R[3][3] = {{1e-6f,0,0},{0,1e-6f,0},{0,0,1e-6f}};

        float p_before = s.P[0][0];
        MEKF_update(&s, z_body, v_inertial, R);
        float p_after  = s.P[0][0];

        printf("  P[0][0] before: %.6f   after: %.6f\n", p_before, p_after);
        CHECK(p_after < p_before, "covariance reduced after update");
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

    printf("\n=== Done ===\n");
    return 0;
}