/**
 * test_chief_pose.c — Chief Pose Estimator C verification
 * =========================================================
 * Compile:
 *   gcc -O2 -Isrc_c -o test_chief_pose.exe \
 *       tests/test_chief_pose.c src_c/chief_pose_estimator.c -lm
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "chief_pose_estimator.h"

#define CHECK(cond, msg) \
    do { printf("  %s — %s\n",(cond)?"✓ PASS":"✗ FAIL",msg); \
         if(!(cond)) n_fail++; } while(0)

static int n_fail = 0;

/* ── Test 1: Init — quaternion is unit norm ─────────────────── */
static void test_init_norm(void) {
    printf("Test 1: Init — quaternion is unit norm\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CHECK(cam.n_model_pts == CPE_MAX_MODEL_PTS, "default chief pose model uses 11 feature points");
    CHECK(fabs(cam.model_pts[0][0] - CFG_CHIEF_BODY_HALF_X_M) < 1e-12,
          "default chief pose model uses IS-1002 scale");
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    double n=sqrt(s.q[0]*s.q[0]+s.q[1]*s.q[1]+s.q[2]*s.q[2]+s.q[3]*s.q[3]);
    printf("  |q| = %.8f\n", n);
    CHECK(fabs(n-1.0)<1e-8, "|q| = 1.0 after init");
}

/* ── Test 2: P init — diagonal positive ─────────────────────── */
static void test_init_P(void) {
    printf("\nTest 2: Init — P diagonal is positive\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);
    int ok=1;
    for(int i=0;i<6;i++) if(s.P[i][i]<=0.) ok=0;
    CHECK(ok, "all diagonal P elements > 0");
    printf("  P[0][0]=%.4f  P[3][3]=%.6f\n", s.P[0][0], s.P[3][3]);
}

/* ── Test 3: Predict — quaternion stays unit norm ─────────────*/
static void test_predict_norm(void) {
    printf("\nTest 3: Predict — quaternion stays unit norm after 200 steps\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    /* Set a nonzero omega */
    s.omega[0]=0.002; s.omega[1]=-0.001; s.omega[2]=0.0005;

    /* Run predict-only for 200 steps */
    double q_id[4]={1.,0.,0.,0.};
    for(int i=0;i<200;i++){
        CPE_update(&s, (double[]){0.,100.,0.}, q_id);
    }
    double n=sqrt(s.q[0]*s.q[0]+s.q[1]*s.q[1]+s.q[2]*s.q[2]+s.q[3]*s.q[3]);
    printf("  |q| after 200 steps = %.8f\n", n);
    CHECK(fabs(n-1.0)<1e-5, "|q| = 1.0 ± 1e-5 after 200 predict steps");
}

/* ── Test 4: P stays symmetric ───────────────────────────────── */
static void test_P_symmetric(void) {
    printf("\nTest 4: Covariance stays symmetric after predict/update\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    srand(42);
    double q_id[4]={1.,0.,0.,0.};
    /* Run at close range (2m) to get PnP successes */
    double dr[3]={0.,-5.,0.};
    for(int i=0;i<100;i++) CPE_update(&s, dr, q_id);

    int sym_ok=1;
    for(int i=0;i<6;i++) for(int j=i+1;j<6;j++)
        if(fabs(s.P[i][j]-s.P[j][i])>1e-8) sym_ok=0;
    CHECK(sym_ok, "P is symmetric after 100 steps");

    int pos_ok=1;
    for(int i=0;i<6;i++) if(s.P[i][i]<=0.) pos_ok=0;
    CHECK(pos_ok, "P diagonal is positive after 100 steps");
}

/* ── Test 5: Valid flag set after 10 updates ─────────────────── */
static void test_valid_flag(void) {
    printf("\nTest 5: valid flag set after >= 10 successful PnP updates\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    srand(42);
    double q_id[4]={1.,0.,0.,0.};
    /* 2m range — close enough for multiple PnP successes */
    double dr[3]={0.,-5.,0.};

    int became_valid=0;
    for(int i=0;i<200;i++){
        CPE_Result r=CPE_update(&s, dr, q_id);
        if(r.valid) { became_valid=1; break; }
    }
    printf("  valid after %d updates=%d\n", s.update_count, s.valid);
    CHECK(became_valid, "valid=1 reached within 200 steps at 2m range");
}

/* ── Test 6: omega_uncertainty decreases with updates ───────── */
static void test_uncertainty_decreases(void) {
    printf("\nTest 6: Omega uncertainty decreases with successful updates\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    double unc_init = CPE_omega_uncertainty(&s);

    srand(42);
    double q_id[4]={1.,0.,0.,0.};
    double dr[3]={0.,-5.,0.};
    for(int i=0;i<100;i++) CPE_update(&s, dr, q_id);

    double unc_after = CPE_omega_uncertainty(&s);
    printf("  omega_uncertainty: init=%.4f  after=%.4f rad/s\n",
           unc_init, unc_after);
    CHECK(unc_after < unc_init, "omega uncertainty decreases after updates");
}

/* ── Test 7: Far-range — no updates (camera out of range) ────── */
static void test_far_range_no_update(void) {
    printf("\nTest 7: Far range (10km) — update_count stays 0\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    srand(0);
    double q_id[4]={1.,0.,0.,0.};
    double dr_far[3]={0.,-10000.,0.};   /* 10 km — beyond max_range=5000m */
    for(int i=0;i<20;i++) CPE_update(&s, dr_far, q_id);

    printf("  update_count = %d  (expect 0)\n", s.update_count);
    CHECK(s.update_count == 0, "no PnP updates beyond max_range");
    CHECK(s.valid == 0,        "valid=0 beyond max_range");
}

/* ── Test 8: R_body2lvlh accessor ────────────────────────────── */
static void test_R_accessor(void) {
    printf("\nTest 8: CPE_get_R_body2lvlh returns 0 before first PnP\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    double R[3][3];
    int got = CPE_get_R_body2lvlh(&s, R);
    printf("  has_R_b2l = %d  (expect 0)\n", got);
    CHECK(got == 0, "CPE_get_R_body2lvlh returns 0 before any PnP");

    /* Run at close range to get a PnP */
    srand(42);
    double q_id[4]={1.,0.,0.,0.};
    double dr[3]={0.,-5.,0.};
    for(int i=0;i<50;i++) CPE_update(&s, dr, q_id);
    got = CPE_get_R_body2lvlh(&s, R);
    printf("  has_R_b2l after 50 steps = %d\n", got);
    /* Not guaranteed if PnP always fails, but usually succeeds at 2m */
    printf("  (1 = PnP succeeded at least once; 0 = all attempts failed)\n");
}

/* ── main ─────────────────────────────────────────────────────── */
static void test_update_rotation_hardware_interface(void) {
    printf("\nTest 9: CPE_update_rotation fuses external pose measurement\n");
    CPE_CamParams cam; CPE_default_cam_params(&cam);
    CPE_State s;
    CPE_init(&s, &cam, 0.1, 0.001, 3.0, 10.0);

    double c = cos(20.0 * M_PI / 180.0);
    double sn = sin(20.0 * M_PI / 180.0);
    double R_meas[3][3] = {
        { c, -sn, 0.0 },
        { sn,  c, 0.0 },
        {0.0, 0.0, 1.0 }
    };

    CPE_Result r = {0};
    for(int i=0;i<12;i++) {
        r = CPE_update_rotation(&s, R_meas, 5.0);
    }

    double R_out[3][3];
    int got = CPE_get_R_body2lvlh(&s, R_out);
    CHECK(got == 1, "hardware rotation update stores R_body2lvlh");
    CHECK(r.valid == 1, "hardware rotation update reaches valid after repeated accepts");
    CHECK(fabs(R_out[0][0] - c) < 1e-12 &&
          fabs(R_out[1][0] - sn) < 1e-12,
          "stored rotation matches external measurement");
}

int main(void) {
    printf("=== Chief Pose Estimator C Verification ===\n\n");
    test_init_norm();
    test_init_P();
    test_predict_norm();
    test_P_symmetric();
    test_valid_flag();
    test_uncertainty_decreases();
    test_far_range_no_update();
    test_R_accessor();
    test_update_rotation_hardware_interface();
    printf("\n=== %s (%d failures) ===\n",
           n_fail==0?"ALL PASS":"FAILURES DETECTED", n_fail);
    return n_fail==0?0:1;
}
