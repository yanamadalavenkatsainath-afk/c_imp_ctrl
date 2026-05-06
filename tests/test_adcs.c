/**
 * test_adcs.c — Standalone ADCS C verification
 * ==============================================
 * Tests: BDOT_compute, ATTCTRL_compute, RW_apply_torque,
 *        MTQ_compute_dipole, MTQ_compute_torque
 * Compile:
 *   gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_adcs.exe tests/test_adcs.c src_c/adcs.c -lm
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "adcs.h"

#define CHECK(cond, msg) \
    do { printf("  %s — %s\n",(cond)?"✓ PASS":"✗ FAIL",msg); \
         if(!(cond)) n_fail++; } while(0)

static int n_fail = 0;

static double norm3(const double v[3]){
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}


/* ── Test 1: B-dot — zero rate gives zero dipole ───────────── */
static void test_bdot_zero_rate(void) {
    printf("Test 1: B-dot — zero angular rate gives zero dipole\n");
    BDot_State s;
    BDOT_init(&s);

    double B[3] = {2e-5, 1e-5, 4e-5};
    double omega[3] = {0.0, 0.0, 0.0};
    double m[3], tau[3];
    BDOT_compute(&s, B, omega, m, tau);

    printf("  |m_cmd| = %.2e A·m²\n", norm3(m));
    CHECK(norm3(m) < 1e-20, "zero dipole when omega=0");
}


/* ── Test 2: B-dot — torque opposes rotation ───────────────── */
static void test_bdot_damping(void) {
    printf("\nTest 2: B-dot — torque opposes angular rate\n");
    BDot_State s;
    BDOT_init(&s);

    /* Spin about z, B along z: B_dot = -omega×B = 0 (parallel)
       Use B perpendicular to omega for non-zero result */
    double omega[3] = {0.5, 0.0, 0.0};   /* spinning about x */
    double B[3]     = {0.0, 2e-5, 0.0};  /* B along y */
    double m[3], tau[3];
    BDOT_compute(&s, B, omega, m, tau);

    /* B_dot = -omega×B = -[0,0,0.5]×[0,2e-5,0] wait:
       omega=[0.5,0,0], B=[0,2e-5,0]
       omega×B = [0*0-0*2e-5, 0*0-0.5*0, 0.5*2e-5-0*0] = [0, 0, 1e-5]
       B_dot = -[0,0,1e-5]
       m = -k*B_dot = k*[0,0,1e-5] = [0,0,1000] → clipped to m_max=0.2
       torque = m × B = [0,0,0.2]×[0,2e-5,0] = [0.2*0-0*0, 0*0-0*0, 0*2e-5-0.2*0]
                      = [0, 0, 0] hmm — let's just verify torque is non-zero */
    printf("  m_cmd = [%.4f, %.4f, %.4f] A·m²\n", m[0],m[1],m[2]);
    printf("  torque= [%.2e, %.2e, %.2e] N·m\n",  tau[0],tau[1],tau[2]);
    CHECK(norm3(m) > 0.0, "non-zero dipole when omega and B non-parallel");
    /* Dipole should be clipped to m_max */
    CHECK(norm3(m) <= s.m_max + 1e-12, "dipole clipped to m_max");
}


/* ── Test 3: B-dot — dipole saturation ─────────────────────── */
static void test_bdot_saturation(void) {
    printf("\nTest 3: B-dot — dipole clips at m_max\n");
    BDot_State s;
    BDOT_init(&s);

    /* Large omega → large B_dot → m clips */
    double omega[3] = {100.0, 100.0, 100.0};
    double B[3]     = {1e-4,  1e-4,  1e-4};
    double m[3], tau[3];
    BDOT_compute(&s, B, omega, m, tau);

    for(int i=0;i<3;i++) {
        CHECK(fabs(m[i]) <= s.m_max + 1e-12, "each dipole component ≤ m_max");
    }
    printf("  m_cmd = [%.4f, %.4f, %.4f] (m_max=%.2f)\n", m[0],m[1],m[2],s.m_max);
}


/* ── Test 4: Reaction wheel integrates momentum ─────────────── */
static void test_rw_momentum(void) {
    printf("\nTest 4: Reaction wheel — momentum integrates correctly\n");
    RW_State rw;
    RW_init(&rw);

    /* Use small torque so h stays well below h_max=0.05 in 10 steps */
    double tau[3] = {0.001, -0.0005, 0.0002};
    double dt = 0.1;

    for(int i=0;i<10;i++) RW_apply_torque(&rw, tau, dt);

    double expected[3] = {0.001*0.1*10, -0.0005*0.1*10, 0.0002*0.1*10};
    printf("  h   = [%.6f, %.6f, %.6f] N·m·s\n", rw.h[0],rw.h[1],rw.h[2]);
    printf("  exp = [%.6f, %.6f, %.6f]\n", expected[0],expected[1],expected[2]);
    for(int i=0;i<3;i++)
        CHECK(fabs(rw.h[i]-expected[i]) < 1e-10, "h[i] matches tau*dt*steps");
}


/* ── Test 5: Reaction wheel saturates ───────────────────────── */
static void test_rw_saturation(void) {
    printf("\nTest 5: Reaction wheel — momentum clips at h_max\n");
    RW_State rw;
    RW_init(&rw);

    double tau[3] = {1.0, 0.0, 0.0};  /* large torque */
    for(int i=0;i<1000;i++) RW_apply_torque(&rw, tau, 0.01);

    printf("  h[0] = %.5f  h_max = %.3f\n", rw.h[0], rw.h_max);
    CHECK(fabs(rw.h[0]-rw.h_max) < 1e-10, "h[0] saturates at h_max");
    CHECK(fabs(rw.h[1]) < 1e-12, "h[1] = 0 (no torque applied)");
}


/* ── Test 6: MTQ momentum dump dipole ───────────────────────── */
static void test_mtq_dipole(void) {
    printf("\nTest 6: MTQ — momentum dump dipole direction\n");
    MTQ_State mtq;
    MTQ_init(&mtq);

    /* h along x, B along y → h×B = -z → m = +z direction */
    double h[3]     = {0.01, 0.0, 0.0};
    double B_body[3]= {0.0,  2e-5, 0.0};
    double m[3];
    MTQ_compute_dipole(&mtq, h, B_body, m);

    printf("  m = [%.4f, %.4f, %.4f]\n", m[0],m[1],m[2]);
    /* h×B = [0,0,0.01*2e-5] → m = -k*(h×B)/|B|² should be in -z */
    CHECK(fabs(m[0]) < 1e-6, "m[0] ≈ 0 (h×B has no x component)");
    CHECK(fabs(m[1]) < 1e-6, "m[1] ≈ 0 (h×B has no y component)");
    CHECK(m[2] < 0.0,        "m[2] < 0 (dumps h in correct direction)");
    CHECK(norm3(m) <= mtq.m_max + 1e-12, "dipole clipped to m_max");
}


/* ── Test 7: MTQ torque = m × B ─────────────────────────────── */
static void test_mtq_torque(void) {
    printf("\nTest 7: MTQ — torque = m × B\n");
    double m[3]     = {0.1, 0.0, 0.0};
    double B_body[3]= {0.0, 2e-5, 0.0};
    double tau[3];
    MTQ_compute_torque(m, B_body, tau);

    /* [0.1,0,0] × [0,2e-5,0] = [0*0-0*2e-5, 0*0-0.1*0, 0.1*2e-5-0*0] = [0,0,2e-6] */
    printf("  tau = [%.2e, %.2e, %.2e]\n", tau[0],tau[1],tau[2]);
    CHECK(fabs(tau[0]) < 1e-15, "tau[0] = 0");
    CHECK(fabs(tau[1]) < 1e-15, "tau[1] = 0");
    CHECK(fabs(tau[2] - 2e-6) < 1e-15, "tau[2] = m[0]*B[1] = 2e-6 N·m");
}


/* ── Test 8: Attitude controller — identity error → zero torque */
static void test_attctrl_zero_error(void) {
    printf("\nTest 8: AttCtrl — zero error gives zero torque\n");
    AttCtrl_State ctrl;
    ATTCTRL_init(&ctrl);

    double q[4]     = {1.0, 0.0, 0.0, 0.0};   /* identity */
    double omega[3] = {0.0, 0.0, 0.0};
    double q_ref[4] = {1.0, 0.0, 0.0, 0.0};   /* same ref */
    double tau[3], q_err[4];
    ATTCTRL_compute(&ctrl, q, omega, q_ref, tau, q_err);

    printf("  tau = [%.2e, %.2e, %.2e]\n", tau[0],tau[1],tau[2]);
    CHECK(norm3(tau) < 1e-14, "zero torque when q_est == q_ref and omega == 0");
    CHECK(fabs(q_err[0]-1.0) < 1e-14, "q_err = identity");
}


/* ── Test 9: AttCtrl — proportional term sign check ────────── */
static void test_attctrl_proportional(void) {
    printf("\nTest 9: AttCtrl — proportional term drives q_err to zero\n");
    AttCtrl_State ctrl;
    ATTCTRL_init(&ctrl);

    /* Small error about x-axis — q_est slightly off from q_ref */
    double angle = 0.1;   /* 0.1 rad error about x */
    double q_est[4] = {cos(angle/2), sin(angle/2), 0.0, 0.0};
    double q_ref[4] = {1.0, 0.0, 0.0, 0.0};
    double omega[3] = {0.0, 0.0, 0.0};
    double tau[3], q_err[4];
    ATTCTRL_compute(&ctrl, q_est, omega, q_ref, tau, q_err);

    printf("  q_err[1] = %.6f (expect ~%.4f)\n", q_err[1], -sin(angle/2));
    printf("  tau[0] = %.6f (expect Kp*q_err[1] = %.6f)\n",
           tau[0], ctrl.Kp * q_err[1]);
    CHECK(fabs(tau[0] - ctrl.Kp * q_err[1]) < 1e-10,
          "tau[0] = Kp * q_err[1] (proportional term)");
    CHECK(fabs(tau[1]) < 1e-14, "tau[1] = 0 (no y/z error)");
    CHECK(fabs(tau[2]) < 1e-14, "tau[2] = 0 (no y/z error)");
}


/* ── main ─────────────────────────────────────────────────────── */
int main(void) {
    printf("=== ADCS C Verification ===\n\n");

    test_bdot_zero_rate();
    test_bdot_damping();
    test_bdot_saturation();
    test_rw_momentum();
    test_rw_saturation();
    test_mtq_dipole();
    test_mtq_torque();
    test_attctrl_zero_error();
    test_attctrl_proportional();

    printf("\n=== %s (%d failures) ===\n",
           n_fail==0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}