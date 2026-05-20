/**
 * test_rpod.c — Standalone RPOD guidance C verification
 * ======================================================
 * Compile:
 *   gcc -O2 -Isrc_c -o test_rpod tests/test_rpod.c src_c/rpod_ctrl.c -lm
 * Run:
 *   test_rpod.exe
 *
 * Tests mirror the Python lambert_controller.py _prox_ops() and _terminal()
 * behaviour. Every pass criterion is derived from constants in rpod_ctrl.h.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "rpod_ctrl.h"

#define CHECK(cond, msg) \
    do { printf("  %s — %s\n", (cond) ? "✓ PASS" : "✗ FAIL", msg); \
         if (!(cond)) n_fail++; } while(0)

static int n_fail = 0;

/* ── helpers ──────────────────────────────────────────────────── */
static double norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void init_term_state(RPOD_TermState *s) {
    memset(s, 0, sizeof(*s));
    s->port_axis_lvlh[2] = 1.0;
    s->geometry_ok = 1;
}


/* ── Test 1: PROX_OPS — correct closing speed selected ─────────
   At range 307m (between 200m and 500m profile entries) the
   desired closing speed should be 100mm/s (the 200m threshold).
   Accel must be bounded by accel_max = 0.020 m/s².              */
static void test_prox_speed_profile(void) {
    printf("Test 1: PROX_OPS velocity profile selection\n");

    double n_chief  = 7.292e-5;   /* rad/s — GEO mean motion */
    double accel_max = 0.020;

    /* Deputy at 307m along the -y LVLH axis (pure along-track) */
    RPOD_State state;
    state.pos[0] = 0.0;
    state.pos[1] = -307.0;   /* behind chief */
    state.pos[2] = 0.0;
    state.vel[0] = 0.0;
    state.vel[1] = 0.0;
    state.vel[2] = 0.0;

    double accel[3];
    RPOD_prox_ops(&state, 307.0, n_chief, accel_max, accel);

    /* At 307m: between 500m and 200m entries.
       Profile walk: 307 <= 500 → v_close = 0.200
                     307 <= 200 → NO → keep 0.200
       So desired closing speed = 200 mm/s toward chief (+y direction).
       vel_des = -pos_hat * v_close = [0, +0.200, 0].
       With vel = 0: vel_err = [0, -0.200, 0].
       accel = -vel_err / TAU = [0, +0.200/5, 0] = [0, +0.04, 0] m/s².
       Clamped to accel_max = 0.020. */
    double mag = norm3(accel);
    CHECK(fabs(mag - accel_max) < 1e-6,
          "accel clamped to accel_max at 307m");
    CHECK(accel[1] > 0.0,
          "accel points toward chief (+y)");
    printf("  accel = [%.4f, %.4f, %.4f] m/s²\n",
           accel[0], accel[1], accel[2]);
}


    /* ── Test 2: PROX_OPS — correct speed at 50m ───────────────────
       sqrt law: v_close = K_SQRT * sqrt(50) ≈ 0.06325 m/s
       accel = -v_close / TAU = -0.06325/5 = -0.01265 m/s²          */
    static void test_prox_speed_50m(void) {
        printf("\nTest 2: PROX_OPS closing speed at 50m (sqrt law)\n");

        double n_chief   = 7.292e-5;
        double accel_max = 0.020;

        RPOD_State state;
        state.pos[0] = 50.0;   /* radial */
        state.pos[1] = 0.0;
        state.pos[2] = 0.0;
        state.vel[0] = 0.0;
        state.vel[1] = 0.0;
        state.vel[2] = 0.0;

        double accel[3];
        RPOD_prox_ops(&state, 50.0, n_chief, accel_max, accel);

        /* sqrt law: K_SQRT*sqrt(50) = 0.06325, vel_des = [-0.06325,0,0]
           vel_err = [0.06325,0,0], accel = -0.06325/TAU = -0.01265 m/s²
           Not saturated (< accel_max=0.020) */
        double expected_ax = -(RPOD_K_SQRT * sqrt(50.0)) / RPOD_PROX_TAU;
        CHECK(fabs(accel[0] - expected_ax) < 1e-4,
              "accel_x = -K_SQRT*sqrt(50)/TAU (sqrt law at 50m)");
        printf("  accel[0] = %.5f  expected %.5f\n", accel[0], expected_ax);
    }


/* ── Test 3: PROX_OPS → TERMINAL handoff at 5m ───────────────
   When truth_range < RPOD_TERMINAL_M the function must call
   through to RPOD_terminal and return what it returns.           */
static void test_prox_terminal_handoff(void) {
    printf("\nTest 3: PROX_OPS → TERMINAL handoff at range < 5m\n");

    double n_chief   = 7.292e-5;
    double accel_max = 0.020;

    RPOD_State state;
    state.pos[0] = 0.5;   /* 0.5m — inside TERMINAL_M = 5m */
    state.pos[1] = 0.0;
    state.pos[2] = 0.0;
    state.vel[0] = -0.003;   /* 3mm/s closing */
    state.vel[1] = 0.0;
    state.vel[2] = 0.0;

    double accel_prox[3], accel_term[3];

    /* PROX_OPS called with truth_range = 0.5m should fall through to TERMINAL */
    RPOD_prox_ops(&state, 0.5, n_chief, accel_max, accel_prox);

    /* Call TERMINAL directly for reference */
    RPOD_terminal_simple(&state, accel_max, accel_term);

    /* Both must give the same accel */
    double diff = fabs(accel_prox[0] - accel_term[0])
                + fabs(accel_prox[1] - accel_term[1])
                + fabs(accel_prox[2] - accel_term[2]);
    CHECK(diff < 1e-9, "PROX_OPS delegates to TERMINAL for range < 5m");
    printf("  prox=[%.5f,%.5f,%.5f]  term=[%.5f,%.5f,%.5f]\n",
           accel_prox[0], accel_prox[1], accel_prox[2],
           accel_term[0], accel_term[1], accel_term[2]);
}


/* ── Test 4: TERMINAL — deceleration law ────────────────────────
   At 0.4m with zero velocity, desired speed = k*range = 0.010*0.4
   = 0.004 m/s (below VMAX=0.005).  Deputy must be commanded to
   close at 4mm/s.                                                */
static void test_terminal_decel(void) {
    printf("\nTest 4: TERMINAL sqrt-law deceleration at 40cm\n");

    double accel_max = 0.020;

    RPOD_State state;
    state.pos[0] = 0.4;   /* 40cm, purely radial */
    state.pos[1] = 0.0;
    state.pos[2] = 0.0;
    /* Already closing at the sqrt-law desired speed:
       v_des = K_SQRT_TERM * sqrt(0.4) = 0.055902 * 0.6325 = 0.03535 m/s
       TAU at 0.4m < 0.3m → TAU_CLOSE = 5s → accel = (v_des - v_des)/5 = 0 */
    double v_des = RPOD_K_SQRT_TERM * sqrt(0.4);
    if (v_des > RPOD_V_TERM_MAX_MS) v_des = RPOD_V_TERM_MAX_MS;
    state.vel[0] = -v_des;   /* closing radially at desired speed */
    state.vel[1] = 0.0;
    state.vel[2] = 0.0;

    double accel[3];
    int ret = RPOD_terminal_simple(&state, accel_max, accel);

    CHECK(norm3(accel) < 1e-8,
          "accel ≈ 0 when already at desired closing speed");
    CHECK(ret == 0,
          "returns 0 (not docked) at 40cm");
    printf("  v_des=%.5f m/s  |accel| = %.2e m/s²\n", v_des, norm3(accel));
}


/* ── Test 5: TERMINAL — docking condition ───────────────────────
   At range < RPOD_DOCK_DONE_M RPOD_terminal returns 2 (docked). */
static void test_terminal_docking(void) {
    printf("\nTest 5: TERMINAL soft-capture request at range < %.0fcm\n",
           RPOD_DOCK_RANGE_M * 100.0);

    double accel_max = 0.020;

    RPOD_State state;
    state.pos[0] = 0.15;   /* below dock gate */
    state.pos[1] = 0.0;
    state.pos[2] = 0.0;
    state.vel[0] = -0.001;
    state.vel[1] = 0.0;
    state.vel[2] = 0.0;

    double accel[3];
    int ret = RPOD_terminal_simple(&state, accel_max, accel);

    CHECK(ret == RPOD_RET_SOFT_CAPTURE_READY,
          "TERMINAL requests SOFT_CAPTURE instead of declaring hard dock");
    CHECK(norm3(accel) > 0.0, "terminal still commands controlled approach");
    printf("  ret = %d  |accel| = %.2e\n", ret, norm3(accel));
}


/* ── Test 6: PROX_OPS — deadband inside dock gate ─────────────*/
/* Test 5b: TERMINAL - high-speed contact is not docked */
static void test_terminal_docking_speed_gate(void) {
    printf("\nTest 5b: TERMINAL docking requires low relative speed\n");

    double accel_max = 0.020;

    RPOD_State state;
    state.pos[0] = 0.15;
    state.pos[1] = 0.0;
    state.pos[2] = 0.0;
    state.vel[0] = -0.050;
    state.vel[1] = 0.0;
    state.vel[2] = 0.0;

    double accel[3];
    int ret = RPOD_terminal_simple(&state, accel_max, accel);

    CHECK(ret != 2, "TERMINAL does not dock when relative speed is too high");
    CHECK(norm3(accel) > 0.0, "guidance keeps braking/controlling instead");
    printf("  ret = %d  |accel| = %.2e\n", ret, norm3(accel));
}

/* Test 5c: TERMINAL - offset port, not CoM, is the dock target */
static void test_terminal_offset_port_target(void) {
    printf("\nTest 5c: TERMINAL docking uses offset port range\n");

    double accel_max = 0.020;
    RPOD_TermState state;
    init_term_state(&state);

    state.pos[0] = 0.0;
    state.pos[1] = 0.0;
    state.pos[2] = 0.46;
    state.vel[0] = 0.0;
    state.vel[1] = 0.0;
    state.vel[2] = 0.001;
    state.port_lvlh[0] = 0.0;
    state.port_lvlh[1] = 0.0;
    state.port_lvlh[2] = 0.50;
    state.has_port = 1;

    double accel[3];
    int brake = 0;
    int ret = RPOD_terminal(&state, accel_max, accel, &brake);

    RPOD_fill_geometry(&state);
    ret = RPOD_terminal(&state, accel_max, accel, &brake);

    CHECK(ret == RPOD_RET_SOFT_CAPTURE_READY,
          "TERMINAL uses port range to request soft capture even when CoM is about 0.46m");
    CHECK(norm3(accel) > 0.0, "terminal command remains active until hard capture");
    printf("  ret = %d  com=%.2fm  port=%.2fm\n",
           ret, norm3(state.pos),
           fabs(state.port_lvlh[2] - state.pos[2]));
}

/* Test 5d: TERMINAL - valid port prevents premature CoM dock */
static void test_terminal_port_prevents_com_dock(void) {
    printf("\nTest 5d: TERMINAL valid port prevents premature CoM docking\n");

    double accel_max = 0.020;
    RPOD_TermState state;
    init_term_state(&state);

    state.pos[0] = 0.0;
    state.pos[1] = 0.0;
    state.pos[2] = 0.15;
    state.vel[0] = 0.0;
    state.vel[1] = 0.0;
    state.vel[2] = 0.001;
    state.port_lvlh[0] = 0.0;
    state.port_lvlh[1] = 0.0;
    state.port_lvlh[2] = 0.50;
    state.has_port = 1;

    double accel[3];
    int brake = 0;
    int ret = RPOD_terminal(&state, accel_max, accel, &brake);

    CHECK(ret != 2, "TERMINAL waits for port range, not CoM range");
    CHECK(norm3(accel) > 0.0, "guidance commands motion toward the port");
    printf("  ret = %d  com=%.2fm  port=%.2fm\n",
           ret, norm3(state.pos),
           fabs(state.port_lvlh[2] - state.pos[2]));
}

static void test_prox_deadband(void) {
    printf("\nTest 6: PROX_OPS deadband below dock gate\n");

    double n_chief   = 7.292e-5;
    double accel_max = 0.020;

    RPOD_State state;
    state.pos[0] = 0.03;   /* 3cm */
    state.pos[1] = 0.0;
    state.pos[2] = 0.0;
    state.vel[0] = 0.0;
    state.vel[1] = 0.0;
    state.vel[2] = 0.0;

    /* truth_range = 0.03 < TERMINAL_M = 5.0, so PROX falls through
       to TERMINAL, which returns 1 and zeros accel */
    double accel[3];
    RPOD_prox_ops(&state, 0.03, n_chief, accel_max, accel);
    CHECK(norm3(accel) > 0.0,
          "terminal path remains controlled inside the soft-capture zone");
}

static void test_terminal_soft_capture_hold(void) {
    printf("\nTest 8: SOFT_CAPTURE damps to hard-capture gate\n");

    double accel_max = 0.020;
    RPOD_TermState state;
    init_term_state(&state);
    state.pos[0] = 0.0;
    state.pos[1] = 0.0;
    state.pos[2] = 0.046;
    state.vel[0] = 0.0;
    state.vel[1] = 0.0;
    state.vel[2] = 0.0005;
    state.port_lvlh[0] = 0.0;
    state.port_lvlh[1] = 0.0;
    state.port_lvlh[2] = 0.050;
    state.has_port = 1;
    RPOD_fill_geometry(&state);

    double accel[3];
    int ret = RPOD_soft_capture(&state, accel_max, accel);

    CHECK(ret == RPOD_RET_DOCKED,
          "SOFT_CAPTURE reports hard-capture-ready for low range and low vrel");
    CHECK(norm3(accel) > 0.0,
          "SOFT_CAPTURE applies spring-damper capture command");
    printf("  ret = %d  |accel| = %.2e\n", ret, norm3(accel));
}


/* ── Test 7: TERMINAL VMAX cap ──────────────────────────────────
   At 8m from chief, sqrt-law speed exceeds the terminal speed cap.
   Desired speed must be capped at RPOD_TERMINAL_VMAX.           */
static void test_terminal_vmax(void) {
    printf("\nTest 7: TERMINAL speed capped at VMAX=25mm/s\n");

    double accel_max = 0.020;

    RPOD_State state;
    state.pos[0] = 0.0;
    state.pos[1] = -8.0;   /* 8m along-track */
    state.pos[2] = 0.0;
    state.vel[0] = 0.0;
    state.vel[1] = RPOD_TERMINAL_VMAX;   /* already at VMAX — accel ≈ 0 */
    state.vel[2] = 0.0;

    double accel[3];
    RPOD_terminal_simple(&state, accel_max, accel);

    double vel_des_mag = RPOD_K_SQRT_TERM * sqrt(8.0);
    CHECK(vel_des_mag > RPOD_TERMINAL_VMAX,
          "sqrt-law speed > VMAX: cap is needed at 8m");
    /* vel_des = [0, +VMAX, 0], vel = [0, +VMAX, 0] → accel ≈ 0 */
    CHECK(norm3(accel) < 1e-8,
          "accel ≈ 0 when already at VMAX closing speed");
    printf("  vel_des_mag = %.4f  VMAX = %.4f  |accel| = %.2e\n",
           vel_des_mag, RPOD_TERMINAL_VMAX, norm3(accel));
}


/* ── main ─────────────────────────────────────────────────────── */
int main(void) {
    printf("=== RPOD Controller C Verification ===\n\n");

    test_prox_speed_profile();
    test_prox_speed_50m();
    test_prox_terminal_handoff();
    test_terminal_decel();
    test_terminal_docking();
    test_terminal_docking_speed_gate();
    test_terminal_offset_port_target();
    test_terminal_port_prevents_com_dock();
    test_prox_deadband();
    test_terminal_vmax();
    test_terminal_soft_capture_hold();

    printf("\n=== %s (%d failures) ===\n",
           n_fail == 0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}
