/**
 * test_mode_manager.c — FSW Mode Manager C verification
 * ======================================================
 * Compile:
 *   gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_mode_manager.exe \
 *       tests/test_mode_manager.c src_c/mode_manager.c -lm
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "mode_manager.h"

#define CHECK(cond, msg) \
    do { printf("  %s — %s\n",(cond)?"✓ PASS":"✗ FAIL",msg); \
         if(!(cond)) n_fail++; } while(0)

static int n_fail = 0;

/* ── Test 1: Init starts in DETUMBLE ──────────────────────────── */
static void test_init(void) {
    printf("Test 1: Init state is DETUMBLE\n");
    MM_State mm;
    MM_init(&mm);
    CHECK(mm.mode == MODE_DETUMBLE, "mode == DETUMBLE after init");
    printf("  mode = %s\n", MM_mode_name(mm.mode));
}


/* ── Test 2: High rate → SAFE_MODE ──────────────────────────── */
static void test_safe_mode_entry(void) {
    printf("\nTest 2: High angular rate triggers SAFE_MODE\n");
    MM_State mm;
    MM_init(&mm);

    /* 45 deg/s = 0.785 rad/s > threshold (40 deg/s = 0.698 rad/s) */
    double omega[3] = {0.785, 0.0, 0.0};
    double h[3]     = {0.0, 0.0, 0.0};

    FSW_Mode mode = MM_update(&mm, 0.0, omega, h, -1.0, 0, -1.0);
    CHECK(mode == MODE_SAFE_MODE, "rate > threshold → SAFE_MODE");
    printf("  mode = %s\n", MM_mode_name(mode));
}


/* ── Test 3: DETUMBLE → SUN_ACQUISITION ─────────────────────── */
static void test_detumble_exit(void) {
    printf("\nTest 3: Rate below threshold → DETUMBLE exits to SUN_ACQUISITION\n");
    MM_State mm;
    MM_init(&mm);

    /* Rate below DETUMBLE_THRESHOLD (3.5 deg/s = 0.0611 rad/s) */
    double omega[3] = {0.01, 0.0, 0.0};   /* 0.57 deg/s */
    double h[3]     = {0.0, 0.0, 0.0};

    FSW_Mode mode = MM_update(&mm, 100.0, omega, h, -1.0, 0, -1.0);
    CHECK(mode == MODE_SUN_ACQUISITION, "low rate exits DETUMBLE → SUN_ACQUISITION");
    printf("  mode = %s\n", MM_mode_name(mode));
}


/* ── Test 4: SUN_ACQUISITION → FINE_POINTING (QUEST ok) ─────── */
static void test_sun_acq_to_fine(void) {
    printf("\nTest 4: QUEST error < threshold → SUN_ACQ → FINE_POINTING\n");
    MM_State mm;
    MM_init(&mm);
    /* Force into SUN_ACQUISITION */
    mm.mode = MODE_SUN_ACQUISITION;
    mm.mode_entry_t = 0.0;

    double omega[3] = {0.01, 0.0, 0.0};
    double h[3]     = {0.0, 0.0, 0.0};
    /* quest_err = 5° < threshold (15°) */
    FSW_Mode mode = MM_update(&mm, 10.0, omega, h, 5.0, 0, 2.0);
    CHECK(mode == MODE_FINE_POINTING, "quest_err < 15° → FINE_POINTING");
    printf("  mode = %s\n", MM_mode_name(mode));
}


/* ── Test 5: SUN_ACQUISITION timeout → FINE_POINTING ────────── */
static void test_sun_acq_timeout(void) {
    printf("\nTest 5: SUN_ACQ timeout (600s) forces FINE_POINTING\n");
    MM_State mm;
    MM_init(&mm);
    mm.mode = MODE_SUN_ACQUISITION;
    mm.mode_entry_t = 0.0;

    double omega[3] = {0.01, 0.0, 0.0};
    double h[3]     = {0.0, 0.0, 0.0};
    /* No QUEST result, but t=700s > 600s timeout */
    FSW_Mode mode = MM_update(&mm, 700.0, omega, h, -1.0, 0, -1.0);
    CHECK(mode == MODE_FINE_POINTING, "timeout → FINE_POINTING");
    printf("  mode = %s\n", MM_mode_name(mode));
}


/* ── Test 6: FINE_POINTING → MOMENTUM_DUMP ──────────────────── */
static void test_momentum_dump_entry(void) {
    printf("\nTest 6: Wheel saturation → MOMENTUM_DUMP (when pointing ok)\n");
    MM_State mm;
    MM_init(&mm);
    mm.mode = MODE_FINE_POINTING;

    double omega[3] = {0.01, 0.0, 0.0};
    /* h[0] > DUMP_TRIGGER (3.0 N·m·s) */
    double h[3]     = {3.5, 0.0, 0.0};
    /* pointing error < DUMP_POINTING_GUARD (5°) so dump is allowed */
    FSW_Mode mode = MM_update(&mm, 500.0, omega, h, -1.0, 0, 2.0);
    CHECK(mode == MODE_MOMENTUM_DUMP, "h > DUMP_TRIGGER → MOMENTUM_DUMP");
    printf("  mode = %s  h_max=%.1f\n", MM_mode_name(mode), h[0]);
}


/* ── Test 7: MOMENTUM_DUMP → FINE_POINTING when h drops ─────── */
static void test_momentum_dump_exit(void) {
    printf("\nTest 7: Wheels dumped → MOMENTUM_DUMP exits to FINE_POINTING\n");
    MM_State mm;
    MM_init(&mm);
    mm.mode = MODE_MOMENTUM_DUMP;
    mm.mode_entry_t = 0.0;

    double omega[3] = {0.01, 0.0, 0.0};
    /* h < DUMP_COMPLETE (0.8 N·m·s) */
    double h[3]     = {0.5, 0.0, 0.0};
    FSW_Mode mode = MM_update(&mm, 600.0, omega, h, -1.0, 0, 2.0);
    CHECK(mode == MODE_FINE_POINTING, "h < DUMP_COMPLETE → FINE_POINTING");
    printf("  mode = %s\n", MM_mode_name(mode));
}


/* ── Test 8: MOMENTUM_DUMP blocked when pointing bad ─────────── */
static void test_dump_pointing_guard(void) {
    printf("\nTest 8: MOMENTUM_DUMP blocked when pointing_err > 5°\n");
    MM_State mm;
    MM_init(&mm);
    mm.mode = MODE_FINE_POINTING;

    double omega[3] = {0.01, 0.0, 0.0};
    double h[3]     = {3.5, 0.0, 0.0};   /* would trigger dump */
    /* pointing error = 10° > 5° guard → dump blocked */
    FSW_Mode mode = MM_update(&mm, 500.0, omega, h, -1.0, 0, 10.0);
    CHECK(mode == MODE_FINE_POINTING, "dump blocked when pointing_err > guard");
    printf("  mode = %s  (dump prevented)\n", MM_mode_name(mode));
}


/* ── Test 9: SAFE_MODE recovery when rate drops ─────────────── */
static void test_safe_mode_recovery(void) {
    printf("\nTest 9: SAFE_MODE recovers to DETUMBLE when rate drops\n");
    MM_State mm;
    MM_init(&mm);
    mm.mode = MODE_SAFE_MODE;

    /* Rate < DETUMBLE_THRESHOLD * 5 = 17.5 deg/s */
    double omega[3] = {0.1, 0.0, 0.0};   /* ~5.7 deg/s */
    double h[3]     = {0.0, 0.0, 0.0};
    FSW_Mode mode = MM_update(&mm, 10.0, omega, h, -1.0, 0, -1.0);
    CHECK(mode == MODE_DETUMBLE, "SAFE_MODE recovers → DETUMBLE");
    printf("  mode = %s\n", MM_mode_name(mode));
}


/* ── Test 10: Full sequence simulation ───────────────────────── */
static void test_full_sequence(void) {
    printf("\nTest 10: Full sequence DETUMBLE → SUN_ACQ → FINE_POINTING\n");
    MM_State mm;
    MM_init(&mm);

    double t = 0.0;
    double omega[3] = {0.5, 0.3, 0.2};   /* 34 deg/s initial — below SAFE threshold */
    double h[3]     = {0.0, 0.0, 0.0};

    FSW_Mode final_mode = MODE_DETUMBLE;

    /* Phase 1: detumble — rate decays */
    for(int i=0; i<200; i++) {
        t += 1.0;
        omega[0] *= 0.97; omega[1] *= 0.97; omega[2] *= 0.97;
        final_mode = MM_update(&mm, t, omega, h, -1.0, 0, -1.0);
    }
    printf("  After 200s: mode=%s  rate=%.3f deg/s\n",
           MM_mode_name(final_mode),
           sqrt(omega[0]*omega[0]+omega[1]*omega[1]+omega[2]*omega[2])*180.0/3.14159);

    /* Phase 2: sun acquisition — provide QUEST result */
    for(int i=0; i<10; i++) {
        t += 1.0;
        final_mode = MM_update(&mm, t, omega, h, 8.0, 0, 3.0);
    }
    printf("  After QUEST: mode=%s\n", MM_mode_name(final_mode));

    CHECK(final_mode == MODE_FINE_POINTING, "reaches FINE_POINTING after full sequence");

    /* Check history logged correctly */
    printf("  Transition log:\n");
    for(int i=0; i<mm.history_count; i++)
        printf("    t=%.1fs → %s\n", mm.history[i].t, MM_mode_name(mm.history[i].mode));
    CHECK(mm.history_count >= 2, "at least 2 transitions logged");
}


/* ── main ─────────────────────────────────────────────────────── */
int main(void) {
    printf("=== Mode Manager C Verification ===\n\n");

    test_init();
    test_safe_mode_entry();
    test_detumble_exit();
    test_sun_acq_to_fine();
    test_sun_acq_timeout();
    test_momentum_dump_entry();
    test_momentum_dump_exit();
    test_dump_pointing_guard();
    test_safe_mode_recovery();
    test_full_sequence();

    printf("\n=== %s (%d failures) ===\n",
           n_fail==0 ? "ALL PASS" : "FAILURES DETECTED", n_fail);
    return n_fail == 0 ? 0 : 1;
}