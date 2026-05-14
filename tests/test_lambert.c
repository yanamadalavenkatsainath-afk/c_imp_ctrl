/**
 * test_lambert.c — Lambert solver C verification
 * ================================================
 * Mirrors the validation block in lambert_solver.py __main__.
 *
 * Compile:
 *   gcc -O2 -Isrc_c -o test_lambert.exe tests/test_lambert.c src_c/lambert_solver.c -lm
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "lambert_solver.h"

#define CHECK(cond, msg) \
    do { printf("  %s — %s\n",(cond)?"✓ PASS":"✗ FAIL",msg); \
         if(!(cond)) n_fail++; } while(0)

static int n_fail = 0;

static double norm3(const double v[3]) {
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

/* ── Test 1: Circular LEO quarter-orbit ───────────────────────── */
static void test_leo_quarter_orbit(void) {
    printf("Test 1: Circular LEO — quarter-orbit velocity accuracy\n");

    double mu  = LAMBERT_MU_DEFAULT;
    double a   = 7000e3;
    double v_c = sqrt(mu / a);
    double T   = 2.0*M_PI*sqrt(a*a*a/mu);

    double r1[3] = {a,   0., 0.};
    double r2[3] = {0.,  a,  0.};

    Lambert_Result lr = LAMBERT_solve(r1, r2, T/4.0, 1, mu);

    printf("  ok = %d\n", lr.ok);
    if (lr.ok) {
        double v1_exp[3] = {0., v_c, 0.};
        double v2_exp[3] = {-v_c, 0., 0.};
        double e1=0., e2=0.;
        for(int i=0;i<3;i++){
            e1+=(lr.v1[i]-v1_exp[i])*(lr.v1[i]-v1_exp[i]);
            e2+=(lr.v2[i]-v2_exp[i])*(lr.v2[i]-v2_exp[i]);
        }
        e1=sqrt(e1); e2=sqrt(e2);
        printf("  v1 error: %.6f m/s\n", e1);
        printf("  v2 error: %.6f m/s\n", e2);
        CHECK(lr.ok == 1,    "solution found");
        CHECK(e1 < 0.01,     "v1 error < 10 mm/s (< 0.01 m/s)");
        CHECK(e2 < 0.01,     "v2 error < 10 mm/s");
    } else {
        CHECK(0, "solution found (ok=0 — FAIL)");
    }
}

/* ── Test 2: GEO 1km catch-up ─────────────────────────────────── */
static void test_geo_catchup(void) {
    printf("\nTest 2: GEO 1km catch-up — minimum ΔV < 15 mm/s\n");

    double mu    = LAMBERT_MU_DEFAULT;
    double a_geo = 42164e3;
    double v_geo = sqrt(mu / a_geo);
    double T_geo = 2.0*M_PI*sqrt(a_geo*a_geo*a_geo/mu);
    double n_geo = sqrt(mu/(a_geo*a_geo*a_geo));

    double angle_dep = -1000.0/a_geo;
    double r_dep[3] = {a_geo*cos(angle_dep), a_geo*sin(angle_dep), 0.};
    double v_dep[3] = {-v_geo*sin(angle_dep), v_geo*cos(angle_dep), 0.};

    double best_total = 1e30, best_tof = 0.;
    double best_dv1=0., best_dv2=0.;

    int n_scan = 80;
    for (int i = 0; i < n_scan; i++) {
        double frac = 0.05 + (0.90-0.05)*(double)i/(double)(n_scan-1);
        double tof  = frac * T_geo;
        double angle_f = n_geo * tof;
        double r_chi_f[3] = {a_geo*cos(angle_f), a_geo*sin(angle_f), 0.};
        double v_chi_f[3] = {-v_geo*sin(angle_f), v_geo*cos(angle_f), 0.};

        Lambert_Result lr = LAMBERT_solve(r_dep, r_chi_f, tof, 1, mu);
        if (!lr.ok) continue;

        double dv1=0., dv2=0.;
        for(int j=0;j<3;j++){
            double d1=lr.v1[j]-v_dep[j], d2=v_chi_f[j]-lr.v2[j];
            dv1+=d1*d1; dv2+=d2*d2;
        }
        dv1=sqrt(dv1); dv2=sqrt(dv2);
        double total=dv1+dv2;
        if (total < best_total) {
            best_total=total; best_tof=tof;
            best_dv1=dv1; best_dv2=dv2;
        }
    }

    printf("  Best TOF: %.2f hr  (%.0f%% of orbit)\n",
           best_tof/3600., best_tof/T_geo*100.);
    printf("  |dv1| = %.3f mm/s\n", best_dv1*1000.);
    printf("  |dv2| = %.3f mm/s\n", best_dv2*1000.);
    printf("  Total = %.3f mm/s\n", best_total*1000.);
    CHECK(best_total < 0.1, "total ΔV < 100 mm/s  (expect ~8-15 mm/s)");
    CHECK(best_total*1000. < 15.0, "total ΔV < 15 mm/s");
}

/* ── Test 3: Keplerian propagator — 1 GEO orbit drift ─────────── */
static void test_keplerian_propagator(void) {
    printf("\nTest 3: Keplerian propagator — 1 GEO orbit radius drift\n");

    double mu    = LAMBERT_MU_DEFAULT;
    double a_geo = 42164e3;
    double v_geo = sqrt(mu / a_geo);
    double T_geo = 2.0*M_PI*sqrt(a_geo*a_geo*a_geo/mu);

    double r0[3] = {a_geo, 0., 0.};
    double v0[3] = {0., v_geo, 0.};
    double r1[3], v1[3];

    LAMBERT_propagate_keplerian(r0, v0, T_geo, 200, mu, r1, v1);

    double drift=0.;
    for(int i=0;i<3;i++) drift+=(r1[i]-r0[i])*(r1[i]-r0[i]);
    drift=sqrt(drift);
    printf("  Radius drift after 1 GEO orbit: %.2f m\n", drift);
    CHECK(drift < 1000., "drift < 1000 m after full GEO orbit");
}

/* ── Test 4: LAMBERT_min_dv wrapper ───────────────────────────── */
static void test_min_dv_wrapper(void) {
    printf("\nTest 4: LAMBERT_min_dv wrapper — GEO 500m catch-up\n");

    double mu    = LAMBERT_MU_DEFAULT;
    double a_geo = 42164e3;
    double v_geo = sqrt(mu / a_geo);
    double T_geo = 2.0*M_PI*sqrt(a_geo*a_geo*a_geo/mu);
    double n_geo = sqrt(mu/(a_geo*a_geo*a_geo));

    double angle_dep = -500.0/a_geo;
    double r_dep[3] = {a_geo*cos(angle_dep), a_geo*sin(angle_dep), 0.};
    double v_dep[3] = {-v_geo*sin(angle_dep), v_geo*cos(angle_dep), 0.};

    /* Chief future state at tof_max */
    double tof_max = 0.5*T_geo;
    double angle_f = n_geo * tof_max;
    double r_chi_f[3] = {a_geo*cos(angle_f), a_geo*sin(angle_f), 0.};
    double v_chi_f[3] = {-v_geo*sin(angle_f), v_geo*cos(angle_f), 0.};

    Lambert_MinDV res = LAMBERT_min_dv(r_dep, v_dep, r_chi_f, v_chi_f,
                                        0.05*T_geo, tof_max,
                                        80, 0.5, mu);
    printf("  ok = %d  total_dv = %.3f mm/s\n", res.ok, res.total_dv*1000.);
    CHECK(res.ok == 1, "min_dv wrapper finds a solution");
    CHECK(res.total_dv < 0.1, "total ΔV < 100 mm/s");
}

/* ── Test 5: Degenerate (0° transfer) returns ok=0 ───────────── */
static void test_degenerate(void) {
    printf("\nTest 5: Degenerate geometry (r1 == r2) returns ok=0\n");
    double mu = LAMBERT_MU_DEFAULT;
    double r1[3] = {42164e3, 0., 0.};
    double r2[3] = {42164e3, 0., 0.};
    Lambert_Result lr = LAMBERT_solve(r1, r2, 1000., 1, mu);
    printf("  ok = %d  (expect 0)\n", lr.ok);
    CHECK(lr.ok == 0, "degenerate case returns ok=0 (no crash)");
}

/* ── main ─────────────────────────────────────────────────────── */
int main(void) {
    printf("=== Lambert Solver C Verification ===\n\n");
    test_leo_quarter_orbit();
    test_geo_catchup();
    test_keplerian_propagator();
    test_min_dv_wrapper();
    test_degenerate();
    printf("\n=== %s (%d failures) ===\n",
           n_fail==0?"ALL PASS":"FAILURES DETECTED", n_fail);
    return n_fail==0?0:1;
}