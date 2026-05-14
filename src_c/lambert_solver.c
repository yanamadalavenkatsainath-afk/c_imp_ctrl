/**
 * lambert_solver.c — Lambert's Problem Solver (Universal Variable)
 * =================================================================
 * Port of lambert_solver.py — Curtis Algorithm 5.2 / Stumpff functions.
 *
 * PATCH P3 — Stumpff series threshold and zero-division guards:
 *
 *   Issue: The original threshold for switching to power-series was 1e-6.
 *   Curtis Alg 5.2 (and Python lambert_solver.py) uses 1e-6 in the Python
 *   version, but the probe requirement (P3) requires the series branch for
 *   |z| < 1e-4 to avoid cancellation errors near z=0 when using double
 *   precision. Tightening to 1e-4 matches the Python golden model output
 *   for near-parabolic transfers (z ≈ 0).
 *
 *   Additional guard: r_norm³ in the RK4 propagator could be zero if a
 *   degenerate position vector is passed.  Added 1e-12 epsilon to the
 *   denominator — safe for all physical inputs (|r| > Earth radius).
 *
 *   No malloc. All temporaries stack-allocated.
 */

#include "lambert_solver.h"
#include <math.h>
#include <string.h>

/* ── Internal math helpers ────────────────────────────────────── */

static double _norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static double _dot3(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void _cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* ── Stumpff functions — P3 FIX: threshold widened to 1e-4 ──── */
/*
 * P3 requirement: use power series for |z| < 1e-4.
 *
 * Rationale: for |z| < 1e-6 the trig/hyperbolic branches compute
 *   cos(sqrt(z)) ≈ 1 - z/2 + ...  and then (1 - cos)/z = 0.5 - z/24
 * Both paths agree, but the trig path has relative cancellation error
 *   ε_rel ≈ eps_machine / |z|
 * which exceeds 1e-8 for |z| < 1e-8.  With z ∈ (1e-8, 1e-6) the error
 * is borderline.  Widening to 1e-4 ensures the series path (which has
 * no cancellation) is used in the entire danger zone, matching the
 * Python __main__ validation result that requires < 1mm/s velocity error.
 */

/**
 * C2(z):  (1 - cos√z)/z   for z > 1e-4
 *         (cosh√(-z)-1)/(-z) for z < -1e-4
 *         0.5 - z/24 + z²/720 - z³/40320  for |z| ≤ 1e-4
 */
static double _C2(double z) {
    if (z > 1e-4) {
        return (1.0 - cos(sqrt(z))) / z;
    } else if (z < -1e-4) {
        double sq = sqrt(-z);
        return (cosh(sq) - 1.0) / (-z);
    }
    /* Power series — 4 terms sufficient for |z| < 1e-4 with double precision */
    return 0.5 - z/24.0 + (z*z)/720.0 - (z*z*z)/40320.0;
}

/**
 * C3(z):  (√z - sin√z)/z^(3/2)       for z > 1e-4
 *         (sinh√(-z) - √(-z))/(-z)^(3/2) for z < -1e-4
 *         1/6 - z/120 + z²/5040 - z³/362880 for |z| ≤ 1e-4
 */
static double _C3(double z) {
    if (z > 1e-4) {
        double sq = sqrt(z);
        return (sq - sin(sq)) / (sq*sq*sq + 1e-12);   /* P3: +epsilon */
    } else if (z < -1e-4) {
        double sq = sqrt(-z);
        return (sinh(sq) - sq) / (sq*sq*sq + 1e-12);  /* P3: +epsilon */
    }
    return 1.0/6.0 - z/120.0 + (z*z)/5040.0 - (z*z*z)/362880.0;
}

/* ── TOF as function of z — for bisection ────────────────────── */

typedef struct {
    double r1, r2, A, mu;
} _TOF_Ctx;

static double _tof_from_z(double z, const _TOF_Ctx *ctx) {
    double C2z = _C2(z);
    double C3z = _C3(z);
    if (C2z < 1e-12) return 1e30;
    double y = ctx->r1 + ctx->r2 + ctx->A * (z*C3z - 1.0) / sqrt(C2z + 1e-12);
    if (y < 0.0) return 1e30;
    double chi2 = y / C2z;
    double chi  = sqrt(chi2);
    return (chi*chi*chi*C3z + ctx->A*sqrt(y)) / sqrt(ctx->mu);
}

/* ── Bisection on f(z) = tgt — matches Python _bisect_z ─────── */

static double _bisect_z(const _TOF_Ctx *ctx,
                         double tgt,
                         double z_lo, double z_hi) {
    double f_lo = _tof_from_z(z_lo, ctx) - tgt;
    double f_hi = _tof_from_z(z_hi, ctx) - tgt;

    for (int i = 0; i < 20; i++) {
        if (f_lo * f_hi < 0.0) break;
        z_hi *= 2.0;
        f_hi = _tof_from_z(z_hi, ctx) - tgt;
    }
    if (f_lo * f_hi >= 0.0) {
        for (int i = 0; i < 20; i++) {
            z_lo *= 2.0;
            f_lo = _tof_from_z(z_lo, ctx) - tgt;
            if (f_lo * f_hi < 0.0) break;
        }
    }

    for (int i = 0; i < 500; i++) {
        double z_mid = (z_lo + z_hi) / 2.0;
        double f_mid = _tof_from_z(z_mid, ctx) - tgt;
        if (fabs(f_mid) < 0.01) return z_mid;
        if (f_lo * f_mid < 0.0) {
            z_hi = z_mid; f_hi = f_mid;
        } else {
            z_lo = z_mid; f_lo = f_mid;
        }
    }
    return (z_lo + z_hi) / 2.0;
}

/* ── Public: LAMBERT_solve ───────────────────────────────────── */

Lambert_Result LAMBERT_solve(const double r1_vec[3],
                              const double r2_vec[3],
                              double tof,
                              int prograde,
                              double mu) {
    Lambert_Result res;
    memset(&res, 0, sizeof(res));
    res.ok = 0;

    double r1 = _norm3(r1_vec);
    double r2 = _norm3(r2_vec);
    if (r1 < 1.0 || r2 < 1.0 || tof <= 0.0) return res;

    double cos_dnu = _dot3(r1_vec, r2_vec) / (r1 * r2 + 1e-12);
    if (cos_dnu >  1.0) cos_dnu =  1.0;
    if (cos_dnu < -1.0) cos_dnu = -1.0;
    double dnu = acos(cos_dnu);

    double cross[3];
    _cross3(r1_vec, r2_vec, cross);
    if (prograde && cross[2] < 0.0)
        dnu = 2.0*M_PI - dnu;
    else if (!prograde && cross[2] >= 0.0)
        dnu = 2.0*M_PI - dnu;

    double sin_dnu = sin(dnu);
    double one_minus_cos = 1.0 - cos_dnu;
    if (fabs(one_minus_cos) < 1e-12) return res;
    double A = sin_dnu * sqrt(r1 * r2 / one_minus_cos);
    if (fabs(A) < 1e-6) return res;

    _TOF_Ctx ctx = {r1, r2, A, mu};
    double z = _bisect_z(&ctx, tof, -4.0*M_PI*M_PI, 4.0*M_PI*M_PI);

    double C2z = _C2(z);
    double C3z = _C3(z);
    if (C2z < 1e-12) return res;
    double y = r1 + r2 + A * (z*C3z - 1.0) / sqrt(C2z + 1e-12);
    if (y < 0.0) return res;

    double f    = 1.0 - y / r1;
    double g    = A * sqrt(y / mu);
    double gdot = 1.0 - y / r2;

    if (fabs(g) < 1e-12) return res;

    for (int i = 0; i < 3; i++) {
        res.v1[i] = (r2_vec[i] - f * r1_vec[i]) / g;
        res.v2[i] = (gdot * r2_vec[i] - r1_vec[i]) / g;
    }
    res.ok = 1;
    return res;
}

/* ── Public: LAMBERT_propagate_keplerian ─────────────────────── */

void LAMBERT_propagate_keplerian(const double pos_in[3],
                                  const double vel_in[3],
                                  double dt,
                                  int n_steps,
                                  double mu,
                                  double pos_out[3],
                                  double vel_out[3]) {
    if (fabs(dt) < 1e-3 || n_steps < 1) {
        for (int i = 0; i < 3; i++) {
            pos_out[i] = pos_in[i];
            vel_out[i] = vel_in[i];
        }
        return;
    }

    double p[3], v[3];
    for (int i = 0; i < 3; i++) { p[i] = pos_in[i]; v[i] = vel_in[i]; }

    double h = dt / (double)n_steps;

/* P3: helper macro — accel from position with epsilon guard on |r|³ */
#define _ACCEL(pv, av)                                                     \
    do {                                                                   \
        double _r2 = (pv)[0]*(pv)[0]+(pv)[1]*(pv)[1]+(pv)[2]*(pv)[2];   \
        double _r3 = sqrt(_r2)*_r2 + 1e-12;  /* |r|³ + epsilon */        \
        for (int _i=0;_i<3;_i++) (av)[_i] = -mu/_r3*(pv)[_i];           \
    } while(0)

    for (int step = 0; step < n_steps; step++) {
        double k1p[3], k1v[3], p2[3], k2p[3], k2v[3];
        double p3[3], k3p[3], k3v[3], p4[3], k4p[3], k4v[3];

        /* k1 */
        for (int i=0;i<3;i++) k1p[i] = v[i];
        _ACCEL(p, k1v);

        /* k2 */
        for (int i=0;i<3;i++) { p2[i]=p[i]+0.5*h*k1p[i]; k2p[i]=v[i]+0.5*h*k1v[i]; }
        _ACCEL(p2, k2v);

        /* k3 */
        for (int i=0;i<3;i++) { p3[i]=p[i]+0.5*h*k2p[i]; k3p[i]=v[i]+0.5*h*k2v[i]; }
        _ACCEL(p3, k3v);

        /* k4 */
        for (int i=0;i<3;i++) { p4[i]=p[i]+h*k3p[i]; k4p[i]=v[i]+h*k3v[i]; }
        _ACCEL(p4, k4v);

        for (int i=0;i<3;i++) {
            p[i] += (h/6.0)*(k1p[i]+2.0*k2p[i]+2.0*k3p[i]+k4p[i]);
            v[i] += (h/6.0)*(k1v[i]+2.0*k2v[i]+2.0*k3v[i]+k4v[i]);
        }
    }
#undef _ACCEL

    for (int i = 0; i < 3; i++) {
        pos_out[i] = p[i];
        vel_out[i] = v[i];
    }
}

/* ── Public: LAMBERT_min_dv ──────────────────────────────────── */

Lambert_MinDV LAMBERT_min_dv(const double r1_dep[3],
                               const double v1_dep[3],
                               const double r2_chief[3],
                               const double v2_chief[3],
                               double tof_min,
                               double tof_max,
                               int n_scan,
                               double dv_cap,
                               double mu) {
    Lambert_MinDV out;
    memset(&out, 0, sizeof(out));
    out.total_dv = HUGE_VAL;
    out.best_tof = NAN;
    out.ok = 0;

    if (n_scan < 1) n_scan = 80;
    double step = (n_scan > 1) ? (tof_max - tof_min) / (double)(n_scan - 1) : 0.0;

    for (int i = 0; i < n_scan; i++) {
        double tof = tof_min + i * step;

        Lambert_Result lr = LAMBERT_solve(r1_dep, r2_chief, tof, 1, mu);
        if (!lr.ok) continue;

        double dv1[3], dv2[3];
        double m1 = 0.0, m2 = 0.0;
        for (int j = 0; j < 3; j++) {
            dv1[j] = lr.v1[j] - v1_dep[j];
            dv2[j] = v2_chief[j] - lr.v2[j];
            m1 += dv1[j]*dv1[j];
            m2 += dv2[j]*dv2[j];
        }
        m1 = sqrt(m1);
        m2 = sqrt(m2);

        if (m1 > dv_cap || m2 > dv_cap) continue;

        double total = m1 + m2;
        if (total < out.total_dv) {
            out.total_dv = total;
            out.best_tof = tof;
            for (int j = 0; j < 3; j++) {
                out.dv1[j] = dv1[j];
                out.dv2[j] = dv2[j];
            }
            out.ok = 1;
        }
    }

    return out;
}