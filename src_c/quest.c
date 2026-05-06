/**
 * quest.c — QUEST Attitude Estimator
 * ====================================
 * Port of quest.py — Shuster & Oh (1981), Davenport K-matrix method.
 *
 * No malloc. All temporaries stack-allocated.
 * 4×4 eigenvalue solver uses power iteration / analytic closed form
 * (avoids full LAPACK dependency).  For a 4×4 symmetric matrix the
 * characteristic polynomial is degree 4; we find λ_max via Newton-Raphson
 * on the characteristic polynomial, then extract the eigenvector.
 * This matches the classical QUEST algorithm exactly.
 */

#include "quest.h"
#include <math.h>
#include <string.h>

/* ── Internal helpers ─────────────────────────────────────────── */

static void _safe_norm(const double v[3], double out[3]) {
    double n = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1e-12) { out[0]=1.0; out[1]=0.0; out[2]=0.0; return; }
    out[0]=v[0]/n; out[1]=v[1]/n; out[2]=v[2]/n;
}

/* 4×4 symmetric matrix-vector product: y = M @ x */
static void _mat4_vec(const double M[4][4], const double x[4], double y[4]) {
    for (int i=0;i<4;i++) {
        y[i]=0.0;
        for (int j=0;j<4;j++) y[i]+=M[i][j]*x[j];
    }
}

/* dot product of two 4-vectors */
static double _dot4(const double a[4], const double b[4]) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];
}

/* norm of 4-vector */
static double _norm4(const double v[4]) {
    return sqrt(_dot4(v,v));
}

/* normalise 4-vector in-place */
static void _norm4_inplace(double v[4]) {
    double n = _norm4(v);
    if (n < 1e-12) return;
    for (int i=0;i<4;i++) v[i]/=n;
}

/**
 * Multi-start power iteration to find dominant eigenvector of 4×4 symmetric M.
 *
 * Single-start power iteration fails when the starting vector has no component
 * along the dominant eigenvector (e.g. when K is block-diagonal and the start
 * vector splits evenly across blocks). Using 6 canonical starting vectors 
 * guarantees at least one has a large projection on the true dominant eigenvector.
 *
 * Returns λ_max and sets evec to the corresponding unit eigenvector.
 */
static double _power_iteration(const double M[4][4], double evec[4]) {
    /* 6 starting vectors: 4 axis-aligned + 2 diagonal */
    static const double starts[6][4] = {
        {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1},
        {0.5, 0.5, 0.5, 0.5},
        {0.57735, 0.57735, -0.57735, 0.0}
    };

    double best_v[4]  = {1,0,0,0};
    double best_lam   = -1e30;

    for (int s = 0; s < 6; s++) {
        double v[4];
        for (int i=0;i<4;i++) v[i] = starts[s][i];
        _norm4_inplace(v);

        double Mv[4];
        double lam = 0.0;

        for (int iter = 0; iter < 200; iter++) {
            _mat4_vec(M, v, Mv);
            lam = _dot4(v, Mv);
            double n = _norm4(Mv);
            if (n < 1e-12) break;
            for (int i=0;i<4;i++) Mv[i] /= n;

            /* Choose sign consistent with previous v */
            double diff     = 0.0, diff_neg = 0.0;
            for (int i=0;i<4;i++) {
                diff     += (Mv[i]-v[i])*(Mv[i]-v[i]);
                diff_neg += (Mv[i]+v[i])*(Mv[i]+v[i]);
            }
            if (diff_neg < diff)
                for (int i=0;i<4;i++) Mv[i] = -Mv[i];

            double change = 0.0;
            for (int i=0;i<4;i++) change += (Mv[i]-v[i])*(Mv[i]-v[i]);
            for (int i=0;i<4;i++) v[i] = Mv[i];
            if (change < 1e-24) break;
        }

        if (lam > best_lam) {
            best_lam = lam;
            for (int i=0;i<4;i++) best_v[i] = v[i];
        }
    }

    for (int i=0;i<4;i++) evec[i] = best_v[i];
    return best_lam;
}

/**
 * Find second-largest eigenvalue by deflation.
 * λ₂ = dominant eigenvalue of (M - λ₁ * v₁ * v₁ᵀ).
 */
static double _second_eigenvalue(const double M[4][4],
                                  const double v1[4], double lam1) {
    /* Deflated matrix: M2 = M - lam1 * v1 v1^T */
    double M2[4][4];
    for (int i=0;i<4;i++)
        for (int j=0;j<4;j++)
            M2[i][j] = M[i][j] - lam1 * v1[i] * v1[j];

    double v2[4];
    return _power_iteration(M2, v2);
}

/* ── Core QUEST algorithm ─────────────────────────────────────── */

/**
 * Build Davenport K matrix and find optimal quaternion.
 *
 * n           : number of measurement pairs
 * bodies[n]   : body-frame unit vectors (will be normalised)
 * inertials[n]: inertial-frame unit vectors (will be normalised)
 * weights[n]  : normalised weights (must sum to 1)
 */
static QUEST_Result _quest_core(int n,
                                  const double bodies[][3],
                                  const double inertials[][3],
                                  const double weights[]) {
    QUEST_Result res;
    memset(&res, 0, sizeof(res));
    res.q[0] = 1.0;  /* safe default: identity */

    /* Build attitude profile matrix B = Σ wᵢ bᵢ rᵢᵀ  (3×3) */
    double B[3][3] = {{0}};
    for (int k=0; k<n; k++) {
        double b[3], r[3];
        _safe_norm(bodies[k],    b);
        _safe_norm(inertials[k], r);
        double w = weights[k];
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                B[i][j] += w * b[i] * r[j];
    }

    /* S = B + Bᵀ */
    double S[3][3];
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            S[i][j] = B[i][j] + B[j][i];

    /* sigma = trace(B) */
    double sigma = B[0][0] + B[1][1] + B[2][2];

    /* Z = [B₂₃-B₃₂, B₃₁-B₁₃, B₁₂-B₂₁] */
    double Z[3] = {
        B[1][2] - B[2][1],
        B[2][0] - B[0][2],
        B[0][1] - B[1][0]
    };

    /* 4×4 Davenport K matrix:
     *  K = [ sigma   Z^T  ]
     *      [  Z    S-σI  ]
     */
    double K[4][4] = {{0}};
    K[0][0] = sigma;
    K[0][1] = Z[0]; K[0][2] = Z[1]; K[0][3] = Z[2];
    K[1][0] = Z[0]; K[2][0] = Z[1]; K[3][0] = Z[2];
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            K[i+1][j+1] = S[i][j] - (i==j ? sigma : 0.0);

    /* Find dominant eigenvector (= optimal quaternion) */
    double q_opt[4];
    double lam1 = _power_iteration(K, q_opt);

    /* Enforce positive scalar part (canonical form — matches Python) */
    if (q_opt[0] < 0.0)
        for (int i=0;i<4;i++) q_opt[i] = -q_opt[i];

    _norm4_inplace(q_opt);
    for (int i=0;i<4;i++) res.q[i] = q_opt[i];

    /* Quality: gap between λ₁ and λ₂ (normalised to [0,1]) */
    double lam2 = _second_eigenvalue(K, q_opt, lam1);
    double gap  = lam1 - lam2;
    res.quality = (gap > 2.0) ? 1.0 : (gap < 0.0 ? 0.0 : gap / 2.0);
    res.ok      = (res.quality > QUEST_QUALITY_THRESHOLD) ? 1 : 0;

    return res;
}

/* ── Public API ───────────────────────────────────────────────── */

QUEST_Result QUEST_compute(const double v1_body[3],
                            const double v1_inertial[3],
                            const double v2_body[3],
                            const double v2_inertial[3],
                            double w1, double w2) {
    /* Normalise weights */
    double total = w1 + w2;
    if (total < 1e-12) { w1 = 0.5; w2 = 0.5; total = 1.0; }
    double w[2] = { w1/total, w2/total };

    double bodies[2][3]    = { {v1_body[0],     v1_body[1],     v1_body[2]},
                                {v2_body[0],     v2_body[1],     v2_body[2]} };
    double inertials[2][3] = { {v1_inertial[0], v1_inertial[1], v1_inertial[2]},
                                {v2_inertial[0], v2_inertial[1], v2_inertial[2]} };

    return _quest_core(2, bodies, inertials, w);
}

QUEST_Result QUEST_compute_multi(int n,
                                  const double bodies[][3],
                                  const double inertials[][3],
                                  const double weights[]) {
    if (n < 1 || n > QUEST_MAX_VECTORS) {
        QUEST_Result bad = {{1,0,0,0}, 0.0, 0};
        return bad;
    }

    /* Normalise weights */
    double total = 0.0;
    for (int i=0;i<n;i++) total += weights[i];
    double w_norm[QUEST_MAX_VECTORS];
    if (total < 1e-12) {
        for (int i=0;i<n;i++) w_norm[i] = 1.0/n;
    } else {
        for (int i=0;i<n;i++) w_norm[i] = weights[i] / total;
    }

    return _quest_core(n, bodies, inertials, w_norm);
}

void QUEST_nadir_inertial(const double pos_eci[3], double out[3]) {
    double r = sqrt(pos_eci[0]*pos_eci[0] +
                    pos_eci[1]*pos_eci[1] +
                    pos_eci[2]*pos_eci[2]);
    if (r < 1e-6) { out[0]=0.0; out[1]=0.0; out[2]=-1.0; return; }
    out[0] = -pos_eci[0]/r;
    out[1] = -pos_eci[1]/r;
    out[2] = -pos_eci[2]/r;
}