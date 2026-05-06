/**
 * quest.h — QUEST Attitude Estimator
 * ====================================
 * Direct port of quest.py — Shuster & Oh (1981).
 *
 * Solves Wahba's problem:  min Σ wᵢ |bᵢ - A·rᵢ|²
 * via the Davenport K-matrix eigendecomposition.
 *
 * Advantages over TRIAD:
 *   - Uses all N measurements simultaneously (optimal L2 sense)
 *   - Never hard-fails on degenerate geometry — degrades gracefully
 *   - No sign ambiguity — eigenvector method is unambiguous
 *   - Quality metric (eigenvalue gap) tells you how reliable the result is
 *
 * Convention: q = [w, x, y, z]  scalar-first, same as mekf.h.
 *
 * Reference:
 *   Shuster & Oh (1981), JGCD 4(1), pp. 70–77.
 *   Markley & Crassidis, FSADC §5.3.
 */

#ifndef QUEST_H
#define QUEST_H

#include <math.h>
#include <string.h>

#define QUEST_MAX_VECTORS  8   /* max measurement vectors per call */

/* ── Result struct ────────────────────────────────────────────── */
typedef struct {
    double q[4];        /* quaternion [w, x, y, z]                     */
    double quality;     /* eigenvalue gap ∈ [0,1] — higher = better    */
    int    ok;          /* 1 if quality > QUEST_QUALITY_THRESHOLD       */
} QUEST_Result;

#define QUEST_QUALITY_THRESHOLD  0.01   /* below this = degenerate geometry */

/* ── API ──────────────────────────────────────────────────────── */

/**
 * QUEST_compute — 2-vector call, drop-in for TRIAD.
 *
 * Mirrors Python quest.compute(v1_body, v1_inertial, v2_body, v2_inertial,
 *                               w1, w2).
 *
 * v1_body[3]     : primary vector in body frame   (e.g. magnetometer)
 * v1_inertial[3] : primary vector in ECI
 * v2_body[3]     : secondary vector in body frame (e.g. sun sensor)
 * v2_inertial[3] : secondary vector in ECI
 * w1, w2         : relative weights (will be normalised to sum=1)
 *                  Pass 0.9, 0.1 for mag-dominant (GEO default).
 *
 * Returns QUEST_Result with .q, .quality, .ok.
 */
QUEST_Result QUEST_compute(const double v1_body[3],
                            const double v1_inertial[3],
                            const double v2_body[3],
                            const double v2_inertial[3],
                            double w1, double w2);

/**
 * QUEST_compute_multi — N-vector call (N ≤ QUEST_MAX_VECTORS).
 *
 * Mirrors Python quest.compute_multi().
 *
 * n                : number of vector pairs
 * bodies[n][3]     : body-frame unit vectors
 * inertials[n][3]  : inertial-frame unit vectors
 * weights[n]       : relative weights (normalised internally)
 *
 * Returns QUEST_Result.
 */
QUEST_Result QUEST_compute_multi(int n,
                                  const double bodies[][3],
                                  const double inertials[][3],
                                  const double weights[]);

/**
 * QUEST_nadir_inertial — compute nadir unit vector in ECI.
 * nadir = -r_hat   (pointing from spacecraft toward Earth).
 *
 * pos_eci[3] : ECI position [any unit — only direction used]
 * out[3]     : unit nadir vector in ECI
 */
void QUEST_nadir_inertial(const double pos_eci[3], double out[3]);

#endif /* QUEST_H */