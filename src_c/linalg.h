/**
 * linalg.h — Static matrix math for GNC flight code
 * ==================================================
 * Dimensions used in TH-EKF:
 *   State x     : 6×1
 *   Covariance P: 6×6
 *   STM Phi     : 6×6
 *   Meas H      : 3×6
 *   Meas noise R: 3×3
 *   Kalman gain K: 6×3
 *   Innovation S : 3×3
 *
 * Rules for flight code:
 *   - No malloc / free anywhere
 *   - All temporaries are stack-allocated
 *   - Row-major storage: A[r][c]
 */

#ifndef LINALG_H
#define LINALG_H

#include <math.h>
#include <string.h>   /* memset, memcpy */

/* ── Scalar helpers ───────────────────────────────────────────── */

static inline double vec3_norm(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static inline double vec6_dot(const double a[6], const double b[6]) {
    double s = 0.0;
    for (int i = 0; i < 6; i++) s += a[i]*b[i];
    return s;
}

/* ── Generic NxM matrix ops (N,M as template via macros) ──────── */

/* C = A @ B  where A is NxK, B is KxM, C is NxM */
#define MAT_MUL(C, A, B, N, K, M)                          \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++)                   \
            for (int _j = 0; _j < (M); _j++) {             \
                (C)[_i][_j] = 0.0;                         \
                for (int _k = 0; _k < (K); _k++)           \
                    (C)[_i][_j] += (A)[_i][_k]*(B)[_k][_j]; \
            }                                               \
    } while(0)

/* C = A + B  (NxM) */
#define MAT_ADD(C, A, B, N, M)                              \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++)                   \
            for (int _j = 0; _j < (M); _j++)               \
                (C)[_i][_j] = (A)[_i][_j] + (B)[_i][_j];  \
    } while(0)

/* C = A - B  (NxM) */
#define MAT_SUB(C, A, B, N, M)                              \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++)                   \
            for (int _j = 0; _j < (M); _j++)               \
                (C)[_i][_j] = (A)[_i][_j] - (B)[_i][_j];  \
    } while(0)

/* B = A^T  where A is NxM, B is MxN */
#define MAT_T(B, A, N, M)                                   \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++)                   \
            for (int _j = 0; _j < (M); _j++)               \
                (B)[_j][_i] = (A)[_i][_j];                 \
    } while(0)

/* C = A @ v  where A is NxM, v is M×1, c is N×1 */
#define MAT_VEC(c, A, v, N, M)                              \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++) {                 \
            (c)[_i] = 0.0;                                  \
            for (int _j = 0; _j < (M); _j++)               \
                (c)[_i] += (A)[_i][_j]*(v)[_j];            \
        }                                                   \
    } while(0)

/* Identity NxN */
#define MAT_EYE(A, N)                                       \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++)                   \
            for (int _j = 0; _j < (N); _j++)               \
                (A)[_i][_j] = (_i == _j) ? 1.0 : 0.0;     \
    } while(0)

/* Scale:  A *= s  (NxM) */
#define MAT_SCALE(A, s, N, M)                               \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++)                   \
            for (int _j = 0; _j < (M); _j++)               \
                (A)[_i][_j] *= (s);                        \
    } while(0)

/* Symmetrize: A = 0.5*(A + A^T)  (NxN) */
#define MAT_SYM(A, N)                                       \
    do {                                                    \
        for (int _i = 0; _i < (N); _i++)                   \
            for (int _j = _i+1; _j < (N); _j++) {          \
                double _avg = 0.5*((A)[_i][_j]+(A)[_j][_i]); \
                (A)[_i][_j] = _avg;                         \
                (A)[_j][_i] = _avg;                         \
            }                                               \
    } while(0)

/* ── 3×3 inverse (used for S^-1 in EKF update) ───────────────── */
static inline int mat3_inv(double Ainv[3][3], const double A[3][3]) {
    double det = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
               - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
               + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    if (det == 0.0) return -1;
    double inv = 1.0 / det;
    Ainv[0][0] =  inv*(A[1][1]*A[2][2] - A[1][2]*A[2][1]);
    Ainv[0][1] = -inv*(A[0][1]*A[2][2] - A[0][2]*A[2][1]);
    Ainv[0][2] =  inv*(A[0][1]*A[1][2] - A[0][2]*A[1][1]);
    Ainv[1][0] = -inv*(A[1][0]*A[2][2] - A[1][2]*A[2][0]);
    Ainv[1][1] =  inv*(A[0][0]*A[2][2] - A[0][2]*A[2][0]);
    Ainv[1][2] = -inv*(A[0][0]*A[1][2] - A[0][2]*A[1][0]);
    Ainv[2][0] =  inv*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    Ainv[2][1] = -inv*(A[0][0]*A[2][1] - A[0][1]*A[2][0]);
    Ainv[2][2] =  inv*(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
    return 0;
}

/* ── angle wrap to [-π, π] ────────────────────────────────────── */
static inline double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

#endif /* LINALG_H */