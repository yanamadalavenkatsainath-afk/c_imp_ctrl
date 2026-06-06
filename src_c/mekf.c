/**
 * mekf.c — MEKF implementation
 * =============================
 * Port of mekf.py — Markley & Crassidis §7.3
 *
 * PATCH P4 — Q dt-scaling and Joseph-form PD floor:
 *
 *   Issue 1 — Q scaling:
 *     Python mekf.py stores Q as a continuous-time PSD and injects it
 *     as  P += Q  every predict step (i.e. it already treats Q as a
 *     discrete per-step noise).  The original Python values are:
 *       Q_att  = 5e-8  rad²/step   (ARW²)
 *       Q_bias = 1e-12 rad²/s²/step (RRW²)
 *     The C code stored the same raw values but the cross-terms
 *       Phi[0:3,3:6] = -I*dt
 *     mean the Phi Q Phi^T product injects  dt² * Q_bias  into the
 *     attitude rows — a factor of dt² smaller than the Python path
 *     which uses P += Q directly.
 *
 *     Fix: store Q as the already-discrete covariance increment
 *     (matching the Python __init__), and compute
 *       P_new = Phi P Phi^T + Q_d
 *     where Q_d = Q (constant, pre-scaled at init).
 *     This is the "additive process noise" form used in Python.
 *
 *   Issue 2 — positive-definite floor after Joseph form:
 *     After  P = (I-KH) P (I-KH)^T + K R K^T  floating-point
 *     rounding can make tiny diagonal elements negative.
 *     Added a floor: P[i][i] = max(P[i][i], 1e-12).
 *     Matches Python mekf.py lines:
 *       eigvals = np.linalg.eigvalsh(self.P)
 *       if np.any(eigvals < 0):
 *           self.P += (-np.min(eigvals) + 1e-12) * np.eye(6)
 *     The C approximation (diagonal floor) is conservative but safe
 *     and avoids an eigensolver in flight code.
 */

#include "mekf.h"
#include <string.h>
#include <math.h>

/* ── Quaternion helpers ───────────────────────────────────────── */

static void quat_normalize(MEKF_FLOAT q[4]) {
    MEKF_FLOAT n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n < 1e-12f) return;   /* P4: 1e-12 epsilon, not 1e-10 */
    q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
}

static void quat_multiply(const MEKF_FLOAT a[4], const MEKF_FLOAT b[4],
                           MEKF_FLOAT out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void rot_matrix(const MEKF_FLOAT q[4], MEKF_FLOAT R[3][3]) {
    MEKF_FLOAT w=q[0],x=q[1],y=q[2],z=q[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z);   R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x);   R[2][2]=1-2*(x*x+y*y);
}

static int inv3(const MEKF_FLOAT A[3][3], MEKF_FLOAT Ainv[3][3]) {
    MEKF_FLOAT det =
        A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
       -A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
       +A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabsf(det) < 1e-30f) return -1;
    MEKF_FLOAT inv = 1.0f/det;
    Ainv[0][0]= inv*(A[1][1]*A[2][2]-A[1][2]*A[2][1]);
    Ainv[0][1]=-inv*(A[0][1]*A[2][2]-A[0][2]*A[2][1]);
    Ainv[0][2]= inv*(A[0][1]*A[1][2]-A[0][2]*A[1][1]);
    Ainv[1][0]=-inv*(A[1][0]*A[2][2]-A[1][2]*A[2][0]);
    Ainv[1][1]= inv*(A[0][0]*A[2][2]-A[0][2]*A[2][0]);
    Ainv[1][2]=-inv*(A[0][0]*A[1][2]-A[0][2]*A[1][0]);
    Ainv[2][0]= inv*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    Ainv[2][1]=-inv*(A[0][0]*A[2][1]-A[0][1]*A[2][0]);
    Ainv[2][2]= inv*(A[0][0]*A[1][1]-A[0][1]*A[1][0]);
    return 0;
}

/* ── CMSIS-DSP vs SIL matrix wrappers ────────────────────────── */

#ifndef MEKF_NO_CMSIS

static void mat_mul(const MEKF_FLOAT *A, const MEKF_FLOAT *B, MEKF_FLOAT *C,
                    int r, int k, int c) {
    arm_matrix_instance_f32 mA = {(uint16_t)r, (uint16_t)k, (MEKF_FLOAT*)A};
    arm_matrix_instance_f32 mB = {(uint16_t)k, (uint16_t)c, (MEKF_FLOAT*)B};
    arm_matrix_instance_f32 mC = {(uint16_t)r, (uint16_t)c, C};
    arm_mat_mult_f32(&mA, &mB, &mC);
}

static void mat_add(const MEKF_FLOAT *A, const MEKF_FLOAT *B, MEKF_FLOAT *C,
                    int r, int c) {
    arm_matrix_instance_f32 mA = {(uint16_t)r, (uint16_t)c, (MEKF_FLOAT*)A};
    arm_matrix_instance_f32 mB = {(uint16_t)r, (uint16_t)c, (MEKF_FLOAT*)B};
    arm_matrix_instance_f32 mC = {(uint16_t)r, (uint16_t)c, C};
    arm_mat_add_f32(&mA, &mB, &mC);
}

static void mat_trans(const MEKF_FLOAT *A, MEKF_FLOAT *AT, int r, int c) {
    arm_matrix_instance_f32 mA  = {(uint16_t)r, (uint16_t)c, (MEKF_FLOAT*)A};
    arm_matrix_instance_f32 mAT = {(uint16_t)c, (uint16_t)r, AT};
    arm_mat_trans_f32(&mA, &mAT);
}

#else

static void mat_mul(const MEKF_FLOAT *A, const MEKF_FLOAT *B, MEKF_FLOAT *C,
                    int r, int k, int c) {
    for (int i=0;i<r;i++)
        for (int j=0;j<c;j++) {
            MEKF_FLOAT s=0;
            for (int l=0;l<k;l++) s+=A[i*k+l]*B[l*c+j];
            C[i*c+j]=s;
        }
}

static void mat_add(const MEKF_FLOAT *A, const MEKF_FLOAT *B, MEKF_FLOAT *C,
                    int r, int c) {
    for (int i=0;i<r*c;i++) C[i]=A[i]+B[i];
}

static void mat_trans(const MEKF_FLOAT *A, MEKF_FLOAT *AT, int r, int c) {
    for (int i=0;i<r;i++)
        for (int j=0;j<c;j++)
            AT[j*r+i]=A[i*c+j];
}

#endif /* MEKF_NO_CMSIS */

/* ── Symmetrize + positive-definite floor 6×6 ────────────────── */
/*
 * P4: floor added here — called after every predict and update.
 * Prevents tiny negative diagonals from accumulating across ticks.
 */
static void sym6_pd(MEKF_FLOAT *P) {
    /* Symmetrize */
    for (int i=0;i<6;i++)
        for (int j=i+1;j<6;j++) {
            MEKF_FLOAT avg = 0.5f*(P[i*6+j]+P[j*6+i]);
            P[i*6+j] = avg;
            P[j*6+i] = avg;
        }
    /* PD floor on diagonal */
    for (int i=0;i<6;i++)
        if (P[i*6+i] < 1e-12f) P[i*6+i] = 1e-12f;
}

/* ── Public API ───────────────────────────────────────────────── */

void MEKF_init(MEKF_State *s, float dt_s) {
    memset(s, 0, sizeof(MEKF_State));
    s->dt   = (MEKF_FLOAT)dt_s;
    s->q[0] = 1.0f;

    /*
     * P — initial covariance (matches Python MEKF.__init__() exactly).
     * Attitude rows (0-2): (0.1°)² = (np.radians(0.1))² ≈ 3.046e-6 rad²
     * Bias rows    (3-5): (0.5 deg/hr)² = (np.radians(0.5)/3600)² ≈ 5.876e-12 rad²/s²
     */
    for (int i=0;i<6;i++) s->P[i][i] = 3.046e-6f;
    s->P[3][3]=5.876e-12f; s->P[4][4]=5.876e-12f; s->P[5][5]=5.876e-12f;

    /*
     * P4 FIX — Q discrete-time scaling.
     *
     * Python mekf.py uses:
     *   self.Q = np.diag([5e-8, 5e-8, 5e-8, 1e-12, 1e-12, 1e-12])
     * and applies  self.P = Phi P Phi^T + self.Q  each predict step.
     * Q is therefore already a per-step discrete covariance increment.
     *
     * The C MEKF_predict() replicates Phi P Phi^T exactly, so Q_d must
     * store the SAME per-step values as the Python Q.
     *
     * Previous C code had Q_att=1e-6 and Q_bias=1e-7 — both wrong vs
     * Python.  Corrected values below restore numerical parity with the
     * golden model.
     *
     * Q_att  = 5e-8  rad²/step  — Sensonor STIM300 ARW²×dt
     * Q_bias = 1e-12 rad²/s²/step — RRW²×dt  (matches Python exactly)
     *
     * NOTE: Python-matched P_bias_init is intentionally very small, so
     * fast gyro-bias identification should not be expected from short
     * realtime smoke tests unless a sufficiently observable attitude
     * measurement set is available.  test_mekf.c uses a focused synthetic
     * vector-measurement case to verify the bias-state plumbing.
     */
    s->Q[0][0] = 5e-8f;  s->Q[1][1] = 5e-8f;  s->Q[2][2] = 5e-8f;
    s->Q[3][3] = 1e-12f; s->Q[4][4] = 1e-12f; s->Q[5][5] = 1e-12f;

    /*
     * R — sensor noise covariance (matches Python MEKF.__init__() exactly).
     * R_mag = 0.5  (GEO magnetometer: field ~100-200nT vs 30000nT LEO,
     *               σ_unit_vector ~ 0.7 rad → R = 0.5)
     * R_sun = 3e-6 (50kg servicer sun sensor: σ ~ 0.1° = 1.7e-3 rad → R = σ²)
     */
    s->R_mag[0][0]=0.5f; s->R_mag[1][1]=0.5f; s->R_mag[2][2]=0.5f;
    s->R_sun[0][0]=3e-6f; s->R_sun[1][1]=3e-6f; s->R_sun[2][2]=3e-6f;
}

void MEKF_predict(MEKF_State *s, const MEKF_FLOAT omega_m[3]) {
    /* Bias-corrected angular rate */
    MEKF_FLOAT omega[3] = {
        omega_m[0] - s->bias[0],
        omega_m[1] - s->bias[1],
        omega_m[2] - s->bias[2]
    };
    MEKF_FLOAT wx=omega[0], wy=omega[1], wz=omega[2];

    /* Omega matrix (4×4) for quaternion kinematics:
     * q_dot = 0.5 * Omega @ q  */
    MEKF_FLOAT Omega[4][4] = {
        { 0,  -wx, -wy, -wz},
        { wx,  0,   wz, -wy},
        { wy, -wz,  0,   wx},
        { wz,  wy, -wx,  0 }
    };

    /* q += 0.5 * dt * Omega @ q  (Euler step — consistent with Python) */
    MEKF_FLOAT Oq[4];
    mat_mul(&Omega[0][0], s->q, Oq, 4, 4, 1);
    for (int i=0;i<4;i++) s->q[i] += 0.5f*s->dt*Oq[i];
    quat_normalize(s->q);

    /*
     * Error-state STM: Phi = I + F*dt
     *   F[0:3, 3:6] = -I   (attitude driven by bias)
     *   All other off-diag blocks = 0
     * So Phi[i][i] = 1, Phi[0][3]=Phi[1][4]=Phi[2][5] = -dt.
     */
    MEKF_FLOAT Phi[6][6];
    memset(Phi, 0, sizeof(Phi));
    for (int i=0;i<6;i++) Phi[i][i] = 1.0f;
    Phi[0][3]=-s->dt; Phi[1][4]=-s->dt; Phi[2][5]=-s->dt;

    /*
     * P4 FIX: P = Phi @ P @ Phi^T + Q_d
     * Q_d is the discrete-time noise increment stored in s->Q.
     * This EXACTLY mirrors Python:
     *   Phi = np.eye(6) + F * self.dt
     *   self.P = Phi @ self.P @ Phi.T + self.Q
     */
    MEKF_FLOAT PhiP[6][6], PhiT[6][6], PhiPPhiT[6][6];
    mat_mul(&Phi[0][0], &s->P[0][0], &PhiP[0][0],     6, 6, 6);
    mat_trans(&Phi[0][0], &PhiT[0][0], 6, 6);
    mat_mul(&PhiP[0][0], &PhiT[0][0], &PhiPPhiT[0][0], 6, 6, 6);
    mat_add(&PhiPPhiT[0][0], &s->Q[0][0], &s->P[0][0], 6, 6);
    sym6_pd(&s->P[0][0]);   /* P4: symmetrize + PD floor */
}

void MEKF_update(MEKF_State *s,
                 const MEKF_FLOAT z_body[3],
                 const MEKF_FLOAT v_inertial[3],
                 const MEKF_FLOAT R[MEKF_NZ][MEKF_NZ]) {

    /* Normalise inputs — P4: epsilon 1e-12 */
    MEKF_FLOAT zb[3], vi[3];
    MEKF_FLOAT nz = sqrtf(z_body[0]*z_body[0]+z_body[1]*z_body[1]+z_body[2]*z_body[2]);
    MEKF_FLOAT nv = sqrtf(v_inertial[0]*v_inertial[0]+v_inertial[1]*v_inertial[1]+v_inertial[2]*v_inertial[2]);
    if (nz < 1e-12f || nv < 1e-12f) return;
    for (int i=0;i<3;i++) { zb[i]=z_body[i]/nz; vi[i]=v_inertial[i]/nv; }

    /* Predicted measurement: z_pred = Rb @ v_inertial */
    MEKF_FLOAT Rb[3][3];
    rot_matrix(s->q, Rb);
    MEKF_FLOAT z_pred[3];
    mat_mul(&Rb[0][0], vi, z_pred, 3, 3, 1);

    /* Skew-symmetric of z_pred for measurement Jacobian */
    MEKF_FLOAT vx=z_pred[0], vy=z_pred[1], vz_=z_pred[2];
    MEKF_FLOAT skew[3][3] = {
        { 0,   -vz_,  vy },
        { vz_,  0,   -vx },
        {-vy,   vx,   0  }
    };

    /* H (3×6): H[:,0:3] = -skew, H[:,3:6] = 0
     * Markley & Crassidis Eq 7.39 */
    MEKF_FLOAT H[3][6];
    memset(H, 0, sizeof(H));
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            H[i][j] = -skew[i][j];

    /* Innovation */
    MEKF_FLOAT y[3] = {zb[0]-z_pred[0], zb[1]-z_pred[1], zb[2]-z_pred[2]};

    /* S = H P H^T + R */
    MEKF_FLOAT HP[3][6], HT[6][3], HPHT[3][3], S[3][3];
    mat_mul(&H[0][0],  &s->P[0][0], &HP[0][0],   3, 6, 6);
    mat_trans(&H[0][0], &HT[0][0], 3, 6);
    mat_mul(&HP[0][0], &HT[0][0],  &HPHT[0][0],  3, 6, 3);
    mat_add(&HPHT[0][0], &R[0][0], &S[0][0], 3, 3);

    /* Mahalanobis gate */
    MEKF_FLOAT S_inv[3][3];
    if (inv3(S, S_inv) != 0) return;
    MEKF_FLOAT Si_y[3];
    mat_mul(&S_inv[0][0], y, Si_y, 3, 3, 1);
    MEKF_FLOAT mahal = y[0]*Si_y[0]+y[1]*Si_y[1]+y[2]*Si_y[2];
    if (mahal > 25.0f) return;   /* 5-sigma gate — matches Python */

    /* K = P H^T S^-1 */
    MEKF_FLOAT PHT[6][3], K[6][3];
    mat_mul(&s->P[0][0], &HT[0][0], &PHT[0][0], 6, 6, 3);
    mat_mul(&PHT[0][0], &S_inv[0][0], &K[0][0],  6, 3, 3);

    /* dx = K @ y */
    MEKF_FLOAT dx[6];
    mat_mul(&K[0][0], y, dx, 6, 3, 1);

    /*
     * P4 FIX — Joseph-form covariance update (numerically stable):
     *   IKH = I - K H
     *   P   = IKH P IKH^T + K R K^T
     *
     * Matches Python mekf.py:
     *   IKH    = np.eye(6) - K @ H
     *   self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
     * Joseph form guarantees P remains symmetric positive semi-definite
     * even when the Kalman gain is not exactly optimal.
     */
    MEKF_FLOAT KH[6][6], IKH[6][6];
    mat_mul(&K[0][0], &H[0][0], &KH[0][0], 6, 3, 6);
    for (int i=0;i<6;i++)
        for (int j=0;j<6;j++)
            IKH[i][j] = (i==j ? 1.0f : 0.0f) - KH[i][j];

    MEKF_FLOAT IKHP[6][6], IKHT[6][6], IKHPIKHT[6][6];
    mat_mul(&IKH[0][0], &s->P[0][0], &IKHP[0][0],      6, 6, 6);
    mat_trans(&IKH[0][0], &IKHT[0][0], 6, 6);
    mat_mul(&IKHP[0][0], &IKHT[0][0], &IKHPIKHT[0][0], 6, 6, 6);

    MEKF_FLOAT KT[3][6], KR[6][3], KRKT[6][6];
    mat_trans(&K[0][0], &KT[0][0], 6, 3);
    mat_mul(&K[0][0], &R[0][0], &KR[0][0],  6, 3, 3);
    mat_mul(&KR[0][0], &KT[0][0], &KRKT[0][0], 6, 3, 6);

    mat_add(&IKHPIKHT[0][0], &KRKT[0][0], &s->P[0][0], 6, 6);
    sym6_pd(&s->P[0][0]);   /* P4: symmetrize + PD floor */

    /* Quaternion reset: dq = [1, 0.5*dtheta], q = normalize(dq ⊗ q) */
    MEKF_FLOAT dq[4] = {1.0f, 0.5f*dx[0], 0.5f*dx[1], 0.5f*dx[2]};
    MEKF_FLOAT q_new[4];
    quat_multiply(dq, s->q, q_new);
    quat_normalize(q_new);
    for (int i=0;i<4;i++) s->q[i]=q_new[i];

    /* Bias update */
    s->bias[0]+=dx[3]; s->bias[1]+=dx[4]; s->bias[2]+=dx[5];
}

void MEKF_update_star_tracker(MEKF_State *s,
                              const MEKF_FLOAT q_meas_in[MEKF_NQ],
                              const MEKF_FLOAT R_st[MEKF_NZ][MEKF_NZ]) {
    if (!s || !q_meas_in || !R_st) return;

    MEKF_FLOAT q_meas[4] = {
        q_meas_in[0], q_meas_in[1], q_meas_in[2], q_meas_in[3]
    };
    quat_normalize(q_meas);

    /* Sign consistency: choose the hemisphere closest to current estimate. */
    MEKF_FLOAT dot = q_meas[0]*s->q[0] + q_meas[1]*s->q[1] +
                     q_meas[2]*s->q[2] + q_meas[3]*s->q[3];
    if (dot < 0.0f) {
        for (int i=0;i<4;i++) q_meas[i] = -q_meas[i];
    }

    MEKF_FLOAT q_hat_conj[4] = {s->q[0], -s->q[1], -s->q[2], -s->q[3]};
    MEKF_FLOAT dq_err[4];
    quat_multiply(q_meas, q_hat_conj, dq_err);
    if (dq_err[0] < 0.0f) {
        for (int i=0;i<4;i++) dq_err[i] = -dq_err[i];
    }

    MEKF_FLOAT y[3] = {
        2.0f*dq_err[1],
        2.0f*dq_err[2],
        2.0f*dq_err[3]
    };

    MEKF_FLOAT H[3][6];
    memset(H, 0, sizeof(H));
    H[0][0] = 1.0f; H[1][1] = 1.0f; H[2][2] = 1.0f;

    MEKF_FLOAT HP[3][6], HT[6][3], HPHT[3][3], S[3][3];
    mat_mul(&H[0][0],  &s->P[0][0], &HP[0][0],   3, 6, 6);
    mat_trans(&H[0][0], &HT[0][0], 3, 6);
    mat_mul(&HP[0][0], &HT[0][0],  &HPHT[0][0],  3, 6, 3);
    mat_add(&HPHT[0][0], &R_st[0][0], &S[0][0], 3, 3);

    MEKF_FLOAT S_inv[3][3];
    if (inv3(S, S_inv) != 0) return;
    MEKF_FLOAT Si_y[3];
    mat_mul(&S_inv[0][0], y, Si_y, 3, 3, 1);
    MEKF_FLOAT mahal = y[0]*Si_y[0]+y[1]*Si_y[1]+y[2]*Si_y[2];
    if (mahal > 25.0f) return;

    MEKF_FLOAT PHT[6][3], K[6][3];
    mat_mul(&s->P[0][0], &HT[0][0], &PHT[0][0], 6, 6, 3);
    mat_mul(&PHT[0][0], &S_inv[0][0], &K[0][0],  6, 3, 3);

    MEKF_FLOAT dx[6];
    mat_mul(&K[0][0], y, dx, 6, 3, 1);

    MEKF_FLOAT KH[6][6], IKH[6][6];
    mat_mul(&K[0][0], &H[0][0], &KH[0][0], 6, 3, 6);
    for (int i=0;i<6;i++)
        for (int j=0;j<6;j++)
            IKH[i][j] = (i==j ? 1.0f : 0.0f) - KH[i][j];

    MEKF_FLOAT IKHP[6][6], IKHT[6][6], IKHPIKHT[6][6];
    mat_mul(&IKH[0][0], &s->P[0][0], &IKHP[0][0],      6, 6, 6);
    mat_trans(&IKH[0][0], &IKHT[0][0], 6, 6);
    mat_mul(&IKHP[0][0], &IKHT[0][0], &IKHPIKHT[0][0], 6, 6, 6);

    MEKF_FLOAT KT[3][6], KR[6][3], KRKT[6][6];
    mat_trans(&K[0][0], &KT[0][0], 6, 3);
    mat_mul(&K[0][0], &R_st[0][0], &KR[0][0],  6, 3, 3);
    mat_mul(&KR[0][0], &KT[0][0], &KRKT[0][0], 6, 3, 6);

    mat_add(&IKHPIKHT[0][0], &KRKT[0][0], &s->P[0][0], 6, 6);
    sym6_pd(&s->P[0][0]);

    MEKF_FLOAT dq_corr[4] = {1.0f, 0.5f*dx[0], 0.5f*dx[1], 0.5f*dx[2]};
    MEKF_FLOAT q_new[4];
    quat_multiply(dq_corr, s->q, q_new);
    quat_normalize(q_new);
    for (int i=0;i<4;i++) s->q[i]=q_new[i];

    s->bias[0]+=dx[3]; s->bias[1]+=dx[4]; s->bias[2]+=dx[5];
}
