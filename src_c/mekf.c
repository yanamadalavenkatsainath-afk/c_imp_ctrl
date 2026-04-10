/**
 * mekf.c — MEKF implementation
 * =============================
 * Port of mekf.py — Markley & Crassidis §7.3
 *
 * KEY DESIGN CHOICE:
 *   All matrix operations use CMSIS-DSP arm_mat_mult_f32 when compiled
 *   for ARM (STM32 etc.). On desktop SIL, #define MEKF_NO_CMSIS and
 *   linalg.h macros are used instead — same logic, different backend.
 *
 *   This is what makes the code "flight-ready": the CMSIS path is
 *   exactly what runs on real hardware. The SIL path verifies it.
 */

#include "mekf.h"
#include <string.h>
#include <math.h>

/* ── Quaternion helpers (no external deps) ────────────────────── */

static void quat_normalize(MEKF_FLOAT q[4]) {
    MEKF_FLOAT n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n < 1e-10f) return;
    q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
}

/* q_out = q_a * q_b  (Hamilton product, w-first convention) */
static void quat_multiply(const MEKF_FLOAT a[4], const MEKF_FLOAT b[4],
                           MEKF_FLOAT out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* Rotation matrix from quaternion [w,x,y,z] → 3×3 */
static void rot_matrix(const MEKF_FLOAT q[4], MEKF_FLOAT R[3][3]) {
    MEKF_FLOAT w=q[0],x=q[1],y=q[2],z=q[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z);   R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x);   R[2][2]=1-2*(x*x+y*y);
}

/* 3×3 inverse via cofactor (R is always 3×3 in this filter) */
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
/*
 * On ARM/STM32: arm_mat_mult_f32 uses SIMD via FPU — hardware-accelerated.
 * On desktop SIL (#define MEKF_NO_CMSIS): plain nested loops.
 * Same function signature either way — zero code change between SIL and flight.
 */

#ifndef MEKF_NO_CMSIS
/* ── CMSIS path: uses arm_mat_mult_f32 ─────────────────────── */

/* Multiply A(r×k) @ B(k×c) → C(r×c) using CMSIS-DSP */
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
/* ── SIL fallback path: plain C loops ─────────────────────────── */

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

/* ── Symmetrize 6×6 ───────────────────────────────────────────── */
static void sym6(MEKF_FLOAT *P) {   /* takes flat pointer, indexes as [i*6+j] */
    for (int i=0;i<6;i++)
        for (int j=i+1;j<6;j++) {
            MEKF_FLOAT avg=0.5f*(P[i*6+j]+P[j*6+i]);
            P[i*6+j]=avg; P[j*6+i]=avg;
        }
}

/* ── Public API ───────────────────────────────────────────────── */

void MEKF_init(MEKF_State *s, float dt_s) {
    memset(s, 0, sizeof(MEKF_State));
    s->dt   = (MEKF_FLOAT)dt_s;
    s->q[0] = 1.0f;   /* identity quaternion */

    /* P: ~5° attitude uncertainty, 1°/hr bias uncertainty */
    for (int i=0;i<6;i++) s->P[i][i] = 0.01f;
    MEKF_FLOAT bias_var = (MEKF_FLOAT)(1.0/3600.0*3.14159/180.0);
    bias_var *= bias_var;
    s->P[3][3]=bias_var; s->P[4][4]=bias_var; s->P[5][5]=bias_var;

    /* Q: gyro ARW + bias instability (matches Python) */
    s->Q[0][0]=1e-6f; s->Q[1][1]=1e-6f; s->Q[2][2]=1e-6f;
    s->Q[3][3]=1e-12f; s->Q[4][4]=1e-12f; s->Q[5][5]=1e-12f;

    /* R: sensor noise (1e-4 rad² — matches Python) */
    s->R_mag[0][0]=1e-4f; s->R_mag[1][1]=1e-4f; s->R_mag[2][2]=1e-4f;
    s->R_sun[0][0]=1e-4f; s->R_sun[1][1]=1e-4f; s->R_sun[2][2]=1e-4f;
}

void MEKF_predict(MEKF_State *s, const MEKF_FLOAT omega_m[3]) {
    /* Bias-corrected angular rate */
    MEKF_FLOAT omega[3] = {
        omega_m[0] - s->bias[0],
        omega_m[1] - s->bias[1],
        omega_m[2] - s->bias[2]
    };
    MEKF_FLOAT wx=omega[0], wy=omega[1], wz=omega[2];

    /* Omega matrix (4×4) for quaternion kinematics */
    MEKF_FLOAT Omega[4][4] = {
        { 0,  -wx, -wy, -wz},
        { wx,  0,   wz, -wy},
        { wy, -wz,  0,   wx},
        { wz,  wy, -wx,  0 }
    };

    /* q += 0.5 * dt * Omega @ q */
    MEKF_FLOAT Oq[4];
    mat_mul(&Omega[0][0], s->q, Oq, 4, 4, 1);
    for (int i=0;i<4;i++) s->q[i] += 0.5f*s->dt*Oq[i];
    quat_normalize(s->q);

    /* F = [[0, -I], [0, 0]]  →  Phi = I + F*dt */
    /* Phi[0:3,3:6] = -I*dt, rest = I */
    MEKF_FLOAT Phi[6][6];
    memset(Phi, 0, sizeof(Phi));
    for (int i=0;i<6;i++) Phi[i][i] = 1.0f;
    Phi[0][3]=-s->dt; Phi[1][4]=-s->dt; Phi[2][5]=-s->dt;

    /* P = Phi @ P @ Phi^T + Q — uses CMSIS arm_mat_mult_f32 */
    MEKF_FLOAT PhiP[6][6], PhiT[6][6], PhiPPhiT[6][6];
    mat_mul(&Phi[0][0], &s->P[0][0], &PhiP[0][0], 6, 6, 6);
    mat_trans(&Phi[0][0], &PhiT[0][0], 6, 6);
    mat_mul(&PhiP[0][0], &PhiT[0][0], &PhiPPhiT[0][0], 6, 6, 6);
    mat_add(&PhiPPhiT[0][0], &s->Q[0][0], &s->P[0][0], 6, 6);
    sym6(&s->P[0][0]);
}

void MEKF_update(MEKF_State *s,
                 const MEKF_FLOAT z_body[3],
                 const MEKF_FLOAT v_inertial[3],
                 const MEKF_FLOAT R[MEKF_NZ][MEKF_NZ]) {
    /* Normalise inputs */
    MEKF_FLOAT zb[3], vi[3];
    MEKF_FLOAT nz = sqrtf(z_body[0]*z_body[0]+z_body[1]*z_body[1]+z_body[2]*z_body[2]);
    MEKF_FLOAT nv = sqrtf(v_inertial[0]*v_inertial[0]+v_inertial[1]*v_inertial[1]+v_inertial[2]*v_inertial[2]);
    if (nz<1e-10f || nv<1e-10f) return;
    for (int i=0;i<3;i++) { zb[i]=z_body[i]/nz; vi[i]=v_inertial[i]/nv; }

    /* Predicted measurement: z_pred = Rb @ v_inertial */
    MEKF_FLOAT Rb[3][3];
    rot_matrix(s->q, Rb);
    MEKF_FLOAT z_pred[3];
    mat_mul(&Rb[0][0], vi, z_pred, 3, 3, 1);

    /* Skew-symmetric of z_pred */
    MEKF_FLOAT vx=z_pred[0], vy=z_pred[1], vz_=z_pred[2];
    MEKF_FLOAT skew[3][3] = {
        { 0,   -vz_,  vy },
        { vz_,  0,   -vx },
        {-vy,   vx,   0  }
    };

    /* H (3×6): H[:,0:3] = -skew, H[:,3:6] = 0 */
    MEKF_FLOAT H[3][6];
    memset(H, 0, sizeof(H));
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            H[i][j] = -skew[i][j];

    /* Innovation y = z_body - z_pred */
    MEKF_FLOAT y[3] = {zb[0]-z_pred[0], zb[1]-z_pred[1], zb[2]-z_pred[2]};

    /* S = H @ P @ H^T + R  (3×3) — CMSIS */
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
    if (mahal > 25.0f) return;   /* 5-sigma gate */

    /* K = P @ H^T @ S^-1  (6×3) — CMSIS */
    MEKF_FLOAT PHT[6][3], K[6][3];
    mat_mul(&s->P[0][0], &HT[0][0], &PHT[0][0], 6, 6, 3);
    mat_mul(&PHT[0][0], &S_inv[0][0], &K[0][0],  6, 3, 3);

    /* dx = K @ y (6×1) */
    MEKF_FLOAT dx[6];
    mat_mul(&K[0][0], y, dx, 6, 3, 1);

    /* P = (I - K@H) @ P @ (I-K@H)^T + K@R@K^T  (Joseph form) */
    MEKF_FLOAT KH[6][6], IKH[6][6], I6[6][6];
    memset(I6, 0, sizeof(I6));
    for (int i=0;i<6;i++) I6[i][i]=1.0f;
    mat_mul(&K[0][0], &H[0][0], &KH[0][0],  6, 3, 6);
    mat_add(&I6[0][0], &KH[0][0], &IKH[0][0], 6, 6);   /* IKH = I - KH? */
    /* Actually: IKH = I - KH */
    for (int i=0;i<36;i++) ((MEKF_FLOAT*)IKH)[i] = ((MEKF_FLOAT*)I6)[i] - ((MEKF_FLOAT*)KH)[i];

    MEKF_FLOAT IKHP[6][6], IKHT[6][6], IKHPIKHT[6][6];
    mat_mul(&IKH[0][0], &s->P[0][0], &IKHP[0][0],    6, 6, 6);
    mat_trans(&IKH[0][0], &IKHT[0][0], 6, 6);
    mat_mul(&IKHP[0][0], &IKHT[0][0], &IKHPIKHT[0][0], 6, 6, 6);

    MEKF_FLOAT KT[3][6], KR[6][3], KRKT[6][6];
    mat_trans(&K[0][0], &KT[0][0], 6, 3);
    mat_mul(&K[0][0], &R[0][0], &KR[0][0],  6, 3, 3);
    mat_mul(&KR[0][0], &KT[0][0], &KRKT[0][0], 6, 3, 6);

    mat_add(&IKHPIKHT[0][0], &KRKT[0][0], &s->P[0][0], 6, 6);
    sym6(&s->P[0][0]);

    /* Quaternion update: dq = [1, 0.5*dtheta], q = normalize(dq * q) */
    MEKF_FLOAT dq[4] = {1.0f, 0.5f*dx[0], 0.5f*dx[1], 0.5f*dx[2]};
    MEKF_FLOAT q_new[4];
    quat_multiply(dq, s->q, q_new);
    quat_normalize(q_new);
    for (int i=0;i<4;i++) s->q[i]=q_new[i];

    /* Bias update */
    s->bias[0]+=dx[3]; s->bias[1]+=dx[4]; s->bias[2]+=dx[5];
}