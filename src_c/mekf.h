/**
 * mekf.h — Multiplicative EKF for Spacecraft Attitude
 * =====================================================
 *
 * Port note: the vector-measurement MEKF path is ported here.  Python's
 * update_star_tracker() full-quaternion measurement update is not exposed
 * in this C API yet; add a dedicated MEKF_update_star_tracker() before
 * wiring a hardware star-tracker packet into the C flight loop.
 * Direct port of mekf.py — Markley & Crassidis §7.3
 *
 * State (error state, 6×1):
 *   dx[0:3]  = dtheta  — attitude error [rad]
 *   dx[3:6]  = dbias   — gyro bias error [rad/s]
 *
 * Nominal state:
 *   q[4]     — attitude quaternion [w, x, y, z]
 *   bias[3]  — gyro bias [rad/s]
 *
 * CMSIS-DSP usage:
 *   All matrix ops use arm_mat_mult_f32 / arm_mat_add_f32 etc.
 *   On desktop (non-ARM): CMSIS-DSP compiles via the CMSIS repo.
 *   On STM32: same code runs natively on FPU.
 *
 * Compile with CMSIS-DSP:
 *   git clone https://github.com/ARM-software/CMSIS-DSP
 *   gcc -I CMSIS-DSP/Include -DARM_MATH_CM4 mekf.c CMSIS-DSP/Source/...
 *
 * For SIL without real CMSIS, define MEKF_NO_CMSIS and linalg.h is used.
 */

#ifndef MEKF_H
#define MEKF_H

/* ── CMSIS-DSP or fallback ────────────────────────────────────── */
#ifndef MEKF_NO_CMSIS
  #include "arm_math.h"
  #define MEKF_FLOAT  float32_t
#else
  #include "linalg.h"
  /* float in SIL — keeps struct size identical to ARM target.
     Internal temporaries in mekf.c use double for precision. */
  #define MEKF_FLOAT  float
#endif

#define MEKF_NX   6   /* error state dimension  */
#define MEKF_NQ   4   /* quaternion dimension   */
#define MEKF_NZ   3   /* measurement dimension  */

/* ── State struct ─────────────────────────────────────────────── */
typedef struct {
    /* Nominal state */
    MEKF_FLOAT q[MEKF_NQ];        /* quaternion [w,x,y,z]     */
    MEKF_FLOAT bias[MEKF_NZ];     /* gyro bias  [rad/s]       */

    /* Error-state covariance (6×6) */
    MEKF_FLOAT P[MEKF_NX][MEKF_NX];

    /* Tuning (set at init, const during flight) */
    MEKF_FLOAT Q[MEKF_NX][MEKF_NX];   /* process noise  */
    MEKF_FLOAT R_mag[MEKF_NZ][MEKF_NZ]; /* mag noise    */
    MEKF_FLOAT R_sun[MEKF_NZ][MEKF_NZ]; /* sun noise    */

    MEKF_FLOAT dt;                /* timestep [s]             */
} MEKF_State;

/* ── API ──────────────────────────────────────────────────────── */

/**
 * MEKF_init — initialise state and noise matrices.
 * Mirrors Python __init__().
 */
void MEKF_init(MEKF_State *s, float dt_s);

/**
 * MEKF_predict — gyro propagation.
 * omega_m[3]: raw gyro measurement [rad/s]
 * Mirrors Python predict().
 */
void MEKF_predict(MEKF_State *s, const MEKF_FLOAT omega_m[3]);

/**
 * MEKF_update — vector measurement update (magnetometer or sun sensor).
 * z_body[3]    : measured unit vector in body frame
 * v_inertial[3]: reference unit vector in inertial frame
 * R[3][3]      : measurement noise covariance
 * Mirrors Python update_vector().
 */
void MEKF_update(MEKF_State *s,
                 const MEKF_FLOAT z_body[3],
                 const MEKF_FLOAT v_inertial[3],
                 const MEKF_FLOAT R[MEKF_NZ][MEKF_NZ]);

/* ── Convenience accessors ────────────────────────────────────── */
static inline void MEKF_get_q   (const MEKF_State *s, MEKF_FLOAT out[4]) {
    for (int i=0;i<4;i++) out[i]=s->q[i]; }
static inline void MEKF_get_bias(const MEKF_State *s, MEKF_FLOAT out[3]) {
    for (int i=0;i<3;i++) out[i]=s->bias[i]; }

#endif /* MEKF_H */
