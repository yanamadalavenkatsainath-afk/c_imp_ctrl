/**
 * th_ekf.h — Tschauner-Hempel EKF for GEO Relative Navigation
 * =============================================================
 * Direct port of th_ekf.py for SIL verification.
 *
 * State:   x[6] = [δx, δy, δz, δẋ, δẏ, δż]  in LVLH [m, m/s]
 * Measure: z[3] = [range_m, azimuth_rad, elevation_rad]
 *
 * Usage:
 *   THEKF_State ekf;
 *   THEKF_init(&ekf, A_CHIEF_M, E_CHIEF, MU_GEO, DT_S);
 *   THEKF_predict(&ekf, accel_lvlh);
 *   THEKF_update(&ekf, z, R_meas, gate_k);
 */

#ifndef TH_EKF_H

#include <math.h>   /* sqrt — needed for THEKF_range inline */
#define TH_EKF_H

#define THEKF_NX   6   /* state dim */
#define THEKF_NZ   3   /* measurement dim */

typedef struct {
    /* Orbit parameters (set once at init, const during flight) */
    double a;       /* chief SMA [m]          */
    double e;       /* chief eccentricity      */
    double mu;      /* gravitational param     */
    double dt;      /* filter timestep [s]     */
    double n;       /* mean motion [rad/s]     */
    double p;       /* semi-latus rectum [m]   */
    double h_orb;   /* specific angular mom    */
    double eta;     /* sqrt(1-e^2)             */

    /* Filter state */
    double x[THEKF_NX];                    /* state vector */
    double P[THEKF_NX][THEKF_NX];         /* covariance   */
    double Q[THEKF_NX][THEKF_NX];         /* process noise (fixed) */

    /* True anomaly tracking */
    double nu;      /* current true anomaly [rad] */
    double t_ekf;   /* internal clock [s]         */
} THEKF_State;

/* ── Lifecycle ───────────────────────────────────────────────── */

/**
 * THEKF_init — configure and zero the EKF.
 * Call once before any predict/update.
 */
void THEKF_init(THEKF_State *ekf,
                double a_chief_m,
                double e_chief,
                double mu,
                double dt_s,
                double q_pos,    /* position process noise PSD */
                double q_vel);   /* velocity process noise PSD */

/**
 * THEKF_seed — set state and covariance directly.
 * Mirrors Python initialise().
 */
void THEKF_seed(THEKF_State *ekf,
                const double x0[THEKF_NX],
                const double P0[THEKF_NX][THEKF_NX],   /* NULL → keep */
                double nu0);

/* ── Core EKF ─────────────────────────────────────────────────── */

/**
 * THEKF_predict — propagate state and covariance one timestep.
 * accel_lvlh: control acceleration [m/s²], NULL → zeros.
 */
void THEKF_predict(THEKF_State *ekf, const double accel_lvlh[3]);

/**
 * THEKF_update — EKF measurement update.
 * z_meas: [range_m, azimuth_rad, elevation_rad]
 * R_meas: 3×3 measurement noise covariance
 * gate_k: Mahalanobis gate (sigma), typically 5.0
 * Returns 1 if measurement accepted, 0 if rejected.
 */
int THEKF_update(THEKF_State *ekf,
                 const double z_meas[THEKF_NZ],
                 const double R_meas[THEKF_NZ][THEKF_NZ],
                 double gate_k);

/* ── Accessors ─────────────────────────────────────────────────── */

/** Copy position estimate [m] into out[3]. */
static inline void THEKF_get_pos(const THEKF_State *ekf, double out[3]) {
    out[0] = ekf->x[0]; out[1] = ekf->x[1]; out[2] = ekf->x[2];
}

/** Copy velocity estimate [m/s] into out[3]. */
static inline void THEKF_get_vel(const THEKF_State *ekf, double out[3]) {
    out[0] = ekf->x[3]; out[1] = ekf->x[4]; out[2] = ekf->x[5];
}

/** Range estimate [m]. */
static inline double THEKF_range(const THEKF_State *ekf) {
    return sqrt(ekf->x[0]*ekf->x[0] + ekf->x[1]*ekf->x[1] + ekf->x[2]*ekf->x[2]);
}

#endif /* TH_EKF_H */