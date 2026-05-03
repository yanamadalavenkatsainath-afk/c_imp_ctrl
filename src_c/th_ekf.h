/**
 * th_ekf.h — Tschauner-Hempel EKF for GEO Relative Navigation
 * =============================================================
 * Direct port of th_ekf.py (updated) for SIL verification.
 *
 * State:   x[6] = [δx, δy, δz, δẋ, δẏ, δż]  in LVLH [m, m/s]
 *
 * Measurement models
 * ------------------
 * Phase 4 (ranging sensor):
 *   z[3] = [range_m, azimuth_rad, elevation_rad]  → THEKF_update()
 *
 * Phase 5 (camera sensor):
 *   z[3] = [δx, δy, δz] relative position in LVLH [m]
 *   H = [I₃ | 0₃]  linear — exact, no Jacobian → THEKF_update_position()
 *
 * Doppler channel (radial velocity only):
 *   z_scalar = r̂ · δv   [m/s]   → THEKF_update_velocity_doppler()
 *   H = [0, 0, 0, r̂ₓ, r̂ᵧ, r̂_z]  (1×6)
 *   Velocity-only Kalman gain (K[0:3] zeroed) prevents cross-term explosion.
 *
 * Reference:
 *   Yamanaka & Ankersen (2002), JGCD 25(1).
 *   Curtis (2014), "Orbital Mechanics for Engineering Students", §7.3.
 */

#ifndef TH_EKF_H
#define TH_EKF_H

#include <math.h>   /* sqrt */

#define THEKF_NX   6   /* state dim        */
#define THEKF_NZ   3   /* measurement dim  */

/* P ceiling: sigma_pos <= 50 m, sigma_vel <= 1 m/s (matches Python). */
#define THEKF_P_CEIL_POS   (50.0  * 50.0)   /* m²   */
#define THEKF_P_CEIL_VEL   ( 1.0  *  1.0)   /* m²/s² */

/* Absolute innovation gate for position updates — rejects camera spikes. */
#define THEKF_INNOV_POS_MAX_M   10.0   /* m */

/* Doppler gate: 5-sigma on scalar innovation. */
#define THEKF_DOPPLER_GATE_SIGMA2  25.0   /* (5σ)² */

typedef struct {
    /* Orbit parameters (set once at init, const during flight) */
    double a;       /* chief SMA [m]          */
    double e;       /* chief eccentricity      */
    double mu;      /* gravitational param     */
    double dt;      /* filter timestep [s]     */
    double n;       /* mean motion [rad/s]     */
    double p;       /* semi-latus rectum [m]   */
    double h_orb;   /* specific angular mom    */
    double eta;     /* sqrt(1-e²)              */

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
 * Mirrors Python initialise(). P0 may be NULL to keep current P.
 */
void THEKF_seed(THEKF_State *ekf,
                const double x0[THEKF_NX],
                const double P0[THEKF_NX][THEKF_NX],   /* NULL → keep */
                double nu0);

/* ── Core EKF ─────────────────────────────────────────────────── */

/**
 * THEKF_predict — propagate state and covariance one timestep.
 * Applies P ceiling (sigma_pos ≤ 50m, sigma_vel ≤ 1m/s).
 * accel_lvlh: control acceleration [m/s²], NULL → zeros.
 */
void THEKF_predict(THEKF_State *ekf, const double accel_lvlh[3]);

/**
 * THEKF_update — EKF measurement update (ranging sensor).
 * z_meas: [range_m, azimuth_rad, elevation_rad]
 * R_meas: 3×3 measurement noise covariance
 * gate_k: Mahalanobis gate (sigma), typically 5.0
 * Returns 1 if measurement accepted, 0 if rejected.
 */
int THEKF_update(THEKF_State *ekf,
                 const double z_meas[THEKF_NZ],
                 const double R_meas[THEKF_NZ][THEKF_NZ],
                 double gate_k);

/**
 * THEKF_update_position — linear camera position update.
 * Mirrors Python update_position().
 *
 * z_pos[3]: measured relative position [dx, dy, dz] in LVLH [m]
 *           (output of camera_sensor.measure())
 * R_pos[3][3]: 3×3 noise covariance from camera_sensor
 * gate_k: Mahalanobis gate (sigma). Default 5.0.
 *
 * Additional absolute gate: if ||innovation|| > THEKF_INNOV_POS_MAX_M
 * the measurement is rejected regardless of the Mahalanobis distance.
 * This guards against camera spikes that pass a large-P gate.
 *
 * H = [I₃ | 0₃] — linear, exact update, no Jacobian required.
 *
 * Returns 1 if accepted, 0 if rejected.
 */
int THEKF_update_position(THEKF_State *ekf,
                           const double z_pos[3],
                           const double R_pos[3][3],
                           double gate_k);

/**
 * THEKF_update_velocity_doppler — scalar Doppler radial-velocity update.
 * Mirrors Python update_velocity_doppler().
 *
 * v_radial_meas: scalar range-rate measurement [m/s]
 *                = dot(true_dv, r_hat_true) + noise(sigma_radial)
 * r_hat[3]:      EKF range-direction unit vector = ekf->x[0:3] / ||ekf->x[0:3]||
 *                Must NOT use the truth direction.
 * sigma_radial:  1-sigma Doppler noise [m/s]. Use 0.005 (VBS class).
 *
 * Design:
 *   H = [0, 0, 0, r̂ₓ, r̂ᵧ, r̂_z]   (1×6 — velocity rows only)
 *   K_vel = K with K[0:3] zeroed — velocity-only correction.
 *   Prevents the position-velocity cross-term explosion at close range.
 *   The CW STM propagates radial information to lateral states naturally.
 *
 * Returns 1 if update applied, 0 if gated out.
 */
int THEKF_update_velocity_doppler(THEKF_State *ekf,
                                   double v_radial_meas,
                                   const double r_hat[3],
                                   double sigma_radial);

/**
 * THEKF_inflate_process_noise — temporarily widen P for TERMINAL phase.
 * Mirrors Python inflate_process_noise().
 *
 * Adds scale * Q[0][0] to P[0:3, 0:3] diagonal.
 * Called at TERMINAL entry when filter is near singularity
 * (sub-metre range, degenerate camera geometry).
 *
 * scale: multiplier on Q diagonal. Default 10.0.
 */
void THEKF_inflate_process_noise(THEKF_State *ekf, double scale);

/* ── Accessors ─────────────────────────────────────────────────── */

static inline void THEKF_get_pos(const THEKF_State *ekf, double out[3]) {
    out[0] = ekf->x[0]; out[1] = ekf->x[1]; out[2] = ekf->x[2];
}

static inline void THEKF_get_vel(const THEKF_State *ekf, double out[3]) {
    out[0] = ekf->x[3]; out[1] = ekf->x[4]; out[2] = ekf->x[5];
}

static inline double THEKF_range(const THEKF_State *ekf) {
    return sqrt(ekf->x[0]*ekf->x[0] + ekf->x[1]*ekf->x[1] + ekf->x[2]*ekf->x[2]);
}

static inline void THEKF_get_pos_std(const THEKF_State *ekf, double out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = (ekf->P[i][i] > 0.0) ? sqrt(ekf->P[i][i]) : 0.0;
}

static inline void THEKF_get_vel_std(const THEKF_State *ekf, double out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = (ekf->P[i+3][i+3] > 0.0) ? sqrt(ekf->P[i+3][i+3]) : 0.0;
}

#endif /* TH_EKF_H */