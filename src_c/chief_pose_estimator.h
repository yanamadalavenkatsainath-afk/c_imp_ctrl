/**
 * chief_pose_estimator.h — Chief Pose EKF (6-state attitude + omega)
 * ===================================================================
 * Port of chief_pose_estimator.py — Markley & Crassidis Ch. 4.
 *
 * State: [dtheta(3), domega(3)] — error-state MEKF formulation.
 * Nominal state: q[4] (quaternion) + omega[3] (angular rate).
 *
 * Measurement: PnP-derived R_body2lvlh rotation matrix each camera frame,
 * converted to a rotation-vector innovation (small-angle).
 *
 * Advantages over frame-differencing (same as Python comments):
 *   - Continuous estimate between frames
 *   - Explicit covariance propagation
 *   - Mahalanobis gate rejects outlier PnP solutions
 *   - Smooth omega estimate — no window latency
 *
 * The EPnP orientation estimator (_estimate_orientation) is ported
 * faithfully from the Python using stack-allocated SVD (Jacobi method).
 *
 * No malloc. All state in caller-owned CPE_State struct.
 *
 * Reference:
 *   Markley & Crassidis, FSADC, Springer 2014, Ch. 4.
 *   Lepetit, Moreno-Noguer, Fua, IJCV 2009 (EPnP).
 *   Opromolla et al., IEEE TAES 2017.
 */

#ifndef CHIEF_POSE_ESTIMATOR_H
#define CHIEF_POSE_ESTIMATOR_H

#include <math.h>
#include <string.h>
#include <stdint.h>

/* ── Camera model parameters (set at init, matches CameraSensor) */
typedef struct {
    double f;           /* focal length [px]            */
    double cx, cy;      /* principal point [px]         */
    double W, H;        /* image size [px]              */
    double sigma_px;    /* pixel noise 1-sigma [px]     */
    double min_range;   /* minimum detectable range [m] */
    double max_range;   /* maximum detectable range [m] */

    /* 3U CubeSat model points in body frame [m], up to 8 corners */
    double model_pts[8][3];
    int    n_model_pts;
} CPE_CamParams;

/* ── EKF state ────────────────────────────────────────────────── */
#define CPE_NX  6    /* error-state dimension: [dtheta(3), domega(3)] */

typedef struct {
    /* Nominal state */
    double q[4];        /* quaternion [w,x,y,z] body→frame             */
    double omega[3];    /* angular rate estimate [rad/s]                */

    /* Error-state covariance (6×6) */
    double P[CPE_NX][CPE_NX];

    /* Tuning matrices */
    double Q[CPE_NX][CPE_NX];   /* process noise covariance (6×6)      */
    double R_meas[3][3];         /* PnP measurement noise covariance     */

    /* Filter metadata */
    double  dt;             /* timestep [s]                             */
    double  gate_k;         /* Mahalanobis gate (sigma)                 */
    int     update_count;   /* successful PnP updates so far            */
    int     valid;          /* 1 after update_count >= 10               */

    /* Last successful PnP rotation matrix (body → LVLH) */
    double  last_R_b2l[3][3];
    int     has_R_b2l;      /* 1 if last_R_b2l is valid                 */

    /* Camera params (copied at init) */
    CPE_CamParams cam;
} CPE_State;

/* ── Result from one update call ─────────────────────────────── */
typedef struct {
    double omega[3];   /* estimated angular rate [rad/s] */
    int    valid;      /* 1 if filter has converged      */
} CPE_Result;

/* ── API ──────────────────────────────────────────────────────── */

/**
 * CPE_init — initialise chief pose EKF.
 *
 * Mirrors Python ChiefPoseEstimator.__init__().
 *
 * s                   : state struct (caller-owned)
 * cam                 : camera parameters
 * dt                  : timestep [s]
 * sigma_omega_process : process noise on omega [rad/s/sqrt(s)]
 * sigma_pnp_deg       : PnP orientation noise 1-sigma [deg]
 * gate_k              : Mahalanobis gate (5.0 matches Python)
 */
void CPE_init(CPE_State *s,
              const CPE_CamParams *cam,
              double dt,
              double sigma_omega_process,
              double sigma_pnp_deg,
              double gate_k);

/**
 * CPE_update — one EKF predict + update step.
 *
 * Mirrors Python ChiefPoseEstimator.update().
 *
 * s            : EKF state (modified in place)
 * dr_lvlh[3]   : relative position of chief in LVLH [m]
 * q_chief[4]   : chief quaternion [w,x,y,z] from pose estimate
 *                (used to project model points into camera frame)
 * rng_seed     : pass -1 to use global rand; pass value for determinism
 *                (noise is injected via C rand() scaled to sigma_px)
 *
 * Returns CPE_Result with .omega and .valid.
 *
 * Range-dependent R gain scheduling (matches Python):
 *   range < 2m  : r_scale = 0.05
 *   2–5m        : r_scale = 0.05 + 0.25*(range-2)/3
 *   5–20m       : r_scale = 0.5
 *   > 20m       : r_scale = 1.0
 */
CPE_Result CPE_update(CPE_State *s,
                       const double dr_lvlh[3],
                       const double q_chief[4]);

/**
 * CPE_get_omega — get current omega estimate [rad/s].
 */
static inline void CPE_get_omega(const CPE_State *s, double out[3]) {
    out[0] = s->omega[0];
    out[1] = s->omega[1];
    out[2] = s->omega[2];
}

/**
 * CPE_get_q — get current quaternion estimate [w,x,y,z].
 */
static inline void CPE_get_q(const CPE_State *s, double out[4]) {
    out[0] = s->q[0]; out[1] = s->q[1];
    out[2] = s->q[2]; out[3] = s->q[3];
}

/**
 * CPE_get_R_body2lvlh — copy last PnP rotation matrix.
 * Returns 1 if valid, 0 if no PnP has succeeded yet.
 */
static inline int CPE_get_R_body2lvlh(const CPE_State *s, double R_out[3][3]) {
    if (!s->has_R_b2l) return 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R_out[i][j] = s->last_R_b2l[i][j];
    return 1;
}

/**
 * CPE_omega_uncertainty — RMS omega uncertainty [rad/s].
 * = sqrt(mean(diag(P)[3:6]))
 */
static inline double CPE_omega_uncertainty(const CPE_State *s) {
    double sum = s->P[3][3] + s->P[4][4] + s->P[5][5];
    return sqrt(sum / 3.0);
}

/**
 * CPE_default_cam_params — fill CPE_CamParams with 3U CubeSat defaults.
 * Matches CameraSensor.__init__() defaults in camera_sensor.py.
 */
void CPE_default_cam_params(CPE_CamParams *cam);

#endif /* CHIEF_POSE_ESTIMATOR_H */