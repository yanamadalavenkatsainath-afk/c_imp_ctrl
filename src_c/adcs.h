/**
 * adcs.h — ADCS Actuator and Controller Library
 * ===============================================
 * Direct port of:
 *   bdot.py            → BDOT_compute()
 *   attitude_controller.py → ATTCTRL_compute()
 *   reaction_wheel.py  → RW_apply_torque()
 *   magnetorquer.py    → MTQ_compute_dipole() / MTQ_compute_torque()
 *
 * Convention: quaternion [w, x, y, z] scalar-first throughout.
 *
 * No malloc. All state is in caller-owned structs.
 */

#ifndef ADCS_H
#define ADCS_H

#include <math.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════
 * B-DOT DETUMBLE CONTROLLER
 * Port of BDotController — bdot.py
 * ══════════════════════════════════════════════════════════════════ */

typedef struct {
    double k_bdot;   /* gain [A·m²/T·s] — default 1e5 */
    double m_max;    /* dipole saturation [A·m²] — default 0.2 */
} BDot_State;

/**
 * BDOT_init — initialise with Python-matching defaults.
 */
static inline void BDOT_init(BDot_State *s) {
    s->k_bdot = 1e5;
    s->m_max  = 0.2;
}

/**
 * BDOT_compute — compute magnetorquer dipole command and resulting torque.
 *
 * B_body[3]    : magnetic field in body frame [T]
 * omega_body[3]: body angular rate [rad/s]
 * m_cmd_out[3] : commanded magnetic dipole [A·m²]
 * torque_out[3]: resulting torque on body [N·m]
 *
 * Physics:
 *   B_dot = -omega × B  (rotating body frame)
 *   m_cmd = -k_bdot * B_dot  (clipped to ±m_max)
 *   torque = m_cmd × B
 */
void BDOT_compute(const BDot_State *s,
                  const double B_body[3],
                  const double omega_body[3],
                  double m_cmd_out[3],
                  double torque_out[3]);

/* ══════════════════════════════════════════════════════════════════
 * REACTION WHEEL
 * Port of ReactionWheel — reaction_wheel.py
 * ══════════════════════════════════════════════════════════════════ */

typedef struct {
    double h[3];      /* stored angular momentum [N·m·s] */
    double h_max;     /* saturation limit [N·m·s] — default 0.05 */
} RW_State;

/**
 * RW_init — initialise with h=0 and Python-matching default h_max.
 */
static inline void RW_init(RW_State *s) {
    s->h[0] = s->h[1] = s->h[2] = 0.0;
    s->h_max = 0.05;
}

/**
 * RW_apply_torque — integrate wheel momentum by dt.
 *
 * torque_cmd[3]: commanded torque TO the wheel [N·m]
 * dt           : timestep [s]
 *
 * Mirrors Python: h += torque_cmd * dt, clipped to ±h_max.
 * Returns torque_cmd (unchanged — reaction on body handled externally).
 */
void RW_apply_torque(RW_State *s,
                     const double torque_cmd[3],
                     double dt);

/* ══════════════════════════════════════════════════════════════════
 * MAGNETORQUER (momentum dump)
 * Port of Magnetorquer — magnetorquer.py
 * ══════════════════════════════════════════════════════════════════ */

typedef struct {
    double m_max;   /* max dipole [A·m²] — default 0.4 */
} MTQ_State;

/**
 * MTQ_init — initialise with Python-matching default.
 */
static inline void MTQ_init(MTQ_State *s) {
    s->m_max = 0.4;
}

/**
 * MTQ_compute_dipole — momentum-dump dipole via cross-product law.
 *
 * Mirrors Python Magnetorquer.compute_dipole():
 *   m = -k_dump * (h × B) / |B|²    clipped to ±m_max
 *
 * h[3]       : wheel momentum vector [N·m·s]
 * B_body[3]  : magnetic field in body frame [T]
 * m_out[3]   : commanded dipole [A·m²]
 */
void MTQ_compute_dipole(const MTQ_State *s,
                         const double h[3],
                         const double B_body[3],
                         double m_out[3]);

/**
 * MTQ_compute_torque — torque from dipole in field.
 *
 * torque = m × B
 *
 * m[3]     : magnetic dipole [A·m²]
 * B_body[3]: magnetic field in body frame [T]
 * torque_out[3]: resulting torque [N·m]
 */
void MTQ_compute_torque(const double m[3],
                         const double B_body[3],
                         double torque_out[3]);

/* ══════════════════════════════════════════════════════════════════
 * ATTITUDE PD CONTROLLER
 * Port of AttitudeController — attitude_controller.py
 * ══════════════════════════════════════════════════════════════════ */

typedef struct {
    double Kp;   /* proportional gain — default 0.3 */
    double Kd;   /* derivative gain  — default 0.08 */
} AttCtrl_State;

/**
 * ATTCTRL_init — initialise with Python-matching defaults.
 */
static inline void ATTCTRL_init(AttCtrl_State *s) {
    s->Kp = 0.3;
    s->Kd = 0.08;
}

/**
 * ATTCTRL_compute — PD torque command to reaction wheel.
 *
 * Mirrors Python AttitudeController.compute():
 *   q_err   = quat_error(q_ref, q_est)
 *   torque  = Kp * q_err[1:] + Kd * omega_est
 *
 * q_est[4]     : estimated quaternion [w,x,y,z] (from MEKF)
 * omega_est[3] : estimated angular rate [rad/s] (from MEKF bias-corrected)
 * q_ref[4]     : reference (target) quaternion [w,x,y,z]
 * torque_out[3]: commanded torque TO the reaction wheel [N·m]
 * q_err_out[4] : quaternion error (may be NULL)
 *
 * Sign convention (matches spacecraft.py):
 *   Wheel stores +torque_out → reaction on body is -torque_out
 *   PD law drives error to zero.
 */
void ATTCTRL_compute(const AttCtrl_State *s,
                     const double q_est[4],
                     const double omega_est[3],
                     const double q_ref[4],
                     double torque_out[3],
                     double q_err_out[4]);   /* may be NULL */

#endif /* ADCS_H */