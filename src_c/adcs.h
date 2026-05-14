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
 * BDOT_init — gains tuned for GEO servicer plant (I=4.167 kg·m², B=4e-5 T).
 *
 * Required detumble from 24.66 deg/s within 300s:
 *   tau_needed = I * omega / ts = 4.167 * 0.43 / 150 = 0.012 N·m
 *   m_max >= tau_needed / B = 0.012 / 4e-5 = 300 A·m²
 *
 * k_bdot large enough to saturate at any omega > 0.01 rad/s:
 *   m_sat when k_bdot * omega * B > m_max
 *   → k_bdot > m_max / (omega_min * B) = 300 / (0.01 * 4e-5) = 7.5e8
 *   Use 1e9 to guarantee saturation throughout detumble.
 */
static inline void BDOT_init(BDot_State *s) {
    s->k_bdot = 1e9;
    s->m_max  = 300.0;
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
    s->h_max = 4.0;    /* matches Python ReactionWheel(h_max=4.0) N·m·s */
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
    double Kp;   /* proportional gain — matches Python Kp=0.08284 */
    double Kd;   /* derivative gain   — matches Python Kd=0.82257 */
} AttCtrl_State;

/**
 * ATTCTRL_init — gains tuned for GEO servicer plant.
 *
 * Plant: I = 4.167 kg·m², tau_max = 2mN·m (wheel clamp).
 *
 *   wn   = sqrt(Kp/I) → Kp = wn² * I
 *   zeta = Kd/(2*wn*I) = 0.9  →  Kd = 2*0.9*wn*I
 *
 * Constraints:
 *   (a) tau at max error (q_err_vec≈1) <= tau_max:  Kp <= 2e-3
 *   (b) h saturates in >> settling time:  h_max/Kp >> 4/(zeta*wn)
 *       4.0/0.002 = 2000s >> 203s  ✓
 *
 * Kp=0.002 → wn=0.0219 rad/s, ts≈203s, zeta=0.9
 * Old Kp=0.08284 caused h saturation in 48s → controller lost authority.
 */
static inline void ATTCTRL_init(AttCtrl_State *s) {
    s->Kp = 0.002;
    s->Kd = 0.1643;
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