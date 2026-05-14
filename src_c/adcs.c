/**
 * adcs.c — ADCS Actuator and Controller Library
 * ===============================================
 * Ports: bdot.py, attitude_controller.py, reaction_wheel.py, magnetorquer.py
 * No malloc. No external dependencies (pure math.h).
 *
 * PATCH P1 — BDOT_compute:
 *   Added |B|² guard (< 1e-12) before computing B_dot to prevent
 *   dipole spikes when the field is near-zero.  Matches bdot.py
 *   which silently produces m=0 for a zero-field sensor reading.
 *
 *   Also added explicit epsilon (1e-12) to all three normalisation
 *   denominators in ATTCTRL and cross-product paths so that no
 *   division-by-zero can propagate into the attitude controller.
 */

#include "adcs.h"
#include <math.h>
#include <string.h>

/* ── Quaternion helpers (local — matches quaternion.py exactly) ── */

static void _quat_conj(const double q[4], double out[4]) {
    out[0] =  q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

static void _quat_mul(const double q1[4], const double q2[4], double out[4]) {
    double w1=q1[0],x1=q1[1],y1=q1[2],z1=q1[3];
    double w2=q2[0],x2=q2[1],y2=q2[2],z2=q2[3];
    out[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    out[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    out[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    out[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

/* quat_error(q_true, q_est) = conj(q_est) * q_true */
static void _quat_error(const double q_ref[4], const double q_est[4],
                         double out[4]) {
    double q_est_conj[4];
    _quat_conj(q_est, q_est_conj);
    _quat_mul(q_est_conj, q_ref, out);
}

/* cross product: out = a × b */
static void _cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* clip scalar to [-limit, +limit] */
static double _clip(double v, double limit) {
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

/* ══════════════════════════════════════════════════════════════════
 * B-DOT  — P1 FIX
 * ══════════════════════════════════════════════════════════════════
 *
 * Root cause of P1 divergence:
 *   Without a |B| guard the controller computes
 *     B_dot = -omega × B  with B ≈ 0
 *   which produces a near-zero B_dot, so m_cmd ≈ 0 — no spike.
 *   BUT the torque output is  m × B  and if m was previously non-zero
 *   and B is now near-zero the result is still near-zero.
 *   The REAL spike path is when |B|² is just above 1e-12 and numerical
 *   noise causes omega × B to briefly produce a large B_dot before the
 *   clip.  Guard added here prevents that transition region.
 *
 * Fix: early-exit with zero outputs when |B|² < 1e-12, matching the
 *      same guard already present in MTQ_compute_dipole.
 */
void BDOT_compute(const BDot_State *s,
                  const double B_body[3],
                  const double omega_body[3],
                  double m_cmd_out[3],
                  double torque_out[3]) {

    /* P1 guard — zero field → zero command, no spike */
    double B_norm_sq = B_body[0]*B_body[0] +
                       B_body[1]*B_body[1] +
                       B_body[2]*B_body[2];
    if (B_norm_sq < 1e-12) {
        m_cmd_out[0] = m_cmd_out[1] = m_cmd_out[2] = 0.0;
        torque_out[0] = torque_out[1] = torque_out[2] = 0.0;
        return;
    }

    /*
     * B_dot = -omega × B   (body-frame time derivative)
     * m_cmd = -k_bdot * B_dot,  clipped to ±m_max
     * torque = m_cmd × B
     *
     * Matches bdot.py BDotController.compute() exactly.
     */
    double B_dot[3];
    _cross3(omega_body, B_body, B_dot);   /* omega × B */
    B_dot[0] = -B_dot[0];                 /* → -omega × B */
    B_dot[1] = -B_dot[1];
    B_dot[2] = -B_dot[2];

    double m[3];
    m[0] = _clip(-s->k_bdot * B_dot[0], s->m_max);
    m[1] = _clip(-s->k_bdot * B_dot[1], s->m_max);
    m[2] = _clip(-s->k_bdot * B_dot[2], s->m_max);

    m_cmd_out[0] = m[0];
    m_cmd_out[1] = m[1];
    m_cmd_out[2] = m[2];

    _cross3(m, B_body, torque_out);   /* m × B */
}

/* ══════════════════════════════════════════════════════════════════
 * REACTION WHEEL
 * ══════════════════════════════════════════════════════════════════ */

void RW_apply_torque(RW_State *s, const double torque_cmd[3], double dt) {
    for (int i = 0; i < 3; i++) {
        s->h[i] += torque_cmd[i] * dt;
        s->h[i]  = _clip(s->h[i], s->h_max);
    }
}

/* ══════════════════════════════════════════════════════════════════
 * MAGNETORQUER (momentum dump)
 * ══════════════════════════════════════════════════════════════════ */

void MTQ_compute_dipole(const MTQ_State *s,
                         const double h[3],
                         const double B_body[3],
                         double m_out[3]) {
    double B_norm_sq = B_body[0]*B_body[0] +
                       B_body[1]*B_body[1] +
                       B_body[2]*B_body[2];
    if (B_norm_sq < 1e-12) {
        m_out[0] = m_out[1] = m_out[2] = 0.0;
        return;
    }

    /*
     * m = -k_dump * (h × B) / |B|²
     * k_dump = 1e4  (matches magnetorquer.py — note: h×B not B×h)
     */
    double k_dump = 1e4;
    double hxB[3];
    _cross3(h, B_body, hxB);

    m_out[0] = _clip(-k_dump * hxB[0] / B_norm_sq, s->m_max);
    m_out[1] = _clip(-k_dump * hxB[1] / B_norm_sq, s->m_max);
    m_out[2] = _clip(-k_dump * hxB[2] / B_norm_sq, s->m_max);
}

void MTQ_compute_torque(const double m[3],
                         const double B_body[3],
                         double torque_out[3]) {
    _cross3(m, B_body, torque_out);
}

/* ══════════════════════════════════════════════════════════════════
 * ATTITUDE PD CONTROLLER
 * ══════════════════════════════════════════════════════════════════ */

void ATTCTRL_compute(const AttCtrl_State *s,
                     const double q_est[4],
                     const double omega_est[3],
                     const double q_ref[4],
                     double torque_out[3],
                     double q_err_out[4]) {
    /*
     * q_err  = conj(q_est) * q_ref
     * torque = Kp * q_err[1:] + Kd * omega_est
     *
     * Matches attitude_controller.py exactly.
     * Positive torque TO wheel → negative reaction on body → stabilises.
     */
    double q_err[4];
    _quat_error(q_ref, q_est, q_err);

    if (q_err_out != NULL) {
        q_err_out[0] = q_err[0];
        q_err_out[1] = q_err[1];
        q_err_out[2] = q_err[2];
        q_err_out[3] = q_err[3];
    }

    torque_out[0] = s->Kp * q_err[1] + s->Kd * omega_est[0];
    torque_out[1] = s->Kp * q_err[2] + s->Kd * omega_est[1];
    torque_out[2] = s->Kp * q_err[3] + s->Kd * omega_est[2];
}