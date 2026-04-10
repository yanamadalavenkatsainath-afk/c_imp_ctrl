/**
 * rpod_ctrl.c — PROX_OPS and TERMINAL guidance
 * Direct port of lambert_controller.py _prox_ops() and _terminal()
 */

#include "rpod_ctrl.h"
#include <math.h>
#include <string.h>

/* Velocity profile — matches PROX_V_PROFILE in lambert_controller.py */
static const RPOD_ProfileEntry _profile[RPOD_PROFILE_LEN] = {
    {500.0, 0.200},
    {200.0, 0.100},
    {100.0, 0.060},
    { 50.0, 0.030},
    { 20.0, 0.015},
    { 10.0, 0.010},
    {  5.0, 0.005},
    {  2.0, 0.003},
};

/* ── Internal helpers ─────────────────────────────────────────── */

static double vec3_norm(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void vec3_scale(const double v[3], double s, double out[3]) {
    out[0]=v[0]*s; out[1]=v[1]*s; out[2]=v[2]*s;
}

static void clamp_accel(double a[3], double accel_max) {
    double mag = vec3_norm(a);
    if (mag > accel_max) {
        double s = accel_max / mag;
        a[0]*=s; a[1]*=s; a[2]*=s;
    }
}

/* ── RPOD_prox_ops ────────────────────────────────────────────── */

int RPOD_prox_ops(const RPOD_State *state,
                  double truth_range,
                  double n_chief,
                  double accel_max,
                  double accel_out[3]) {

    /* Transition to TERMINAL */
    if (truth_range < RPOD_TERMINAL_M) {
        return RPOD_terminal(state, accel_max, accel_out);
    }

    /* Deadband — very close, coast */
    if (truth_range < 0.05) {
        accel_out[0] = accel_out[1] = accel_out[2] = 0.0;
        return 0;
    }

    const double *pos = state->pos;
    const double *vel = state->vel;

    /* Desired closing speed from profile — walk reversed to find
       the tightest threshold that truth_range is below.
       Mirrors Python: reversed(PROX_V_PROFILE) */
    double v_close = _profile[0].v_close_ms;   /* default: largest range */
    for (int i = RPOD_PROFILE_LEN - 1; i >= 0; i--) {
        if (truth_range <= _profile[i].range_m) {
            v_close = _profile[i].v_close_ms;
            break;
        }
    }

    /* Unit vector toward chief (origin) */
    double rng_pos = vec3_norm(pos);
    double pos_hat[3];
    if (rng_pos < 1e-3) {
        pos_hat[0]=0.0; pos_hat[1]=-1.0; pos_hat[2]=0.0;
    } else {
        pos_hat[0]=pos[0]/rng_pos;
        pos_hat[1]=pos[1]/rng_pos;
        pos_hat[2]=pos[2]/rng_pos;
    }

    /* vel_des = -pos_hat * v_close */
    double vel_des[3];
    vec3_scale(pos_hat, -v_close, vel_des);

    /* vel_accel = -(vel - vel_des) / PROX_TAU */
    double accel[3];
    accel[0] = -(vel[0] - vel_des[0]) / RPOD_PROX_TAU;
    accel[1] = -(vel[1] - vel_des[1]) / RPOD_PROX_TAU;
    accel[2] = -(vel[2] - vel_des[2]) / RPOD_PROX_TAU;

    /* Weak position correction toward origin — mirrors Python omega_pos term */
    double omega_pos = 0.5 * n_chief;
    double omega2    = omega_pos * omega_pos;
    accel[0] -= omega2 * pos[0];
    accel[1] -= omega2 * pos[1];
    accel[2] -= omega2 * pos[2];

    clamp_accel(accel, accel_max);
    accel_out[0]=accel[0]; accel_out[1]=accel[1]; accel_out[2]=accel[2];
    return 0;
}

/* ── RPOD_terminal ────────────────────────────────────────────── */

int RPOD_terminal(const RPOD_State *state,
                  double accel_max,
                  double accel_out[3]) {

    const double *pos = state->pos;
    const double *vel = state->vel;

    /* Fresh range — always recompute, never trust caller value */
    double fresh_range = vec3_norm(pos);

    /* Docking condition */
    if (fresh_range < 0.05) {
        accel_out[0] = accel_out[1] = accel_out[2] = 0.0;
        return 1;
    }

    /* Unit vector toward chief */
    double pos_hat[3];
    if (fresh_range < 1e-3) {
        pos_hat[0]=0.0; pos_hat[1]=-1.0; pos_hat[2]=0.0;
    } else {
        pos_hat[0]=pos[0]/fresh_range;
        pos_hat[1]=pos[1]/fresh_range;
        pos_hat[2]=pos[2]/fresh_range;
    }

    /* Speed law: v_des = k * range, capped at VMAX
       Matches Python: v_des_mag = min(k * fresh_range, 0.005) */
    double v_des_mag = RPOD_TERMINAL_K * fresh_range;
    if (v_des_mag > RPOD_TERMINAL_VMAX) v_des_mag = RPOD_TERMINAL_VMAX;

    /* vel_des = -pos_hat * v_des_mag */
    double vel_des[3];
    vec3_scale(pos_hat, -v_des_mag, vel_des);

    /* accel = (vel_des - vel) / 5.0  — matches Python time constant */
    double accel[3];
    accel[0] = (vel_des[0] - vel[0]) / 5.0;
    accel[1] = (vel_des[1] - vel[1]) / 5.0;
    accel[2] = (vel_des[2] - vel[2]) / 5.0;

    clamp_accel(accel, accel_max);
    accel_out[0]=accel[0]; accel_out[1]=accel[1]; accel_out[2]=accel[2];
    return 0;
}