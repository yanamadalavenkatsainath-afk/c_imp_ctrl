/**
 * rpod_ctrl.c — RPOD guidance (PROX_OPS + TERMINAL + LOST_TARGET + FORMATION_HOLD)
 * ==================================================================================
 * Updated to match lambert_controller.py exactly:
 *
 *  PROX_OPS  : sqrt closing-speed law (was step-profile table)
 *  TERMINAL  : port targeting, TAU scheduling, entry brake, EKF spike guard
 *  LOST_TARGET: velocity null when camera is lost
 *  FORMATION_HOLD: PD to standoff point
 */

#include "rpod_ctrl.h"
#include "capture_gate.h"
#include <math.h>
#include <string.h>

/* ── Internal helpers ─────────────────────────────────────────── */

static double vec3_norm(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void vec3_scale(const double v[3], double s, double out[3]) {
    out[0] = v[0]*s; out[1] = v[1]*s; out[2] = v[2]*s;
}

static void vec3_sub(const double a[3], const double b[3], double out[3]) {
    out[0] = a[0]-b[0]; out[1] = a[1]-b[1]; out[2] = a[2]-b[2];
}

static double vec3_dot(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void clamp_accel(double a[3], double accel_max) {
    double mag = vec3_norm(a);
    if (mag > accel_max && mag > 0.0) {
        double s = accel_max / mag;
        a[0] *= s; a[1] *= s; a[2] *= s;
    }
}

static int attitude_gate_ok(const RPOD_TermState *state) {
    if (!state->has_attitude_align) return 1;
    return state->attitude_align_deg <= RPOD_DOCK_ALIGN_MAX_DEG;
}

static int geometry_gate_ok(const RPOD_TermState *state) {
    if (!state->has_geometry) return 1;
    return state->geometry_ok ? 1 : 0;
}

static int body_clear_ok(const RPOD_TermState *state) {
    if (!state->has_geometry) return 1;
    return state->body_clear ? 1 : 0;
}

static int capture_core_ok(const RPOD_TermState *state, double port_range) {
    if (state->has_geometry) return state->capture_core ? 1 : 0;
    return port_range <= RPOD_DOCK_CONE_MIN_RANGE_M;
}

static void fill_capture_gate(const RPOD_TermState *state,
                              double port_range,
                              double port_vrel,
                              CaptureGateIn *in) {
    CaptureGate_default(in);
    in->port_range_m = port_range;
    in->port_vrel_ms = port_vrel;
    in->align_deg = state->has_attitude_align ? state->attitude_align_deg : -1.0;
    in->body_clear = body_clear_ok(state);
    in->capture_core = capture_core_ok(state, port_range);
    in->geometry_ok = geometry_gate_ok(state);
    in->align_ok = attitude_gate_ok(state);
}

void RPOD_fill_geometry(RPOD_TermState *state) {
    state->has_geometry = 0;
    state->geometry_ok = 1;
    state->body_clear = 1;
    state->capture_core = 0;
    state->cone_angle_deg = 0.0;
    state->cone_error_deg = 0.0;
    state->lateral_m = 0.0;

    if (!state->has_port) return;

    double axis[3] = {
        state->port_axis_lvlh[0],
        state->port_axis_lvlh[1],
        state->port_axis_lvlh[2]
    };
    double axis_n = vec3_norm(axis);
    if (axis_n < 1e-9) {
        axis[0] = 0.0; axis[1] = 0.0; axis[2] = 1.0;
        axis_n = 1.0;
    }
    vec3_scale(axis, 1.0 / axis_n, axis);

    double port_to_dep[3];
    double axial = 0.0;
    double lateral = 0.0;
    int inside_body = 0;
    int on_dock_face = 1;

    if (state->has_body_R) {
        double dep_body[3];
        for (int i = 0; i < 3; ++i) {
            dep_body[i] = state->R_body_to_lvlh[0][i] * state->pos[0]
                        + state->R_body_to_lvlh[1][i] * state->pos[1]
                        + state->R_body_to_lvlh[2][i] * state->pos[2];
        }

        double axis_body[3] = {
            CFG_DOCK_AXIS_BODY_X,
            CFG_DOCK_AXIS_BODY_Y,
            CFG_DOCK_AXIS_BODY_Z
        };
        double axis_body_n = vec3_norm(axis_body);
        if (axis_body_n < 1e-9) {
            axis_body[0] = 0.0; axis_body[1] = 0.0; axis_body[2] = 1.0;
            axis_body_n = 1.0;
        }
        vec3_scale(axis_body, 1.0 / axis_body_n, axis_body);

        double port_body[3] = {
            CFG_DOCK_PORT_BODY_X_M,
            CFG_DOCK_PORT_BODY_Y_M,
            CFG_DOCK_PORT_BODY_Z_M
        };
        double port_to_dep_body[3];
        vec3_sub(dep_body, port_body, port_to_dep_body);
        axial = vec3_dot(port_to_dep_body, axis_body);
        double lat_body[3] = {
            port_to_dep_body[0] - axial * axis_body[0],
            port_to_dep_body[1] - axial * axis_body[1],
            port_to_dep_body[2] - axial * axis_body[2]
        };
        lateral = vec3_norm(lat_body);
        port_to_dep[0] = port_to_dep_body[0];
        port_to_dep[1] = port_to_dep_body[1];
        port_to_dep[2] = port_to_dep_body[2];

        inside_body = (fabs(dep_body[0]) < CFG_CHIEF_BODY_HALF_X_M &&
                       fabs(dep_body[1]) < CFG_CHIEF_BODY_HALF_Y_M &&
                       fabs(dep_body[2]) < CFG_CHIEF_BODY_HALF_Z_M);
        on_dock_face = (dep_body[2] > CFG_CHIEF_BODY_HALF_Z_M - RPOD_DOCK_FACE_TOL_M);
    } else {
        vec3_sub(state->pos, state->port_lvlh, port_to_dep);
        axial = vec3_dot(port_to_dep, axis);
        double lat_vec[3] = {
            port_to_dep[0] - axial * axis[0],
            port_to_dep[1] - axial * axis[1],
            port_to_dep[2] - axial * axis[2]
        };
        lateral = vec3_norm(lat_vec);

        double dep_body_z = CFG_DOCK_PORT_BODY_Z_M + axial;
        double half_lat = (CFG_CHIEF_BODY_HALF_X_M > CFG_CHIEF_BODY_HALF_Y_M)
                        ? CFG_CHIEF_BODY_HALF_X_M : CFG_CHIEF_BODY_HALF_Y_M;
        inside_body = (lateral < half_lat &&
                       fabs(dep_body_z) < CFG_CHIEF_BODY_HALF_Z_M);
        on_dock_face = (dep_body_z > CFG_CHIEF_BODY_HALF_Z_M - RPOD_DOCK_FACE_TOL_M);
    }

    double range = vec3_norm(port_to_dep);
    if (range < 1e-9) {
        state->has_geometry = 1;
        state->geometry_ok = 1;
        state->body_clear = 1;
        state->capture_core = 1;
        return;
    }

    double cosang = axial / range;
    if (cosang > 1.0) cosang = 1.0;
    if (cosang < -1.0) cosang = -1.0;

    double angle_deg = acos(cosang) * 180.0 / 3.141592653589793;
    double cone_err = angle_deg - RPOD_DOCK_CONE_HALF_ANGLE_DEG;
    int capture_core = (range <= RPOD_DOCK_CONE_MIN_RANGE_M &&
                        lateral <= RPOD_DOCK_PORT_APERTURE_M);
    int cone_ok = ((axial > 0.0 && cone_err <= 0.0) || capture_core);
    int aperture_ok = lateral <= RPOD_DOCK_PORT_APERTURE_M;
    int body_clear = (!inside_body) || (on_dock_face && aperture_ok);

    state->cone_angle_deg = angle_deg;
    state->cone_error_deg = capture_core ? 0.0 : (cone_err > 0.0 ? cone_err : 0.0);
    state->lateral_m = lateral;
    state->has_geometry = 1;
    state->body_clear = body_clear ? 1 : 0;
    state->capture_core = capture_core ? 1 : 0;
    state->geometry_ok = (body_clear && cone_ok && aperture_ok) ? 1 : 0;
}

/* ── RPOD_prox_ops ────────────────────────────────────────────── */

int RPOD_prox_ops(const RPOD_State *state,
                  double truth_range,
                  double n_chief,
                  double accel_max,
                  double accel_out[3]) {

    /* ── Transition to TERMINAL ──────────────────────────────── */
    if (truth_range < RPOD_TERMINAL_M) {
        /* Delegate to simple terminal using RPOD_State (no port info) */
        return RPOD_terminal_simple(state, accel_max, accel_out);
    }

    /* ── Deadband — already within dock zone ─────────────────── */
    if (truth_range < RPOD_DOCK_DONE_M) {
        accel_out[0] = accel_out[1] = accel_out[2] = 0.0;
        return -1;
    }

    const double *pos = state->pos;
    const double *vel = state->vel;

    /*
     * EKF range drives both speed and direction (consistent with Python).
     * pos is the EKF position (passed in as state->pos).
     */
    double rng_ekf = vec3_norm(pos);
    double pos_hat[3];
    if (rng_ekf < 1e-3) {
        pos_hat[0] = 0.0; pos_hat[1] = -1.0; pos_hat[2] = 0.0;
    } else {
        vec3_scale(pos, 1.0 / rng_ekf, pos_hat);
    }

    /*
     * sqrt closing-speed law — mirrors Python _prox_ops():
     *   k_sqrt  = 0.200 / sqrt(500)
     *   v_close = k_sqrt * sqrt(max(rng_ekf, 0.1))
     *   v_close = min(v_close, 0.200)
     *   if rng_ekf < 10.0: v_close = min(v_close, 0.005)
     */
    double rng_eff  = rng_ekf > 0.1 ? rng_ekf : 0.1;
    double v_close  = RPOD_K_SQRT * sqrt(rng_eff);
    if (v_close > RPOD_V_CLOSE_MAX_MS) v_close = RPOD_V_CLOSE_MAX_MS;
    if (rng_ekf < RPOD_V_CLOSE_NEAR_RNG && v_close > RPOD_V_CLOSE_NEAR_MS)
        v_close = RPOD_V_CLOSE_NEAR_MS;

    /* vel_des = -pos_hat * v_close */
    double vel_des[3];
    vec3_scale(pos_hat, -v_close, vel_des);

    /* accel = -(vel - vel_des) / PROX_TAU */
    double accel[3];
    accel[0] = -(vel[0] - vel_des[0]) / RPOD_PROX_TAU;
    accel[1] = -(vel[1] - vel_des[1]) / RPOD_PROX_TAU;
    accel[2] = -(vel[2] - vel_des[2]) / RPOD_PROX_TAU;

    clamp_accel(accel, accel_max);
    accel_out[0] = accel[0];
    accel_out[1] = accel[1];
    accel_out[2] = accel[2];
    return 0;
}

/* ── RPOD_terminal ────────────────────────────────────────────── */

int RPOD_terminal(const RPOD_TermState *state,
                  double accel_max,
                  double accel_out[3],
                  int *is_braking) {

    const double *pos = state->pos;
    const double *vel = state->vel;

    double com_range = vec3_norm(pos);

    /* ── Docking complete ─────────────────────────────────────── */
    /* ── TAU gain scheduling — matches Python ─────────────────── */
    double TAU;
    if (com_range < 0.30)      TAU = RPOD_TAU_CLOSE;
    else if (com_range < 0.60) TAU = RPOD_TAU_MID;
    else                        TAU = RPOD_TAU_FAR;

    /* ── Entry brake state machine ────────────────────────────── */
    /* is_braking is an in/out flag managed by the caller.
     * -1 means "unset / first call" — evaluate entry condition. */
    double vel_mag = vec3_norm(vel);
    if (*is_braking == -1) {
        /* Fresh TERMINAL entry */
        *is_braking = (vel_mag > RPOD_BRAKE_ENTRY_MS) ? 1 : 0;
    }

    if (*is_braking) {
        /* Hard brake: accel = -vel / 1.0  (τ = 1s, same as Python) */
        double accel[3];
        accel[0] = -vel[0] / 1.0;
        accel[1] = -vel[1] / 1.0;
        accel[2] = -vel[2] / 1.0;
        clamp_accel(accel, accel_max);
        accel_out[0] = accel[0];
        accel_out[1] = accel[1];
        accel_out[2] = accel[2];

        if (vel_mag < RPOD_BRAKE_DONE_MS)
            *is_braking = 0;   /* brake complete */

        return -1;   /* braking */
    }

    /* ── EKF spike guard on port ──────────────────────────────── */
    /*
     * If port_lvlh is more than PORT_SANITY_M from deputy, fall back to CoM.
     * Mirrors Python: port = port_lvlh if _cand_range < PORT_SANITY_M else [0,0,0]
     */
    double port[3] = {0.0, 0.0, 0.0};
    double port_vel[3] = {0.0, 0.0, 0.0};
    if (state->has_port) {
        double diff[3];
        vec3_sub(state->port_lvlh, pos, diff);
        double cand_range = vec3_norm(diff);
        if (cand_range < RPOD_PORT_SANITY_M) {
            port[0] = state->port_lvlh[0];
            port[1] = state->port_lvlh[1];
            port[2] = state->port_lvlh[2];
            port_vel[0] = state->port_vel_lvlh[0];
            port_vel[1] = state->port_vel_lvlh[1];
            port_vel[2] = state->port_vel_lvlh[2];
        }
        /* else: leave port = {0,0,0} — target CoM */
    }

    /* port_range: distance from deputy to port (or to CoM if port=0) */
    double port_diff[3];
    vec3_sub(port, pos, port_diff);
    double port_range = vec3_norm(port_diff);
    double tgt_range = port_range;
    double rel_vel[3];
    vec3_sub(vel, port_vel, rel_vel);
    double rel_speed = vec3_norm(rel_vel);

    CaptureGateIn cg_in;
    CaptureGateOut cg_out;
    fill_capture_gate(state, tgt_range, rel_speed, &cg_in);
    CaptureGate_eval(&cg_in, &cg_out);

    /* ── Speed law (sqrt on com_range) — mirrors Python ─────── */
    /*
     * K_SQRT_TERM = V_TERM_MAX_MS / sqrt(0.8m)
     * v_des_mag   = min(K_SQRT_TERM * sqrt(max(com_range, 0.001)), V_TERM_MAX_MS)
     * Inside capture: v_des_mag = min(v_des_mag, V_CAPTURE_MS).
     * Keep closing until RPOD_DOCK_DONE_M; do not park at capture entry.
     */
    double rng_eff  = com_range > 0.001 ? com_range : 0.001;
    double v_des_mag = RPOD_K_SQRT_TERM * sqrt(rng_eff);
    if (v_des_mag > RPOD_V_TERM_MAX_MS) v_des_mag = RPOD_V_TERM_MAX_MS;
    if (tgt_range < RPOD_DOCK_RANGE_M && v_des_mag > RPOD_V_CAPTURE_MS)
        v_des_mag = RPOD_V_CAPTURE_MS;

    /* ── Target direction ─────────────────────────────────────── */
    double vel_des[3];
    if (port_range > 0.001) {
        /* Drive toward port */
        double tgt_hat[3];
        vec3_scale(port_diff, 1.0 / port_range, tgt_hat);
        vec3_scale(tgt_hat, v_des_mag, vel_des);
    } else {
        /* Port at same location as deputy — close on CoM */
        double com_hat[3];
        vec3_scale(pos, -1.0 / (com_range + 1e-9), com_hat);
        vec3_scale(com_hat, v_des_mag, vel_des);
    }
    vel_des[0] += port_vel[0];
    vel_des[1] += port_vel[1];
    vel_des[2] += port_vel[2];

    /* accel = (vel_des - vel) / TAU */
    double accel[3];
    accel[0] = (vel_des[0] - vel[0]) / TAU;
    accel[1] = (vel_des[1] - vel[1]) / TAU;
    accel[2] = (vel_des[2] - vel[2]) / TAU;

    clamp_accel(accel, accel_max);
    accel_out[0] = accel[0];
    accel_out[1] = accel[1];
    accel_out[2] = accel[2];

    if (cg_out.soft_capture_ready) return RPOD_RET_SOFT_CAPTURE_READY;
    return (tgt_range < RPOD_DOCK_RANGE_M) ? RPOD_RET_CAPTURE_ZONE : RPOD_RET_NORMAL;
}

int RPOD_soft_capture(const RPOD_TermState *state,
                      double accel_max,
                      double accel_out[3]) {
    double port[3] = {0.0, 0.0, 0.0};
    double port_vel[3] = {0.0, 0.0, 0.0};

    if (state->has_port) {
        port[0] = state->port_lvlh[0];
        port[1] = state->port_lvlh[1];
        port[2] = state->port_lvlh[2];
        port_vel[0] = state->port_vel_lvlh[0];
        port_vel[1] = state->port_vel_lvlh[1];
        port_vel[2] = state->port_vel_lvlh[2];
    }

    double err[3];
    vec3_sub(port, state->pos, err);
    double rel_vel[3];
    vec3_sub(state->vel, port_vel, rel_vel);

    accel_out[0] = RPOD_SOFT_CAPTURE_KP * err[0] - RPOD_SOFT_CAPTURE_KD * rel_vel[0];
    accel_out[1] = RPOD_SOFT_CAPTURE_KP * err[1] - RPOD_SOFT_CAPTURE_KD * rel_vel[1];
    accel_out[2] = RPOD_SOFT_CAPTURE_KP * err[2] - RPOD_SOFT_CAPTURE_KD * rel_vel[2];
    clamp_accel(accel_out, accel_max);

    double port_range = vec3_norm(err);
    double rel_speed = vec3_norm(rel_vel);
    CaptureGateIn cg_in;
    CaptureGateOut cg_out;
    fill_capture_gate(state, port_range, rel_speed, &cg_in);
    CaptureGate_eval(&cg_in, &cg_out);

    if (cg_out.hard_capture_ready || cg_out.soft_capture_certified) {
        return RPOD_RET_DOCKED;
    }
    return RPOD_RET_CAPTURE_ZONE;
}

/* ── RPOD_lost_target ─────────────────────────────────────────── */

void RPOD_lost_target(const RPOD_State *state,
                      double accel_max,
                      double accel_out[3]) {
    /*
     * Mirrors Python _lost_target():
     *   if |vel| > 5mm/s: accel = -vel / 2.0, clamped
     *   else:             accel = 0  (already nearly stopped)
     */
    const double *vel = state->vel;
    double vel_mag = vec3_norm(vel);

    if (vel_mag > 0.005) {
        accel_out[0] = -vel[0] / 2.0;
        accel_out[1] = -vel[1] / 2.0;
        accel_out[2] = -vel[2] / 2.0;
        clamp_accel(accel_out, accel_max);
    } else {
        accel_out[0] = accel_out[1] = accel_out[2] = 0.0;
    }
}

/* ── RPOD_formation_hold ──────────────────────────────────────── */

void RPOD_formation_hold(const RPOD_FormHoldState *state,
                         double accel_out[3]) {
    /*
     * Mirrors Python _formation_hold():
     *   target   = [0, -standoff, 0]   (1000m behind in LVLH)
     *   omega    = 0.05 * n_chief
     *   Kp       = omega^2
     *   Kd       = 2 * 0.8 * omega   (zeta=0.8 critically damped)
     *   pos_err  = pos - target
     *   vel_err  = vel
     *   accel    = -Kp * pos_err - Kd * vel_err, clamped
     */
    double n  = state->n_chief;
    double omega_hold = 0.05 * n;
    double Kp = omega_hold * omega_hold;
    double Kd = 2.0 * 0.8 * omega_hold;

    double target[3] = {0.0, -RPOD_STANDOFF_M, 0.0};

    double pos_err[3], vel_err[3];
    vec3_sub(state->pos, target, pos_err);
    vel_err[0] = state->vel[0];
    vel_err[1] = state->vel[1];
    vel_err[2] = state->vel[2];

    accel_out[0] = -Kp * pos_err[0] - Kd * vel_err[0];
    accel_out[1] = -Kp * pos_err[1] - Kd * vel_err[1];
    accel_out[2] = -Kp * pos_err[2] - Kd * vel_err[2];

    clamp_accel(accel_out, state->accel_max);
}
/* ── RPOD_terminal_simple ─────────────────────────────────────── */
/*
 * 3-argument wrapper for test compatibility.
 * Converts RPOD_State to RPOD_TermState with no port info, then
 * calls RPOD_terminal() with an internal static is_braking flag.
 * State machine resets when vel_mag drops below BRAKE_DONE_MS.
 */
int RPOD_terminal_simple(const RPOD_State *state,
                          double accel_max,
                          double accel_out[3]) {
    static int _is_braking = -1;   /* -1 = unset (fresh entry) */

    RPOD_TermState ts;
    ts.pos[0] = state->pos[0];
    ts.pos[1] = state->pos[1];
    ts.pos[2] = state->pos[2];
    ts.vel[0] = state->vel[0];
    ts.vel[1] = state->vel[1];
    ts.vel[2] = state->vel[2];
    ts.port_lvlh[0] = 0.0;
    ts.port_lvlh[1] = 0.0;
    ts.port_lvlh[2] = 0.0;
    ts.port_axis_lvlh[0] = 0.0;
    ts.port_axis_lvlh[1] = 0.0;
    ts.port_axis_lvlh[2] = 1.0;
    ts.port_vel_lvlh[0] = 0.0;
    ts.port_vel_lvlh[1] = 0.0;
    ts.port_vel_lvlh[2] = 0.0;
    ts.attitude_align_deg = 0.0;
    ts.cone_angle_deg = 0.0;
    ts.cone_error_deg = 0.0;
    ts.lateral_m = 0.0;
    ts.has_port = 0;
    ts.has_body_R = 0;
    ts.has_attitude_align = 0;
    ts.has_geometry = 0;
    ts.geometry_ok = 1;
    ts.body_clear = 1;
    ts.capture_core = 0;

    return RPOD_terminal(&ts, accel_max, accel_out, &_is_braking);
}

