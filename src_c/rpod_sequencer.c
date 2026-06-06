#include "rpod_sequencer.h"
#include "lambert_solver.h"
#include "sim_config.h"
#include <math.h>
#include <string.h>

static double norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void zero3(double v[3]) {
    v[0] = 0.0; v[1] = 0.0; v[2] = 0.0;
}

static void copy3(const double a[3], double b[3]) {
    b[0] = a[0]; b[1] = a[1]; b[2] = a[2];
}

static void set_mode(RPODSequencer *seq, RPODSeqMode mode, double t_s) {
    if (seq->mode == mode) return;
    seq->mode = mode;
    seq->mode_entry_t_s = t_s;
    if (mode != RPOD_SEQ_LAMBERT) {
        seq->lambert_active = 0;
        zero3(seq->lambert_dv2_lvlh);
    }
}

static void R_l2e(const double r[3], const double v[3], double R[3][3]) {
    double xh[3], yh[3], zh[3];
    double rn = norm3(r);
    if (rn < 1.0) {
        R[0][0]=1.0; R[0][1]=0.0; R[0][2]=0.0;
        R[1][0]=0.0; R[1][1]=1.0; R[1][2]=0.0;
        R[2][0]=0.0; R[2][1]=0.0; R[2][2]=1.0;
        return;
    }
    xh[0]=r[0]/rn; xh[1]=r[1]/rn; xh[2]=r[2]/rn;
    zh[0]=r[1]*v[2]-r[2]*v[1];
    zh[1]=r[2]*v[0]-r[0]*v[2];
    zh[2]=r[0]*v[1]-r[1]*v[0];
    double zn = norm3(zh);
    if (zn < 1e-12) {
        zh[0]=0.0; zh[1]=0.0; zh[2]=1.0;
    } else {
        zh[0]/=zn; zh[1]/=zn; zh[2]/=zn;
    }
    yh[0]=zh[1]*xh[2]-zh[2]*xh[1];
    yh[1]=zh[2]*xh[0]-zh[0]*xh[2];
    yh[2]=zh[0]*xh[1]-zh[1]*xh[0];

    R[0][0]=xh[0]; R[0][1]=yh[0]; R[0][2]=zh[0];
    R[1][0]=xh[1]; R[1][1]=yh[1]; R[1][2]=zh[1];
    R[2][0]=xh[2]; R[2][1]=yh[2]; R[2][2]=zh[2];
}

static void mat_t_vec(const double R[3][3], const double v[3], double out[3]) {
    out[0] = R[0][0]*v[0] + R[1][0]*v[1] + R[2][0]*v[2];
    out[1] = R[0][1]*v[0] + R[1][1]*v[1] + R[2][1]*v[2];
    out[2] = R[0][2]*v[0] + R[1][2]*v[1] + R[2][2]*v[2];
}

static void mat_vec(const double R[3][3], const double v[3], double out[3]) {
    out[0] = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    out[1] = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    out[2] = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
}

void RPODSeq_init(RPODSequencer *seq) {
    memset(seq, 0, sizeof(*seq));
    seq->mode = RPOD_SEQ_FORMATION_HOLD;
    seq->last_plan_t_s = -1e9;
}

void RPODSeq_start_rendezvous(RPODSequencer *seq, double t_s, double range_m) {
    if (range_m > CFG_PROX_FAR_FIELD_M) set_mode(seq, RPOD_SEQ_LAMBERT, t_s);
    else set_mode(seq, RPOD_SEQ_PROX_OPS, t_s);
}

static void formation_hold_accel(const RPODSeqInput *in, double accel[3]) {
    RPOD_FormHoldState fh;
    for (int i = 0; i < 3; ++i) {
        fh.pos[i] = in->nav_lvlh[i];
        fh.vel[i] = in->nav_lvlh[3+i];
    }
    fh.n_chief = in->n_chief;
    fh.accel_max = in->accel_max;
    RPOD_formation_hold(&fh, accel);
}

static int plan_lambert(RPODSequencer *seq,
                        const RPODSeqInput *in,
                        RPODSeqOutput *out) {
    static const double tofs[CFG_LAMBERT_TOF_CANDIDATE_COUNT] = {
        3600.0, 5400.0, 7200.0, 9000.0, 10800.0, 14400.0, 21600.0
    };
    double mu = in->mu > 0.0 ? in->mu : CFG_MU_GEO;
    double R[3][3];
    R_l2e(in->chief_pos_eci, in->chief_vel_eci, R);

    double dep_pos_eci[3], rel_pos_eci[3];
    double dep_vel_eci[3], rel_vel_eci[3];
    mat_vec(R, in->nav_lvlh, rel_pos_eci);
    mat_vec(R, &in->nav_lvlh[3], rel_vel_eci);
    for (int i = 0; i < 3; ++i) {
        dep_pos_eci[i] = in->chief_pos_eci[i] + rel_pos_eci[i];
        dep_vel_eci[i] = in->chief_vel_eci[i] + rel_vel_eci[i];
    }

    double best_total = HUGE_VAL;
    double best_tof = 0.0;
    double best_dv1_lvlh[3] = {0.0, 0.0, 0.0};
    double best_dv2_lvlh[3] = {0.0, 0.0, 0.0};

    for (int k = 0; k < CFG_LAMBERT_TOF_CANDIDATE_COUNT; ++k) {
        double tof = tofs[k];
        double chi_arr[3], chi_vel_arr[3];
        LAMBERT_propagate_keplerian(in->chief_pos_eci, in->chief_vel_eci,
                                    tof, 200, mu, chi_arr, chi_vel_arr);
        Lambert_Result lr = LAMBERT_solve(dep_pos_eci, chi_arr, tof, 1, mu);
        if (!lr.ok) continue;

        double dv1_eci[3], dv2_eci[3];
        double m1 = 0.0, m2 = 0.0;
        for (int i = 0; i < 3; ++i) {
            dv1_eci[i] = lr.v1[i] - dep_vel_eci[i];
            dv2_eci[i] = chi_vel_arr[i] - lr.v2[i];
            m1 += dv1_eci[i]*dv1_eci[i];
            m2 += dv2_eci[i]*dv2_eci[i];
        }
        m1 = sqrt(m1);
        m2 = sqrt(m2);
        if (m1 > CFG_LAMBERT_DV_CAP_MS || m2 > CFG_LAMBERT_DV_CAP_MS) continue;

        double dep_arr[3], dep_arr_vel[3], rel_arr_eci[3], rel_arr_lvlh[3];
        LAMBERT_propagate_keplerian(dep_pos_eci, lr.v1, tof, 200, mu,
                                    dep_arr, dep_arr_vel);
        for (int i = 0; i < 3; ++i) rel_arr_eci[i] = dep_arr[i] - chi_arr[i];
        double R_arr[3][3];
        R_l2e(chi_arr, chi_vel_arr, R_arr);
        mat_t_vec(R_arr, rel_arr_eci, rel_arr_lvlh);
        if (norm3(rel_arr_lvlh) > CFG_LAMBERT_ARRIVAL_TOL_M) continue;

        double total = m1 + m2;
        if (total < best_total) {
            best_total = total;
            best_tof = tof;
            mat_t_vec(R, dv1_eci, best_dv1_lvlh);
            mat_t_vec(R_arr, dv2_eci, best_dv2_lvlh);
        }
    }

    if (!isfinite(best_total)) return 0;

    seq->lambert_active = 1;
    seq->lambert_burn2_t_s = in->t_s + best_tof;
    copy3(best_dv2_lvlh, seq->lambert_dv2_lvlh);
    copy3(best_dv1_lvlh, out->impulse_dv_lvlh);
    out->has_impulse = 1;
    out->plan_ok = 1;
    return 1;
}

void RPODSeq_step(RPODSequencer *seq,
                  const RPODSeqInput *in,
                  RPODSeqOutput *out) {
    memset(out, 0, sizeof(*out));
    out->mode = seq->mode;
    out->lambert_active = seq->lambert_active;

    double range = norm3(in->nav_lvlh);

    if (seq->mode == RPOD_SEQ_FORMATION_HOLD) {
        formation_hold_accel(in, out->accel_lvlh);
        out->mode = seq->mode;
        return;
    }

    if (seq->mode == RPOD_SEQ_PROX_OPS) {
        RPOD_State state;
        for (int i = 0; i < 3; ++i) {
            state.pos[i] = in->nav_lvlh[i];
            state.vel[i] = in->nav_lvlh[3+i];
        }
        RPOD_prox_ops(&state, range, in->n_chief, in->accel_max, out->accel_lvlh);
        out->mode = seq->mode;
        return;
    }

    if (seq->mode == RPOD_SEQ_LAMBERT) {
        if (range < CFG_PROX_FAR_FIELD_M) {
            set_mode(seq, RPOD_SEQ_PROX_OPS, in->t_s);
            RPODSeq_step(seq, in, out);
            return;
        }

        if (seq->lambert_active && in->t_s >= seq->lambert_burn2_t_s) {
            copy3(seq->lambert_dv2_lvlh, out->impulse_dv_lvlh);
            out->has_impulse = 1;
            seq->lambert_active = 0;
            zero3(seq->lambert_dv2_lvlh);
            if (range <= CFG_PROX_FAR_FIELD_M) set_mode(seq, RPOD_SEQ_PROX_OPS, in->t_s);
            out->mode = seq->mode;
            return;
        }

        if (!seq->lambert_active) {
            if (!in->chief_eci_valid ||
                in->t_s - seq->last_plan_t_s < CFG_LAMBERT_PLAN_COOLDOWN_S) {
                formation_hold_accel(in, out->accel_lvlh);
                out->mode = seq->mode;
                return;
            }
            seq->last_plan_t_s = in->t_s;
            seq->replan_count++;
            if (!plan_lambert(seq, in, out)) {
                formation_hold_accel(in, out->accel_lvlh);
            }
            out->mode = seq->mode;
            out->lambert_active = seq->lambert_active;
            return;
        }

        zero3(out->accel_lvlh);
        out->mode = seq->mode;
    }
}
